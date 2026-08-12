#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <string>
#include <functional>
#include <optional>

using sensor_msgs::msg::PointCloud2;
namespace mf = message_filters;

class CloudMergeNode : public rclcpp::Node
{
public:
    CloudMergeNode()
    : Node("cloud_merge_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
    {
        target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
        slop_sec_ = this->declare_parameter<double>("approximate_time_slop", 0.05);
        failover_sec_ = this->declare_parameter<double>("failover_timeout", 0.28);

        int qos_depth = this->declare_parameter<int>("qos_depth", 20);
        rclcpp::QoS qos{rclcpp::KeepLast(qos_depth)};
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        go2_lidar_topic_ = this->declare_parameter<std::string>(
            "go2_lidar_topic", "/go2/lidar_points");
        secondary_lidar_topic_ = this->declare_parameter<std::string>(
            "secondary_lidar_topic", "/lidar_points");

        go2_lidar_sub_ = std::make_shared<mf::Subscriber<PointCloud2>>(
            this, go2_lidar_topic_, qos.get_rmw_qos_profile());
        secondary_lidar_sub_ = std::make_shared<mf::Subscriber<PointCloud2>>(
            this, secondary_lidar_topic_, qos.get_rmw_qos_profile());

        using Policy = mf::sync_policies::ApproximateTime<PointCloud2, PointCloud2>;
        sync_ = std::make_shared<mf::Synchronizer<Policy>>(
            Policy(14), *go2_lidar_sub_, *secondary_lidar_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop_sec_));
        sync_->registerCallback(std::bind(&CloudMergeNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        raw_go2_lidar_sub_ = this->create_subscription<PointCloud2>(go2_lidar_topic_, qos,
            std::bind(&CloudMergeNode::rawGo2LidarCB, this, std::placeholders::_1));
        
        raw_secondary_lidar_sub_ = this->create_subscription<PointCloud2>(
            secondary_lidar_topic_, qos,
            std::bind(&CloudMergeNode::rawSecondaryLidarCB, this, std::placeholders::_1));

        point_pub_ = this->create_publisher<PointCloud2>("/merged/points", qos);

        RCLCPP_INFO(
            get_logger(),
            "cloud merger node started. target_frame=%s go2=%s secondary=%s slop=%.3fs",
            target_frame_.c_str(), go2_lidar_topic_.c_str(),
            secondary_lidar_topic_.c_str(), slop_sec_);
    }

private:
    std::shared_ptr<mf::Subscriber<PointCloud2>> go2_lidar_sub_, secondary_lidar_sub_;
    std::shared_ptr<mf::Synchronizer<mf::sync_policies::ApproximateTime<PointCloud2, PointCloud2>>> sync_;
    rclcpp::Subscription<PointCloud2>::SharedPtr raw_go2_lidar_sub_, raw_secondary_lidar_sub_;
    rclcpp::Publisher<PointCloud2>::SharedPtr point_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string target_frame_;
    std::string go2_lidar_topic_;
    std::string secondary_lidar_topic_;
    double slop_sec_{0.09};
    double failover_sec_{0.28};


    rclcpp::Time last_c1_time_{0, 0, RCL_ROS_TIME};  // 수정
    rclcpp::Time last_c2_time_{0, 0, RCL_ROS_TIME};  // 수정
    rclcpp::Time last_pub_time_{0, 0, RCL_ROS_TIME};

    void syncCallback(const PointCloud2::ConstSharedPtr& c1, const PointCloud2::ConstSharedPtr& c2)
    {
        // Use tf2::TimePointZero (= latest available transform) so that TF lookup
        // never fails due to sensor hardware-timestamp vs ROS-clock mismatch.
        // Static transforms (hesai_lidar → base_link, etc.) are always valid at any time.
        PointCloud2 c1_tf, c2_tf;
        try {
          auto tf1 = tf_buffer_.lookupTransform(target_frame_, c1->header.frame_id, tf2::TimePointZero);
          tf2::doTransform(*c1, c1_tf, tf1);

          auto tf2x = tf_buffer_.lookupTransform(target_frame_, c2->header.frame_id, tf2::TimePointZero);
          tf2::doTransform(*c2, c2_tf, tf2x);
        } catch (const tf2::TransformException& ex) {
          RCLCPP_WARN(get_logger(), "TF transform failed: %s", ex.what());
          return;
        }

        pcl::PointCloud<pcl::PointXYZI> p1i, p2i;
        if (!to_XYZI(c1_tf, p1i) || !to_XYZI(c2_tf, p2i)) {
            RCLCPP_WARN(get_logger(), "Failed to convert input clouds to XYZI");
            return;
        }
        p1i += p2i;

        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(p1i, out);
        out.header.frame_id = target_frame_;
        // Use current ROS time as the merged stamp so that:
        // 1. TF lookups downstream always succeed (stamp is in the TF buffer)
        // 2. The robot pose used by AMCL/RViz matches the actual moment of publication,
        //    eliminating the rotation error introduced by ApproximateTime slop (up to 90ms).
        out.header.stamp = this->now();
        point_pub_->publish(out);
        last_pub_time_ = this->now();
    }

    void rawGo2LidarCB(const PointCloud2::SharedPtr msg)
    {
        last_c1_time_ = rclcpp::Time(msg->header.stamp);
        maybePublishSingle(*msg, last_c2_time_);
    }

    void rawSecondaryLidarCB(const PointCloud2::SharedPtr msg)
    {
        last_c2_time_ = rclcpp::Time(msg->header.stamp);
        maybePublishSingle(*msg, last_c1_time_);
    }

    void maybePublishSingle(const PointCloud2& msg, const rclcpp::Time& other_last_time)
    {
        // if other stream is less than failover time
        if ((this->now() - other_last_time).seconds() < failover_sec_) return;

        // if the syncronized was published
        if ((this->now() - last_pub_time_).seconds() < 0.01) return;

        PointCloud2 tfed;
        if(!transformToTarget(msg, tfed)) return;
        tfed.header.stamp = this->now();
        last_pub_time_ = this->now();
        point_pub_->publish(tfed);
    }

    bool transformToTarget(const PointCloud2& in, PointCloud2& out) {
        try {
          // TimePointZero = latest available transform, avoids timestamp mismatch failures
          auto tf = tf_buffer_.lookupTransform(target_frame_, in.header.frame_id, tf2::TimePointZero);
          tf2::doTransform(in, out, tf);
          out.header.frame_id = target_frame_;
          return true;
        }catch (const tf2::TransformException& ex) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF failed %s", ex.what());
          return false;
        }
    }

    // check the field
    inline bool has_field(const PointCloud2& msg, const std::string& name)
    {
        return std::any_of(msg.fields.begin(), msg.fields.end(), [&](const auto& f){ return f.name == name; });
    }

    bool to_XYZI(const sensor_msgs::msg::PointCloud2& in, pcl::PointCloud<pcl::PointXYZI>& out)
    {
        
        if (!has_field(in, "x") || !has_field(in, "y") || !has_field(in, "z")) {
          return false;
        }
        const bool has_i = has_field(in, "intensity") || has_field(in, "i");
    
        if (has_i) {
          try {
            pcl::fromROSMsg(in, out);
            return true;
          } catch (...) {
          }
        } else {
        }
    
        sensor_msgs::PointCloud2ConstIterator<float> it_x(in, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(in, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(in, "z");
        std::optional<sensor_msgs::PointCloud2ConstIterator<float>> it_i;
    
        if (has_i) {
          if (has_field(in, "intensity")) {
            it_i.emplace(sensor_msgs::PointCloud2ConstIterator<float>(in, "intensity"));
          } else {
            it_i.emplace(sensor_msgs::PointCloud2ConstIterator<float>(in, "i"));
          }
        }
    
        out.clear();
        out.reserve(in.width * in.height);
    
        for (size_t idx = 0; idx < in.width * in.height; ++idx, ++it_x, ++it_y, ++it_z) {
          pcl::PointXYZI q;
          q.x = *it_x;
          q.y = *it_y;
          q.z = *it_z;
          if (it_i.has_value()) {
            q.intensity = **it_i;
            ++(*it_i);
          } else {
            q.intensity = 0.0f;
          }
          out.push_back(q);
        }
        return true;
    }

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudMergeNode>());
    rclcpp::shutdown();
    return 0;
}
