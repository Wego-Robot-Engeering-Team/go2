#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <fstream>
#include <filesystem>
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class WaypointSave : public rclcpp::Node
{
public:
    WaypointSave()
    :Node("waypoint_save"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
    {
        this->declare_parameter<std::string>("waypoint_file", "/home/wego/waypoints/waypoint_one.csv");
        this->declare_parameter<double>("tf_timeout", 0.2);
        file_path_ = this->get_parameter("waypoint_file").as_string();
        tf_timeout_ = this->get_parameter("tf_timeout").as_double();

        std::filesystem::path p = std::filesystem::path(file_path_).parent_path();
        if(!p.empty() && !std::filesystem::exists(p)){
            std::error_code ec;
            std::filesystem::create_directories(p, ec);
            if(ec){
                RCLCPP_WARN(get_logger(), "Failed to create directory: %s (%s)", p.string().c_str(), ec.message().c_str());
            }
        }

        srv_ = this->create_service<std_srvs::srv::Trigger>(
            "save_waypoint",
            std::bind(&WaypointSave::onSave, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(get_logger(), "SaveWaypointNode ready, Service: /save_waypoint");
        RCLCPP_INFO(get_logger(), " file: %s", file_path_.c_str());
    }

private:
    void onSave(const std::shared_ptr<std_srvs::srv::Trigger::Request> _req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> _res)
    {
        geometry_msgs::msg::TransformStamped tf;
        try{
            tf = tf_buffer_.lookupTransform(
                "map", "base_footprint", rclcpp::Time(0), rclcpp::Duration::from_seconds(tf_timeout_)
            );
        }catch(const std::exception& e){
            _res->success=false;
            _res->message=std::string("TF lookup failed: ") + e.what();
            RCLCPP_WARN(get_logger(), "%s", _res->message.c_str());
            return;
        }

        const auto& t = tf.transform.translation;
        const auto& q = tf.transform.rotation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        std::ofstream ofs(file_path_, std::ios_base::app);
        if(!ofs.is_open()){
            _res->success=false;
            _res->message="Fail to open file for append: " + file_path_;
            RCLCPP_ERROR(get_logger(), "%s", _res->message.c_str());
            return;
        }

        ofs << std::fixed
            << t.x << "," << t.y << "," <<yaw << "\n";
        ofs.close();

        _res->success = true;
        _res->message = "save waypoint";
    }

    std::string file_path_;
    double tf_timeout_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointSave>());
    rclcpp::shutdown();
    return 0;
}