#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;

class DoosanWaypointFollow : public rclcpp::Node
{
public:
    DoosanWaypointFollow()
    : Node("doosan_waypoint_follow")
    {
        file_path_ = declare_parameter<std::string>("file_path", "/home/nvidia/waypoints/scenario_one.csv");
        wait_server_sec_ = declare_parameter<double>("wait_server_sec", 5.0);
        goal_timeout_sec_ = declare_parameter<double>("goal_timeout_sec", 0.0);
        stop_on_failure_ = declare_parameter<bool>("stop_on_failure", false);

        
        client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
        srv_run_ = create_service<std_srvs::srv::Trigger>(
      "run_waypoints",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        if (!ensureServer()) {
          res->success = false;
          res->message = "follow_waypoints action server not available";
          return;
        }

        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        std::string msg;
        if (!loadCSV(file_path_, waypoints, msg)) {
          res->success = false;
          res->message = "Failed to load waypoints: " + msg;
          return;
        }
        if (waypoints.empty()) {
          res->success = false;
          res->message = "No waypoints in file";
          return;
        }

        auto goal = FollowWaypoints::Goal();
        goal.poses = waypoints;

        RCLCPP_INFO(get_logger(), "Sending %zu waypoints (stop_on_failure=%s)",
                    goal.poses.size(), stop_on_failure_ ? "true" : "false");

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions{};
        send_goal_options.result_callback =
          [this](const rclcpp_action::ClientGoalHandle<FollowWaypoints>::WrappedResult & result)
          {
            switch (result.code) {
              case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(), "Waypoint following succeeded");
                break;
              case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "Waypoint following aborted");
                break;
              case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(get_logger(), "Waypoint following canceled");
                break;
              default:
                RCLCPP_ERROR(get_logger(), "Unknown result code");
                break;
            }
          };

        auto future_handle = client_->async_send_goal(goal, send_goal_options);
        if (goal_timeout_sec_ > 0.0) {
          if (future_handle.wait_for(std::chrono::duration<double>(goal_timeout_sec_)) == std::future_status::timeout) {
            RCLCPP_ERROR(get_logger(), "Timeout waiting for goal acceptance");
            res->success = false;
            res->message = "Timeout waiting for goal acceptance";
            return;
          }
        } else {
          future_handle.wait();
        }

        auto handle = future_handle.get();
        if (!handle) {
          res->success = false;
          res->message = "Goal was rejected by server";
          return;
        }

        res->success = true;
        res->message = "Goal accepted. Following waypoints...";
      });

    RCLCPP_INFO(get_logger(), "ready. call ~/run_waypoints to start.");

    }

private:
static bool loadCSV(const std::string& path,
                      std::vector<geometry_msgs::msg::PoseStamped>& out,
                      std::string& msg)
  {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
      msg = "cannot open file: " + path;
      return false;
    }
    std::string line;
    size_t line_no = 0;
    while (std::getline(ifs, line)) {
      ++line_no;
      if (line.empty()) continue;
      std::stringstream ss(line);
      std::string sx, sy, syaw;
      if (!std::getline(ss, sx, ',')) continue;
      if (!std::getline(ss, sy, ',')) continue;
      if (!std::getline(ss, syaw, ',')) syaw = "0.0"; // yaw 없으면 0

      try {
        double x = std::stod(sx);
        double y = std::stod(sy);
        double yaw = std::stod(syaw);

        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = rclcpp::Time(0);
        p.header.frame_id = "map";
        p.pose.position.x = x;
        p.pose.position.y = y;
        p.pose.position.z = 0.0;

        // yaw -> quaternion
        double cy = std::cos(yaw * 0.5);
        double syq = std::sin(yaw * 0.5);
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = syq;
        p.pose.orientation.w = cy;

        out.emplace_back(std::move(p));
      } catch (const std::exception& e) {
        // 한 줄이 잘못됐어도 전체 실패로 보지 않고 스킵
        (void)e;
        continue;
      }
    }
    return true;
  }

  bool ensureServer() {
    const auto to = std::chrono::duration<double>(wait_server_sec_);
    if (!client_->wait_for_action_server(to)) {
      RCLCPP_ERROR(get_logger(), "follow_waypoints action server not available after %.1fs", wait_server_sec_);
      return false;
    }
    return true;
  }
    std::string file_path_;
    double wait_server_sec_;
    double goal_timeout_sec_;
    bool stop_on_failure_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_run_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr client_;

};