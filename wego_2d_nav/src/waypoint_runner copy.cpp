#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <wego_2d_nav/srv/waypoint_task.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <atomic>
#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;

class WaypointRunner : public rclcpp::Node {
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleWaypointFollow = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
  using Wego2dNav = wego_2d_nav::srv::WaypointTask;

  WaypointRunner()
  : Node("waypoint_runner")
  {
    // ===== 파라미터 (CSV 파일 경로) =====
    this->declare_parameter("scene_1_go_file_path",   "/home/nvidia/waypoints/scenario_one_go.csv");
    this->declare_parameter("scene_1_back_file_path", "/home/nvidia/waypoints/scenario_one_back.csv");
    this->declare_parameter("scene_2_go_file_path",   "/home/nvidia/waypoints/scenario_two_go.csv");
    this->declare_parameter("scene_2_back_file_path", "/home/nvidia/waypoints/scenario_two_back.csv");

    scene_one_go_file_path_   = this->get_parameter("scene_1_go_file_path").as_string();
    scene_one_back_file_path_ = this->get_parameter("scene_1_back_file_path").as_string();
    scene_two_go_file_path_   = this->get_parameter("scene_2_go_file_path").as_string();
    scene_two_back_file_path_ = this->get_parameter("scene_2_back_file_path").as_string();

    // ===== 콜백 그룹 =====
    cbg_action_  = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cbg_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // ===== 퍼블리셔 (상태 알림) =====
    // 10=goal accepted, 11=rejected, 20=SUCCEEDED, 21=ABORTED, 22=CANCELED, 30=UNKNOWN
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepAll());
    waypoint_state_pub_ = this->create_publisher<std_msgs::msg::UInt8>("waypoint_follower_state", qos);

    // ===== 액션 클라이언트 =====
    waypoint_follower_client_ =
      rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints", cbg_action_);
    (void) waypoint_follower_client_->wait_for_action_server(std::chrono::seconds(3));

    // ===== 서비스 (시나리오 실행/일시정지/재개) =====
    waypoint_task_srv_ = this->create_service<Wego2dNav>(
      "run_waypoint_follower",
      std::bind(&WaypointRunner::setTask, this, _1, _2),
      rmw_qos_profile_services_default,
      cbg_service_);

    // ===== 액션 옵션 (콜백들) =====
    send_goal_options_.goal_response_callback =
        std::bind(&WaypointRunner::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options_.feedback_callback =
        std::bind(&WaypointRunner::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback =
        std::bind(&WaypointRunner::resultCallback, this, std::placeholders::_1);
  }

private:
  // ===== CSV 로더 =====
  // 형식: x,y,yaw(rad)  (yaw는 없으면 0)
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
    while (std::getline(ifs, line)) {
      if (line.empty()) continue;
      std::stringstream ss(line);
      std::string sx, sy, syaw;
      if (!std::getline(ss, sx, ',')) continue;
      if (!std::getline(ss, sy, ',')) continue;
      if (!std::getline(ss, syaw, ',')) syaw = "0.0";

      try {
        const double x = std::stod(sx);
        const double y = std::stod(sy);
        const double yaw = std::stod(syaw);

        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = rclcpp::Time(0); // Nav2가 내부에서 stamp를 다시 찍어도 무방
        p.header.frame_id = "map";
        p.pose.position.x = x;
        p.pose.position.y = y;
        p.pose.position.z = 0.0;

        // Yaw -> quaternion (Z축 회전)
        const double cy = std::cos(yaw * 0.5);
        const double syq = std::sin(yaw * 0.5);
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = syq;
        p.pose.orientation.w = cy;

        out.emplace_back(std::move(p));
      } catch (...) {
        continue;
      }
    }
    return true;
  }

  // 남은 웨이포인트만 잘라내기
  static std::vector<geometry_msgs::msg::PoseStamped>
  sliceFrom(const std::vector<geometry_msgs::msg::PoseStamped>& v, int start_idx_inclusive) {
    if (start_idx_inclusive < 0) start_idx_inclusive = 0;
    if (start_idx_inclusive >= static_cast<int>(v.size())) return {};
    return std::vector<geometry_msgs::msg::PoseStamped>(v.begin() + start_idx_inclusive, v.end());
  }

  // 공통 goal 전송 (핸들 저장 포함)
  void sendWaypointsGoal(std::vector<geometry_msgs::msg::PoseStamped>&& waypoints) {
    if (waypoints.empty()) {
      RCLCPP_WARN(get_logger(), "sendWaypointsGoal: empty waypoints");
      return;
    }
    FollowWaypoints::Goal goal;
    goal.poses = std::move(waypoints);

    // goal handle 저장 (비동기 결과)
    (void) waypoint_follower_client_->async_send_goal(goal, send_goal_options_);
  }

  // ===== 서비스 핸들러 =====
  void setTask(const std::shared_ptr<Wego2dNav::Request> req,
               std::shared_ptr<Wego2dNav::Response> res)
  {
    if (!waypoint_follower_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "FollowWaypoints action server not ready");
      res->respond = false;
      return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    std::string msg;
    bool ok = false;

    switch (req->scene) {
      case Wego2dNav::Request::SCENE_ONE_GO:
        ok = loadCSV(scene_one_go_file_path_, waypoints, msg);
        paused_waypoint_ = 0;
        if (ok) last_scene_.store(Wego2dNav::Request::SCENE_ONE_GO);
        break;
      case Wego2dNav::Request::SCENE_ONE_BACK:
        ok = loadCSV(scene_one_back_file_path_, waypoints, msg);
        if (ok) last_scene_.store(Wego2dNav::Request::SCENE_ONE_BACK);
        paused_waypoint_ = 0;
        break;
      case Wego2dNav::Request::SCENE_TWO_GO:
        ok = loadCSV(scene_two_go_file_path_, waypoints, msg);
        if (ok) last_scene_.store(Wego2dNav::Request::SCENE_TWO_GO);
        paused_waypoint_ = 0;
        break;
      case Wego2dNav::Request::SCENE_TWO_BACK:
        ok = loadCSV(scene_two_back_file_path_, waypoints, msg);
        if (ok) last_scene_.store(Wego2dNav::Request::SCENE_TWO_BACK);
        paused_waypoint_ = 0;
        break;

      case Wego2dNav::Request::PAUSE: {
        std::lock_guard<std::mutex> lk(goal_mtx_);
        if (current_goal_) {
          RCLCPP_INFO(get_logger(), "[PAUSE] cancel current FollowWaypoints goal");
          (void) waypoint_follower_client_->async_cancel_goal(current_goal_);
          res->respond = true;
          return;
        } else {
          RCLCPP_WARN(get_logger(), "[PAUSE] no active goal");
          res->respond = false;
          return;
        }
      }

      case Wego2dNav::Request::CONTINUE: {
        // 마지막 실행한 scene을 기준으로 CSV 다시 로드
        int scene = last_scene_.load();
        bool ok_resume = false;
        std::string msg2;
        std::vector<geometry_msgs::msg::PoseStamped> all;
        switch (scene) {
          case Wego2dNav::Request::SCENE_ONE_GO:
            ok_resume = loadCSV(scene_one_go_file_path_, all, msg2); break;
          case Wego2dNav::Request::SCENE_ONE_BACK:
            ok_resume = loadCSV(scene_one_back_file_path_, all, msg2); break;
          case Wego2dNav::Request::SCENE_TWO_GO:
            ok_resume = loadCSV(scene_two_go_file_path_, all, msg2); break;
          case Wego2dNav::Request::SCENE_TWO_BACK:
            ok_resume = loadCSV(scene_two_back_file_path_, all, msg2); break;
          default:
            RCLCPP_WARN(get_logger(), "[CONTINUE] unknown last scene; nothing to resume");
            res->respond = false;
            return;
        }
        if (!ok_resume || all.empty()) {
          RCLCPP_WARN(get_logger(), "[CONTINUE] reload failed: %s", msg2.c_str());
          res->respond = false;
          return;
        }

        // current_waypoint_는 feedback에서 업데이트됨
        // 보통 "다음 인덱스"부터 재개하고 싶으면 +1이 안전
        const int resume_idx = std::max(0, paused_waypoint_ + 1);
        auto remain = sliceFrom(all, resume_idx);
        if (remain.empty()) {
          RCLCPP_INFO(get_logger(), "[CONTINUE] nothing to resume (already at end)");
          res->respond = true;
          return;
        }
        RCLCPP_INFO(get_logger(), "[CONTINUE] resume from index %d (remaining %zu)", resume_idx, remain.size());
        {
          std::lock_guard<std::mutex> lk(goal_mtx_);
          current_goal_.reset();
        }
        sendWaypointsGoal(std::move(remain));
        res->respond = true;
        return;
      }

      default:
        RCLCPP_WARN(get_logger(), "Unknown scene id: %d", static_cast<int>(req->scene));
        res->respond = false;
        return;
    }

    if (!ok) {
      RCLCPP_WARN(get_logger(), "Failed to load CSV: %s", msg.c_str());
      res->respond = false;
      return;
    }
    if (waypoints.empty()) {
      RCLCPP_WARN(get_logger(), "No waypoints loaded");
      res->respond = false;
      return;
    }

    // 새 시나리오 시작(Go/Back 등)
    {
      std::lock_guard<std::mutex> lk(goal_mtx_);
      current_goal_.reset();  // 새 goal 시작 전 초기화(안전)
    }
    sendWaypointsGoal(std::move(waypoints));
    res->respond = true;
  }

  // ===== 액션 콜백들 =====
  void goalResponseCallback(GoalHandleWaypointFollow::SharedPtr goal_handle)
  {
    std_msgs::msg::UInt8 fed;
    if (!goal_handle) {
      RCLCPP_WARN(get_logger(), "Goal was rejected by server");
      fed.data = 11;  // rejected
    } else {
      RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result...");
      fed.data = 10;  // accepted
      {
        std::lock_guard<std::mutex> lk(goal_mtx_);
        current_goal_ = goal_handle;
      }
    }
    waypoint_state_pub_->publish(fed);
  }

  void feedbackCallback(
      GoalHandleWaypointFollow::SharedPtr /*unused*/,
      const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
  {
    // Nav2 버전에 따라 의미가 다를 수 있음(진행 중 or 마지막 완료 인덱스)
    current_waypoint_ = feedback->current_waypoint;
  }

  void resultCallback(const GoalHandleWaypointFollow::WrappedResult & result)
  {
    std_msgs::msg::UInt8 fed;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Waypoint following SUCCEEDED");
        fed.data = 20;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(get_logger(), "Waypoint following ABORTED");
        fed.data = 21;
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(get_logger(), "Waypoint following CANCELED");
        fed.data = 22;
        // 취소 시점의 인덱스를 기록(재개 시 +1부터 가는 게 일반적)
        paused_waypoint_ += current_waypoint_;
        {
          std::lock_guard<std::mutex> lk(goal_mtx_);
          current_goal_.reset();
        }
        break;
      default:
        RCLCPP_WARN(get_logger(), "Unknown result code");
        fed.data = 30;
        break;
    }
    waypoint_state_pub_->publish(fed);
  }

private:
  // ===== 파라미터(파일 경로) =====
  std::string scene_one_go_file_path_;
  std::string scene_one_back_file_path_;
  std::string scene_two_go_file_path_;
  std::string scene_two_back_file_path_;

  // ===== 상태 =====
  int current_waypoint_{-1};
  int paused_waypoint_{-1};
  std::atomic<int> last_scene_{-1};

  // ===== 동시성 제어 =====
  std::mutex goal_mtx_;
  GoalHandleWaypointFollow::SharedPtr current_goal_;

  // ===== 콜백 그룹 =====
  rclcpp::CallbackGroup::SharedPtr cbg_action_;
  rclcpp::CallbackGroup::SharedPtr cbg_service_;

  // ===== ROS 엔티티 =====
  rclcpp::Service<Wego2dNav>::SharedPtr waypoint_task_srv_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr waypoint_follower_client_;
  rclcpp_action::Client<FollowWaypoints>::SendGoalOptions send_goal_options_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr waypoint_state_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointRunner>();

  // 멀티스레드 실행자: 액션/서비스 병행 처리
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
