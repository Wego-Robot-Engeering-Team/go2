// src/waypoint_runner.cpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <wego_2d_nav/srv/waypoint_task.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/empty.hpp>  // (optional) costmap clear

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <thread>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class WaypointRunner : public rclcpp::Node {
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleWaypointFollow = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
  using Wego2dNav = wego_2d_nav::srv::WaypointTask;

  WaypointRunner()
  : Node("waypoint_runner")
  {
    // ===== CSV 경로 파라미터 =====
    this->declare_parameter("waypoint_1_file_path", "/home/wego/waypoints/waypoint_one.csv");
    this->declare_parameter("waypoint_2_file_path", "/home/wego/waypoints/waypoint_two.csv");
    this->declare_parameter("waypoint_3_file_path", "/home/wego/waypoints/waypoint_three.csv");
    this->declare_parameter("waypoint_4_file_path", "/home/wego/waypoints/waypoint_four.csv");
    this->declare_parameter("waypoint_5_file_path", "/home/wego/waypoints/waypoint_five.csv");

    waypoint_one_file_path_   = this->get_parameter("waypoint_1_file_path").as_string();
    waypoint_two_file_path_ = this->get_parameter("waypoint_2_file_path").as_string();
    waypoint_three_file_path_ = this->get_parameter("waypoint_3_file_path").as_string();
    waypoint_four_file_path_ = this->get_parameter("waypoint_4_file_path").as_string();
    waypoint_five_file_path_ = this->get_parameter("waypoint_5_file_path").as_string();

    // ===== 콜백 그룹 =====
    cbg_action_  = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cbg_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // ===== 상태 퍼블리셔 =====
    // 10=accepted, 11=rejected, 20=SUCCEEDED, 21=ABORTED, 22=CANCELED, 30=UNKNOWN
    waypoint_state_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      "waypoint_follower_state", rclcpp::QoS(rclcpp::KeepAll()));

    // ===== 액션 클라이언트 =====
    waypoint_follower_client_ =
      rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints", cbg_action_);
    (void) waypoint_follower_client_->wait_for_action_server(3s);

    // ===== 서비스 서버 =====
    waypoint_task_srv_ = this->create_service<Wego2dNav>(
      "run_waypoint_follower",
      std::bind(&WaypointRunner::setTask, this, _1, _2),
      rmw_qos_profile_services_default,
      cbg_service_);

    // ===== 액션 콜백 옵션 =====
    send_goal_options_.goal_response_callback =
        std::bind(&WaypointRunner::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options_.feedback_callback =
        std::bind(&WaypointRunner::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback =
        std::bind(&WaypointRunner::resultCallback, this, std::placeholders::_1);
  }

private:
  // ===== CSV 로더: x,y,yaw(rad) (yaw 없으면 0) =====
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
        p.header.stamp = rclcpp::Time(0);  // Nav2 내부에서 재타임스탬프 가능
        p.header.frame_id = "map";
        p.pose.position.x = x;
        p.pose.position.y = y;
        p.pose.position.z = 0.0;
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

  static std::vector<geometry_msgs::msg::PoseStamped>
  sliceFrom(const std::vector<geometry_msgs::msg::PoseStamped>& v, int start_idx_inclusive) {
    if (start_idx_inclusive < 0) start_idx_inclusive = 0;
    if (start_idx_inclusive >= static_cast<int>(v.size())) return {};
    return std::vector<geometry_msgs::msg::PoseStamped>(v.begin() + start_idx_inclusive, v.end());
  }

  // ===== (선택) costmap clear: fire-and-forget =====
  void tryClearCostmaps() {
    auto call_empty = [&](const std::string& name) {
      auto cli = this->create_client<std_srvs::srv::Empty>(name);
      if (!cli->wait_for_service(200ms)) return;
      auto req = std::make_shared<std_srvs::srv::Empty::Request>();
      (void)cli->async_send_request(req);
    };
    call_empty("/global_costmap/clear_entirely_global_costmap");
    call_empty("/local_costmap/clear_entirely_local_costmap");
  }

  // ===== goal 전송(전역/로컬 인덱스 리셋 포함) =====
  void sendWaypointsGoal(std::vector<geometry_msgs::msg::PoseStamped>&& waypoints,
                         int base_idx)
  {
    if (waypoints.empty()) {
      RCLCPP_WARN(get_logger(), "sendWaypointsGoal: empty waypoints");
      return;
    }
    base_start_idx_ = base_idx;
    local_wp_idx_   = -1;

    FollowWaypoints::Goal goal;
    goal.poses = std::move(waypoints);

    // 성공 시 마지막 완료 인덱스 계산용( base + goal_size - 1 )
    last_goal_size_ = static_cast<int>(goal.poses.size());

    (void) waypoint_follower_client_->async_send_goal(goal, send_goal_options_);
  }

  // ===== 디바운스(중복 호출 방지) =====
  bool debounce(int kind, int ms = 200) {
    auto now = std::chrono::steady_clock::now();
    if (kind == last_req_kind_ &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_req_tp_).count() < ms) {
      return true; // drop
    }
    last_req_kind_ = kind;
    last_req_tp_ = now;
    return false;
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

    // kind: 0=SCENE(새 실행), 1=PAUSE, 2=CONTINUE
    const int kind = (req->scene == Wego2dNav::Request::PAUSE) ? 1 :
                     (req->scene == Wego2dNav::Request::CONTINUE) ? 2 : 0;
    if (debounce(kind)) {
      RCLCPP_WARN(get_logger(), "[REQ] debounced");
      res->respond = false;
      return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    std::string msg;
    bool ok = false;

    switch (req->scene) {
      // === 새 시나리오 시작 ===
      case Wego2dNav::Request::ONE_GO:
        ok = loadCSV(waypoint_one_file_path_, waypoints, msg);
        if (ok) last_scene_.store(Wego2dNav::Request::ONE_GO);
        paused_waypoint_ = -1;
        break;
      case Wego2dNav::Request::TWO_GO:
        ok = loadCSV(waypoint_two_file_path_, waypoints, msg);
        if (ok) last_scene_.store(Wego2dNav::Request::TWO_GO);
        paused_waypoint_ = -1;
        break;
      case Wego2dNav::Request::THREE_GO:
        ok = loadCSV(waypoint_three_file_path_, waypoints, msg);
        if (ok) last_scene_.store(Wego2dNav::Request::THREE_GO);
        paused_waypoint_ = -1;
        break;
      case Wego2dNav::Request::FOUR_GO:
        ok = loadCSV(waypoint_four_file_path_, waypoints, msg);
        if (ok) last_scene_.store(Wego2dNav::Request::FOUR_GO);
        paused_waypoint_ = -1;
        break;
      case Wego2dNav::Request::FIVE_GO:
        ok = loadCSV(waypoint_five_file_path_, waypoints, msg);
        if (ok) last_scene_.store(Wego2dNav::Request::FIVE_GO);
        paused_waypoint_ = -1;
        break;
      // === 일시정지 ===
      case Wego2dNav::Request::PAUSE: {
        auto st = state_.load();
        if (st != RunState::Running) {
          RCLCPP_WARN(get_logger(), "[PAUSE] ignored: state=%d", (int)st);
          res->respond = false;
          return;
        }
        std::lock_guard<std::mutex> lk(goal_mtx_);
        if (current_goal_) {
          RCLCPP_INFO(get_logger(), "[PAUSE] cancel current FollowWaypoints goal");
          cancel_in_flight_.store(true);
          cancel_done_.store(false);
          state_.store(RunState::Pausing);
          (void) waypoint_follower_client_->async_cancel_goal(current_goal_);
          res->respond = true;
        } else {
          RCLCPP_WARN(get_logger(), "[PAUSE] no active goal");
          res->respond = false;
        }
        return;
      }

      // === 재개 ===
      case Wego2dNav::Request::CONTINUE: {
        // 취소 완료 대기(최대 1초)
        for (int i = 0; i < 10 && cancel_in_flight_.load() && !cancel_done_.load(); ++i) {
          RCLCPP_WARN(get_logger(), "[CONTINUE] waiting cancel to finish...");
          std::this_thread::sleep_for(100ms);
        }
        auto st = state_.load();
        if (st != RunState::Paused) {
          RCLCPP_WARN(get_logger(), "[CONTINUE] ignored: state=%d (need Paused)", (int)st);
          res->respond = false;
          return;
        }

        // 마지막 scene 로드
        int scene = last_scene_.load();
        bool ok_resume = false;
        std::string msg2;
        std::vector<geometry_msgs::msg::PoseStamped> all;
        switch (scene) {
          case Wego2dNav::Request::ONE_GO:
            ok_resume = loadCSV(waypoint_one_file_path_, all, msg2); break;
          case Wego2dNav::Request::TWO_GO:
            ok_resume = loadCSV(waypoint_two_file_path_, all, msg2); break;
          case Wego2dNav::Request::THREE_GO:
            ok_resume = loadCSV(waypoint_three_file_path_, all, msg2); break;
          case Wego2dNav::Request::FOUR_GO:
            ok_resume = loadCSV(waypoint_four_file_path_, all, msg2); break;
          case Wego2dNav::Request::FIVE_GO:
            ok_resume = loadCSV(waypoint_five_file_path_, all, msg2); break;
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

        tryClearCostmaps();
        sendWaypointsGoal(std::move(remain), /*base_idx=*/resume_idx);
        // state_는 goalResponse에서 Running으로 변경
        res->respond = true;
        return;
      }

      default:
        RCLCPP_WARN(get_logger(), "Unknown scene id: %d", static_cast<int>(req->scene));
        res->respond = false;
        return;
    }

    // === 새 시나리오 시작 처리 ===
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

    {
      std::lock_guard<std::mutex> lk(goal_mtx_);
      current_goal_.reset();
    }
    tryClearCostmaps();
    sendWaypointsGoal(std::move(waypoints), /*base_idx=*/0);
    // state_는 goalResponse에서 Running으로 변경
    res->respond = true;
  }

  // ===== 액션 콜백 =====
  void goalResponseCallback(GoalHandleWaypointFollow::SharedPtr goal_handle)
  {
    std_msgs::msg::UInt8 fed;
    if (!goal_handle) {
      RCLCPP_WARN(get_logger(), "Goal was rejected by server");
      fed.data = 11;
    } else {
      RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result...");
      fed.data = 10;
      {
        std::lock_guard<std::mutex> lk(goal_mtx_);
        current_goal_ = goal_handle;
      }
      state_.store(RunState::Running);
    }
    waypoint_state_pub_->publish(fed);
  }

  void feedbackCallback(GoalHandleWaypointFollow::SharedPtr /*unused*/,
                        const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
  {
    // 현재 슬라이스(부분목록) 기준 로컬 인덱스
    local_wp_idx_    = feedback->current_waypoint;  // 0..(N-1) (타겟 인덱스)
    current_waypoint_ = local_wp_idx_;             // (이전 필드 유지)
  }

  void resultCallback(const GoalHandleWaypointFollow::WrappedResult & result)
  {
    std_msgs::msg::UInt8 fed;

    // === 마지막 완료 인덱스 계산 ===
    int last_completed_global = -1;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      // 전체 성공: 마지막 완료 = base + (goal_size - 1)
      last_completed_global = base_start_idx_ + std::max(last_goal_size_ - 1, 0);
    } else {
      // 취소/중단: 피드백의 current_waypoint는 "현재 타겟"
      // 완료 기준은 (local - 1). 피드백이 없으면 local=-1 → base-1 (아무 것도 완료 X)
      last_completed_global = base_start_idx_ + std::max(local_wp_idx_ - 1, -1);
    }

    if (result.result && !result.result->missed_waypoints.empty()) {
      std::ostringstream oss;
      for (auto i : result.result->missed_waypoints) oss << i << " ";
      RCLCPP_WARN(get_logger(), "Missed waypoints: %s", oss.str().c_str());
    }

    // === 상태 업데이트 ===
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Waypoint following SUCCEEDED");
        fed.data = 20;
        paused_waypoint_ = std::max(paused_waypoint_, last_completed_global);
        cancel_in_flight_.store(false);
        cancel_done_.store(true);
        state_.store(RunState::Idle);
        break;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(get_logger(), "Waypoint following ABORTED");
        fed.data = 21;
        paused_waypoint_ = std::max(paused_waypoint_, last_completed_global);
        {
          std::lock_guard<std::mutex> lk(goal_mtx_);
          current_goal_.reset();
        }
        cancel_in_flight_.store(false);
        cancel_done_.store(true);
        state_.store(RunState::Paused);
        break;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(get_logger(), "Waypoint following CANCELED");
        fed.data = 22;
        paused_waypoint_ = std::max(paused_waypoint_, last_completed_global);
        {
          std::lock_guard<std::mutex> lk(goal_mtx_);
          current_goal_.reset();
        }
        cancel_in_flight_.store(false);
        cancel_done_.store(true);
        state_.store(RunState::Paused);
        break;

      default:
        RCLCPP_WARN(get_logger(), "Unknown result code");
        fed.data = 30;
        break;
    }
    waypoint_state_pub_->publish(fed);
  }

private:
  // ===== 상태 enum =====
  enum class RunState { Idle = 0, Running = 1, Pausing = 2, Paused = 3 };
  std::atomic<RunState> state_{RunState::Idle};

  // ===== 파일 경로 =====
  std::string waypoint_one_file_path_;
  std::string waypoint_two_file_path_;
  std::string waypoint_three_file_path_;
  std::string waypoint_four_file_path_;
  std::string waypoint_five_file_path_;

  // ===== 인덱스/상태 =====
  int current_waypoint_{-1};   // (로컬) 현재 슬라이스 내 인덱스(디버그용)
  int paused_waypoint_{-1};    // (전역) 마지막 완료 인덱스
  std::atomic<int> last_scene_{-1};

  int base_start_idx_{0};      // 이번 슬라이스의 전역 시작 인덱스
  int local_wp_idx_{-1};       // 이번 슬라이스 내 로컬 타겟 인덱스
  int last_goal_size_{0};      // 이번 슬라이스의 포즈 개수 (SUCCEEDED 시 계산용)

  std::atomic<bool> cancel_in_flight_{false};
  std::atomic<bool> cancel_done_{true};

  // 디바운스
  std::chrono::steady_clock::time_point last_req_tp_{};
  int last_req_kind_{-1}; // 0:SCENE, 1:PAUSE, 2:CONTINUE

  // 동시성
  std::mutex goal_mtx_;
  GoalHandleWaypointFollow::SharedPtr current_goal_;

  // 콜백 그룹
  rclcpp::CallbackGroup::SharedPtr cbg_action_;
  rclcpp::CallbackGroup::SharedPtr cbg_service_;

  // ROS 엔티티
  rclcpp::Service<Wego2dNav>::SharedPtr waypoint_task_srv_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr waypoint_follower_client_;
  rclcpp_action::Client<FollowWaypoints>::SendGoalOptions send_goal_options_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr waypoint_state_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointRunner>();

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
