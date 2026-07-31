// =============================================================
// 描述：
// UI状态管理节点，发布UI专用状态消息
// 订阅RSI心跳、打印头状态、KUKA状态、轨迹与事件
// TODO: 发布系统级 stop/continue/restart等服务请求
// =============================================================

#include <rclcpp/rclcpp.hpp>

#include <my_project_interfaces/msg/ui_status.hpp>
#include <my_project_interfaces/msg/trajectory_point.hpp>
#include <my_project_interfaces/msg/planned_event.hpp>
#include <my_project_interfaces/msg/print_head_status.hpp>
#include <my_project_interfaces/msg/print_timing_plan.hpp>
#include <my_project_interfaces/msg/rsi_heart_beat.hpp>
#include <my_project_interfaces/msg/kuka_status.hpp>

#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <string>

using my_project_interfaces::msg::UiStatus;
using my_project_interfaces::msg::TrajectoryPoint;
using my_project_interfaces::msg::PlannedEvent;
using my_project_interfaces::msg::PrintHeadStatus;
using my_project_interfaces::msg::PrintTimingPlan;
using my_project_interfaces::msg::RsiHeartBeat;
using my_project_interfaces::msg::KukaStatus;

class SystemManagerNode : public rclcpp::Node
{
public:
  SystemManagerNode()
  : Node("system_manager_node")
  {
    publish_period_ms_ = declare_parameter<int>("ui_publish_period_ms", 100);
    heartbeat_timeout_s_ = declare_parameter<double>("heartbeat_timeout_s", 1.0);
    traj_queue_limit_ = declare_parameter<int>("traj_queue_limit", 5000);
    event_queue_limit_ = declare_parameter<int>("event_queue_limit", 2000);
    print_time_update_period_ms_ = declare_parameter<int>("print_time_update_period_ms", 500);
    tool_change_fixed_time_s_ = std::max(
      0.0, declare_parameter<double>("tool_change_fixed_time_s", 15.0));

    ui_pub_ = create_publisher<UiStatus>("/ui/status", 10);

    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "/system/command",
      rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (msg->data == "REQUEST_PAUSE") {
          system_command_ = "PAUSE_REQUESTED";
        } else if (msg->data == "PAUSE_READY") {
          system_command_ = "PAUSE_READY";
        } else if (msg->data == "PAUSE") {
          system_command_ = "PAUSED";
        } else if (msg->data == "RESUME") {
          system_command_ = "";           // 清除，恢复正常状态判定
        } else if (msg->data == "ABORT") {
          system_command_ = "ABORTING";
        }
      });

    kuka_sub_ = create_subscription<KukaStatus>(
      "/kuka/status", 10,
      [this](KukaStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_kuka_status_ = *msg;
        last_kuka_stamp_ = now();
      });

    hb_sub_ = create_subscription<RsiHeartBeat>(
      "/rsi/heartbeat", 10,
      [this](RsiHeartBeat::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_hb_ = *msg;
        last_hb_stamp_ = now();
      });

    auto ready_qos = rclcpp::QoS(200).reliable().transient_local();
    printhead_sub_ = create_subscription<PrintHeadStatus>(
      "/printhead/status", ready_qos,
      [this](PrintHeadStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_printhead_status_ = *msg;
        last_printhead_stamp_ = now();
      });

    planned_event_sub_ = create_subscription<PlannedEvent>(
      "/planned_events", 10,
      [this](PlannedEvent::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        event_queue_.push_back(*msg);
        trim_queue(event_queue_, static_cast<size_t>(event_queue_limit_));
      });

    triggered_event_sub_ = create_subscription<PlannedEvent>(
      "/rsi/triggered_event", 10,
      [this](PlannedEvent::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_triggered_event_ = *msg;
        last_triggered_stamp_ = now();
        if (msg->event_type == "tool_change_cf" ||
        msg->event_type == "tool_change_resin")
        {
          const std::string event_key =
          std::to_string(msg->trigger_seq) + ":" + msg->event_type;
          if (event_key != last_counted_tool_change_key_) {
            ++triggered_tool_change_count_;
            last_tool_change_start_ = last_triggered_stamp_;
            last_counted_tool_change_key_ = event_key;
          }
        }
        auto same_event = [&msg](const PlannedEvent & ev) {
          return ev.trigger_seq == msg->trigger_seq &&
          ev.event_type == msg->event_type &&
          ev.payload == msg->payload;
        };
        // 先丢弃比当前触发序号小的事件
        while (!event_queue_.empty() &&
        event_queue_.front().trigger_seq < msg->trigger_seq)
        {
          event_queue_.pop_front();
        }
        // 再移除队头与当前触发事件完全一致的一条，避免 current/next 重复
        if (!event_queue_.empty() && same_event(event_queue_.front())) {
          event_queue_.pop_front();
        }
      });

    planned_traj_sub_ = create_subscription<TrajectoryPoint>(
      "/planned_trajectory", 10,
      [this](TrajectoryPoint::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        traj_queue_.push_back(*msg);
        trim_queue(traj_queue_, static_cast<size_t>(traj_queue_limit_));
      });

    timing_plan_sub_ = create_subscription<PrintTimingPlan>(
      "/print/timing_plan", rclcpp::QoS(1).reliable().transient_local(),
      [this](PrintTimingPlan::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        timing_plan_ = *msg;
        print_time_cache_valid_ = false;
        triggered_tool_change_count_ = 0;
        last_tool_change_start_ = rclcpp::Time();
        last_counted_tool_change_key_.clear();
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms_),
      [this]() {publish_ui_status();});

    RCLCPP_INFO(get_logger(), "system_manager_node 已启动");
  }

private:
  template<typename T>
  void trim_queue(std::deque<T> & q, size_t limit)
  {
    while (q.size() > limit) {
      q.pop_front();
    }
  }

  void publish_ui_status()
  {
    UiStatus out;
    auto now_t = now();
    out.stamp = now_t;

    std::lock_guard<std::mutex> lk(mutex_);

    // 系统状态：优先使用命令状态
    if (!system_command_.empty()) {
      out.state = system_command_;
      // ABORTING 状态下，如果心跳丢失，切换为 ABORTED
      if (system_command_ == "ABORTING" && last_hb_) {
        double hb_age = (now_t - last_hb_stamp_).seconds();
        if (hb_age > heartbeat_timeout_s_) {
          system_command_ = "ABORTED";
          out.state = "ABORTED";
        }
      }
    } else if (!last_hb_) {
      out.state = "WAIT_HEARTBEAT";
    } else {
      double hb_age = (now_t - last_hb_stamp_).seconds();
      if (hb_age <= heartbeat_timeout_s_) {
        out.state = "RUNNING";
      } else {
        out.state = "HEARTBEAT_LOST";
      }
    }

    // KUKA状态
    if (last_kuka_status_) {
      out.kuka_status = *last_kuka_status_;
      out.kuka_status_valid = true;
      out.kuka_status_age_s = (now_t - last_kuka_stamp_).seconds();
    } else {
      out.kuka_status_valid = false;
      out.kuka_status_age_s = 0.0f;
    }

    // RSI心跳
    if (last_hb_) {
      out.rsi_heartbeat = *last_hb_;
      out.rsi_heartbeat_valid = true;
      out.rsi_heartbeat_age_s = (now_t - last_hb_stamp_).seconds();
    } else {
      out.rsi_heartbeat_valid = false;
      out.rsi_heartbeat_age_s = 0.0f;
    }

    // 打印头状态
    if (last_printhead_status_) {
      out.printhead_status = *last_printhead_status_;
      out.printhead_status_valid = true;
      out.printhead_status_age_s = (now_t - last_printhead_stamp_).seconds();
      out.ready_for_motion = last_printhead_status_->ready_for_motion;
    } else {
      out.printhead_status_valid = false;
      out.printhead_status_age_s = 0.0f;
      out.ready_for_motion = false;
    }

    // 轨迹：基于心跳seq对齐
    bool has_seq = last_hb_.has_value();
    uint32_t seq_used = has_seq ? last_hb_->seq_used : 0;
    while (has_seq && !traj_queue_.empty() && traj_queue_.front().seq < seq_used) {
      traj_queue_.pop_front();
    }
    if (has_seq && !traj_queue_.empty() && traj_queue_.front().seq == seq_used) {
      out.current_traj = traj_queue_.front();
      out.current_traj_valid = true;
      if (traj_queue_.size() >= 2) {
        out.next_traj = traj_queue_[1];
        out.next_traj_valid = true;
        out.traj_next_seq = out.next_traj.seq;
      } else {
        out.next_traj_valid = false;
        out.traj_next_seq = 0;
      }
    } else {
      out.current_traj_valid = false;
      if (!traj_queue_.empty()) {
        out.next_traj = traj_queue_.front();
        out.next_traj_valid = true;
        out.traj_next_seq = out.next_traj.seq;
      } else {
        out.next_traj_valid = false;
        out.traj_next_seq = 0;
      }
    }
    out.traj_backlog = static_cast<uint32_t>(traj_queue_.size());

    // 事件：触发事件为当前，队列头为下一条
    if (last_triggered_event_) {
      out.current_event = *last_triggered_event_;
      out.current_event_valid = true;
    } else {
      out.current_event_valid = false;
    }

    if (!event_queue_.empty()) {
      out.next_event = event_queue_.front();
      out.next_event_valid = true;
      out.event_next_seq = out.next_event.trigger_seq;
    } else {
      out.next_event_valid = false;
      out.event_next_seq = 0;
    }
    out.event_pending = static_cast<uint32_t>(event_queue_.size());

    update_print_time_status(out, now_t);

    // 警告/错误占位（后续可接入）
    out.last_warn = "";
    out.last_error = "";

    ui_pub_->publish(out);
  }

  void update_print_time_status(UiStatus & out, const rclcpp::Time & now_t)
  {
    out.planned_total_time_s = 0.0F;
    out.planned_elapsed_time_s = 0.0F;
    out.planned_remaining_time_s = 0.0F;
    out.planned_trajectory_time_s = 0.0F;
    out.planned_print_motion_time_s = 0.0F;
    out.planned_travel_time_s = 0.0F;
    out.planned_wait_time_s = 0.0F;
    out.planned_cut_time_s = 0.0F;
    out.cut_injected_wait_s = 0.0F;
    out.planned_cut_injected_wait_time_s = 0.0F;
    out.planned_tool_change_time_s = 0.0F;
    out.planned_tool_change_elapsed_time_s = 0.0F;
    out.tool_change_fixed_time_s = static_cast<float>(tool_change_fixed_time_s_);
    out.planned_cut_count = 0;
    out.planned_tool_change_count = 0;
    out.planned_unquantified_event_count = 0;
    out.print_time_breakdown_valid = false;
    out.print_time_valid = false;

    if (out.current_traj_valid && out.current_traj.planned_time_valid &&
      timing_plan_ && timing_plan_->valid)
    {
      const float trajectory_total =
        std::max(0.0F, timing_plan_->planned_total_time_s);
      const float tool_change_total =
        static_cast<float>(timing_plan_->planned_tool_change_count) *
        static_cast<float>(tool_change_fixed_time_s_);
      const float total = trajectory_total + tool_change_total;
      const bool new_run = print_time_cache_valid_ &&
        (out.current_traj.seq<print_time_last_seq_ ||
        std::abs(total - print_time_total_s_)>1e-3F ||
        timing_plan_->planned_tool_change_count !=
        print_time_tool_change_count_);
      if (new_run) {
        print_time_cache_valid_ = false;
        triggered_tool_change_count_ = 0;
        last_tool_change_start_ = rclcpp::Time();
        last_counted_tool_change_key_.clear();
      }

      const double elapsed_since_update = print_time_last_update_.nanoseconds() == 0 ?
        std::numeric_limits<double>::infinity() :
        (now_t - print_time_last_update_).seconds();
      const bool update_due = !print_time_cache_valid_ ||
        elapsed_since_update * 1000.0 >= static_cast<double>(print_time_update_period_ms_);
      if (update_due) {
        const uint32_t planned_tool_changes =
          timing_plan_->planned_tool_change_count;
        float tool_change_elapsed = 0.0F;
        if (triggered_tool_change_count_ > 0) {
          const uint32_t completed_before_current = triggered_tool_change_count_ - 1;
          double current_progress = tool_change_fixed_time_s_;
          if (last_tool_change_start_.nanoseconds() != 0) {
            current_progress = std::clamp(
              (now_t - last_tool_change_start_).seconds(),
              0.0, tool_change_fixed_time_s_);
          }
          tool_change_elapsed =
            static_cast<float>(completed_before_current) *
            static_cast<float>(tool_change_fixed_time_s_) +
            static_cast<float>(current_progress);
        }
        tool_change_elapsed = std::clamp(
          tool_change_elapsed, 0.0F, tool_change_total);
        const float trajectory_elapsed = std::clamp(
          out.current_traj.planned_time_s, 0.0F, trajectory_total);
        const float elapsed = std::clamp(
          trajectory_elapsed + tool_change_elapsed, 0.0F, total);
        print_time_total_s_ = total;
        print_time_elapsed_s_ = elapsed;
        print_time_remaining_s_ = std::clamp(total - elapsed, 0.0F, total);
        print_time_trajectory_total_s_ = trajectory_total;
        print_time_print_motion_s_ =
          timing_plan_->planned_print_motion_time_s;
        print_time_travel_s_ = timing_plan_->planned_travel_time_s;
        print_time_wait_s_ = timing_plan_->planned_wait_time_s;
        print_time_cut_s_ = timing_plan_->planned_cut_time_s;
        print_time_cut_injected_wait_s_ =
          timing_plan_->cut_injected_wait_s;
        print_time_cut_injected_wait_total_s_ =
          timing_plan_->planned_cut_injected_wait_time_s;
        print_time_tool_change_s_ = tool_change_total;
        print_time_tool_change_elapsed_s_ = tool_change_elapsed;
        print_time_cut_count_ = timing_plan_->planned_cut_count;
        print_time_tool_change_count_ = planned_tool_changes;
        print_time_unquantified_event_count_ =
          timing_plan_->planned_unquantified_event_count;
        print_time_breakdown_valid_ =
          timing_plan_->breakdown_valid;
        print_time_last_seq_ = out.current_traj.seq;
        print_time_last_update_ = now_t;
        print_time_cache_valid_ = true;
      }
    }

    if (print_time_cache_valid_) {
      out.planned_total_time_s = print_time_total_s_;
      out.planned_elapsed_time_s = print_time_elapsed_s_;
      out.planned_remaining_time_s = print_time_remaining_s_;
      out.planned_trajectory_time_s = print_time_trajectory_total_s_;
      out.planned_print_motion_time_s = print_time_print_motion_s_;
      out.planned_travel_time_s = print_time_travel_s_;
      out.planned_wait_time_s = print_time_wait_s_;
      out.planned_cut_time_s = print_time_cut_s_;
      out.cut_injected_wait_s = print_time_cut_injected_wait_s_;
      out.planned_cut_injected_wait_time_s =
        print_time_cut_injected_wait_total_s_;
      out.planned_tool_change_time_s = print_time_tool_change_s_;
      out.planned_tool_change_elapsed_time_s =
        print_time_tool_change_elapsed_s_;
      out.planned_cut_count = print_time_cut_count_;
      out.planned_tool_change_count = print_time_tool_change_count_;
      out.planned_unquantified_event_count =
        print_time_unquantified_event_count_;
      out.print_time_breakdown_valid = print_time_breakdown_valid_;
      out.print_time_valid = true;
    }
  }

private:
  rclcpp::Publisher<UiStatus>::SharedPtr ui_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;

  rclcpp::Subscription<KukaStatus>::SharedPtr kuka_sub_;
  rclcpp::Subscription<RsiHeartBeat>::SharedPtr hb_sub_;
  rclcpp::Subscription<PrintHeadStatus>::SharedPtr printhead_sub_;
  rclcpp::Subscription<PlannedEvent>::SharedPtr planned_event_sub_;
  rclcpp::Subscription<PlannedEvent>::SharedPtr triggered_event_sub_;
  rclcpp::Subscription<TrajectoryPoint>::SharedPtr planned_traj_sub_;
  rclcpp::Subscription<PrintTimingPlan>::SharedPtr timing_plan_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  std::optional<KukaStatus> last_kuka_status_;
  std::optional<RsiHeartBeat> last_hb_;
  std::optional<PrintHeadStatus> last_printhead_status_;
  std::optional<PrintTimingPlan> timing_plan_;
  std::optional<PlannedEvent> last_triggered_event_;
  rclcpp::Time last_kuka_stamp_;
  rclcpp::Time last_hb_stamp_;
  rclcpp::Time last_printhead_stamp_;
  rclcpp::Time last_triggered_stamp_;

  std::deque<TrajectoryPoint> traj_queue_;
  std::deque<PlannedEvent> event_queue_;

  int publish_period_ms_{200};
  double heartbeat_timeout_s_{1.0};
  int traj_queue_limit_{5000};
  int event_queue_limit_{2000};
  int print_time_update_period_ms_{500};
  double tool_change_fixed_time_s_{15.0};
  bool print_time_cache_valid_{false};
  uint32_t print_time_last_seq_{0};
  float print_time_total_s_{0.0F};
  float print_time_elapsed_s_{0.0F};
  float print_time_remaining_s_{0.0F};
  float print_time_trajectory_total_s_{0.0F};
  float print_time_print_motion_s_{0.0F};
  float print_time_travel_s_{0.0F};
  float print_time_wait_s_{0.0F};
  float print_time_cut_s_{0.0F};
  float print_time_cut_injected_wait_s_{0.0F};
  float print_time_cut_injected_wait_total_s_{0.0F};
  float print_time_tool_change_s_{0.0F};
  float print_time_tool_change_elapsed_s_{0.0F};
  uint32_t print_time_cut_count_{0};
  uint32_t print_time_tool_change_count_{0};
  uint32_t print_time_unquantified_event_count_{0};
  bool print_time_breakdown_valid_{false};
  uint32_t triggered_tool_change_count_{0};
  rclcpp::Time last_tool_change_start_;
  std::string last_counted_tool_change_key_;
  rclcpp::Time print_time_last_update_;
  std::string system_command_;   // 当前系统命令状态: "PAUSED" / "ABORTING" / "ABORTED" / ""
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemManagerNode>());
  rclcpp::shutdown();
  return 0;
}
