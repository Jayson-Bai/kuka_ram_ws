// =============================================================
// runtime_logger_node: ROS 2 node that owns a Collector session and
// publishes 1 Hz RuntimeLogHealth.
// =============================================================

#include "runtime_logger/collector.hpp"

#include <rclcpp/rclcpp.hpp>
#include <my_project_interfaces/msg/runtime_log_health.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <random>
#include <sstream>
#include <string>

using my_project_interfaces::msg::RuntimeLogHealth;

namespace
{

std::string make_run_id()
{
  std::random_device rd;
  std::ostringstream os;
  os << std::hex;
  for (int i = 0; i < 4; ++i) {
    os.width(8);
    os.fill('0');
    os << rd();
  }
  return os.str();
}

}  // namespace

class RuntimeLoggerNode : public rclcpp::Node
{
public:
  RuntimeLoggerNode()
  : Node("runtime_logger_node")
  {
    enabled_ = declare_parameter<bool>("runtime_log_enabled", true);
    root_ = declare_parameter<std::string>(
      "runtime_log_root", "/home/jayson/kuka_ram_ws/data/runtime_logs");
    socket_path_ = declare_parameter<std::string>("runtime_log_socket", "");
    buffer_bytes_ = static_cast<size_t>(declare_parameter<int>(
      "runtime_log_buffer_bytes", 8 * 1024 * 1024));
    batch_bytes_ = static_cast<uint64_t>(declare_parameter<int>(
      "runtime_log_batch_bytes", 64 * 1024));
    flush_ms_ = declare_parameter<int>("runtime_log_flush_period_ms", 100);
    sync_ms_ = declare_parameter<int>("runtime_log_sync_period_ms", 1000);
    rotate_bytes_ = static_cast<uint64_t>(declare_parameter<int>(
      "runtime_log_rotate_bytes", 64 * 1024 * 1024));
    shutdown_timeout_ms_ = declare_parameter<int>(
      "runtime_log_shutdown_timeout_ms", 5000);
    run_id_ = declare_parameter<std::string>("run_id", "");
    test_run_ = declare_parameter<bool>("runtime_log_test_run", false);
    if (run_id_.empty()) {
      run_id_ = make_run_id();
    }
    if (socket_path_.empty()) {
      socket_path_ = "/tmp/kuka_runtime_log_" + run_id_ + ".sock";
    }

    health_pub_ = create_publisher<RuntimeLogHealth>("/runtime/log_health", 10);

    if (enabled_) {
      start_session();
    } else {
      RCLCPP_WARN(
        get_logger(), "runtime logging disabled by parameter; no log session");
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000),
      [this]() {publish_health();});
  }

  ~RuntimeLoggerNode() override
  {
    stop_session();
  }

private:
  void start_session()
  {
    if (!manifest_.start(
        root_, run_id_,
        "kuka_ram_ws@runtime_logger_v1",
        "{}", test_run_))
    {
      RCLCPP_ERROR(
        get_logger(), "无法创建运行日志会话目录 %s", manifest_.run_dir().c_str());
      session_ok_ = false;
      return;
    }
    if (!collector_.start(
        socket_path_, manifest_.run_dir(), run_id_, buffer_bytes_))
    {
      RCLCPP_ERROR(
        get_logger(), "无法启动运行日志 collector (%s)", socket_path_.c_str());
      manifest_.finalize(false, "{}", "collector_start_failed");
      session_ok_ = false;
      return;
    }
    session_ok_ = true;
    RCLCPP_INFO(
      get_logger(), "runtime log session %s at %s",
      run_id_.c_str(), manifest_.run_dir().c_str());
  }

  void stop_session()
  {
    if (!session_ok_) {
      return;
    }
    collector_.stop();
    const auto & integ = collector_.integrity();
    const bool complete =
      integ.total_gaps() == 0 && collector_.writer().health().write_errors == 0;
    manifest_.finalize(
      complete, integ.to_json(complete), "shutdown");
    session_ok_ = false;
  }

  void publish_health()
  {
    RuntimeLogHealth msg;
    msg.stamp = now();
    msg.run_id = run_id_;
    msg.output_dir = session_ok_ ? manifest_.run_dir() : "";
    const auto s = collector_.stats();
    msg.recording = session_ok_;
    msg.records_received = s.accepted;
    msg.records_written = s.written;
    msg.records_durable = s.durable;
    msg.dropped_records = s.rejected;
    msg.gaps_detected = collector_.integrity().total_gaps();
    msg.write_errors = s.write_errors;
    msg.parse_errors = 0;
    for (const auto & kv : collector_.integrity().accounts()) {
      msg.parse_errors += kv.second.parse_errors;
    }
    msg.last_write_age_s = 0.0;
    msg.last_error = collector_.writer().health().last_error;
    msg.complete = !session_ok_;
    health_pub_->publish(msg);
  }

  bool enabled_{true};
  bool test_run_{false};
  std::string root_;
  std::string socket_path_;
  std::string run_id_;
  size_t buffer_bytes_{0};
  uint64_t batch_bytes_{0};
  uint64_t rotate_bytes_{0};
  int flush_ms_{100};
  int sync_ms_{1000};
  int shutdown_timeout_ms_{5000};
  bool session_ok_{false};

  runtime_logger::SessionManifest manifest_;
  runtime_logger::Collector collector_;
  rclcpp::Publisher<RuntimeLogHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RuntimeLoggerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
