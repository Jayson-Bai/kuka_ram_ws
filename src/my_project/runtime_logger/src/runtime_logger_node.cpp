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
#include <cstdlib>
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
    // Default log root is $HOME/kuka_ram_ws/data/runtime_logs so the
    // same binary works on hosts with a different username or a
    // relocated workspace without a launch override.
    const char * home = std::getenv("HOME");
    const std::string default_root =
      (home && *home) ?
        std::string(home) + "/kuka_ram_ws/data/runtime_logs" :
        "/tmp/kuka_ram_ws/data/runtime_logs";
    root_ = declare_parameter<std::string>(
      "runtime_log_root", default_root);
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
    // RSI experiment metadata; empty values are omitted from manifest.
    rsi_xml_sha256_ = declare_parameter<std::string>("rsi_xml_sha256", "");
    rsi_context_ = declare_parameter<std::string>("rsi_context", "");
    rsi_pt1_t1_ = declare_parameter<std::string>("rsi_pt1_t1", "");
    rsi_precision_ = declare_parameter<std::string>("rsi_precision", "");
    rsi_tool_base_ = declare_parameter<std::string>("rsi_tool_base", "");
    rsi_spline_config_ = declare_parameter<std::string>("rsi_spline_config", "");
    rsi_extra_json_ = declare_parameter<std::string>("rsi_extra_json", "");
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
        build_config_json(), test_run_))
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

  std::string build_config_json()
  {
    std::ostringstream os;
    os << "{";
    bool first = true;
    auto field = [&](const char * k, const std::string & v) {
      if (v.empty()) {
        return;
      }
      if (!first) {
        os << ",";
      }
      first = false;
      os << "\"" << k << "\":\"" << v << "\"";
    };
    field("rsi_xml_sha256", rsi_xml_sha256_);
    field("rsi_context", rsi_context_);
    field("rsi_pt1_t1", rsi_pt1_t1_);
    field("rsi_precision", rsi_precision_);
    field("rsi_tool_base", rsi_tool_base_);
    field("rsi_spline_config", rsi_spline_config_);
    if (!rsi_extra_json_.empty()) {
      if (!first) {
        os << ",";
      }
      first = false;
      os << "\"extra\":" << rsi_extra_json_;
    }
    os << "}";
    return os.str();
  }

  bool enabled_{true};
  bool test_run_{false};
  std::string rsi_xml_sha256_;
  std::string rsi_context_;
  std::string rsi_pt1_t1_;
  std::string rsi_precision_;
  std::string rsi_tool_base_;
  std::string rsi_spline_config_;
  std::string rsi_extra_json_;
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
