// =============================================================
// runtime_telemetry_ui_node: 1 Hz aggregate of the latest UART
// telemetry + latency status into RuntimeTelemetryUi for rqt.
// No control authority: subscribe-only + one 1 Hz publisher.
// =============================================================

#include <rclcpp/rclcpp.hpp>

#include <my_project_interfaces/msg/extruder_latency_status.hpp>
#include <my_project_interfaces/msg/runtime_telemetry_ui.hpp>
#include <my_project_interfaces/msg/uart_telemetry_ui.hpp>

#include <chrono>
#include <mutex>
#include <optional>

using my_project_interfaces::msg::ExtruderLatencyStatus;
using my_project_interfaces::msg::RuntimeTelemetryUi;
using my_project_interfaces::msg::UartTelemetryUi;

class RuntimeTelemetryUiNode : public rclcpp::Node
{
public:
  RuntimeTelemetryUiNode()
  : Node("runtime_telemetry_ui_node")
  {
    const auto ui_period_ms = declare_parameter<int>("runtime_ui_period_ms", 1000);

    ui_pub_ = create_publisher<RuntimeTelemetryUi>(
      "/runtime/telemetry_ui", rclcpp::QoS(1).keep_last(1).best_effort());

    uart_sub_ = create_subscription<UartTelemetryUi>(
      "/uart/telemetry_ui", rclcpp::QoS(1).keep_last(1).best_effort(),
      [this](UartTelemetryUi::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_uart_ = *msg;
        last_uart_stamp_ = now();
      });

    latency_sub_ = create_subscription<ExtruderLatencyStatus>(
      "/extruder/latency_status", rclcpp::QoS(1).keep_last(1).best_effort(),
      [this](ExtruderLatencyStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_latency_ = *msg;
        last_latency_stamp_ = now();
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(ui_period_ms),
      [this]() {publish_snapshot();});
  }

private:
  void publish_snapshot()
  {
    RuntimeTelemetryUi out;
    out.stamp = now();
    std::lock_guard<std::mutex> lk(mutex_);
    if (last_uart_) {
      out.uart = *last_uart_;
    }
    if (last_latency_) {
      out.latency = *last_latency_;
      out.latency_valid = true;
      out.latency_age_s = (rclcpp::Time(out.stamp) - last_latency_stamp_).seconds();
    } else {
      out.latency_valid = false;
      out.latency_age_s = 0.0;
    }
    ui_pub_->publish(out);
  }

  std::mutex mutex_;
  std::optional<UartTelemetryUi> last_uart_;
  std::optional<ExtruderLatencyStatus> last_latency_;
  rclcpp::Time last_uart_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_latency_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<RuntimeTelemetryUi>::SharedPtr ui_pub_;
  rclcpp::Subscription<UartTelemetryUi>::SharedPtr uart_sub_;
  rclcpp::Subscription<ExtruderLatencyStatus>::SharedPtr latency_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RuntimeTelemetryUiNode>());
  rclcpp::shutdown();
  return 0;
}
