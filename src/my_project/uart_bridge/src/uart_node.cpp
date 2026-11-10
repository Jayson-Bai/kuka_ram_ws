// =============================================================
// 描述：
// 单片机通信节点，UART串口协议
// 订阅RSI心跳（包含挤出）
// 发布接收到的打印头各模块状态消息，发布READY消息
//
// 运行日志架构（schema_version 1）：
//   - UART RX/TX 关键路径只执行非阻塞捕获（bounded ring + 时间戳）
//   - 独立 relay 线程做白名单投影、IPC 发送、ROS 观测/遥测 UI 发布
//   - 正式运行日志由 runtime_logger 节点经 AF_UNIX/SOCK_SEQPACKET 写盘
//   - 其他回传数据仅服务 1 Hz UartTelemetryUi 快照和 UI 显示
// =============================================================

#include "uart_bridge/extrusion_wire.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <std_msgs/msg/string.hpp>

#include <my_project_interfaces/msg/planned_event.hpp>
#include <my_project_interfaces/msg/runtime_log_config.hpp>
#include <my_project_interfaces/msg/rsi_heart_beat.hpp>
#include <my_project_interfaces/msg/print_head_status.hpp>
#include <my_project_interfaces/msg/uart_observation.hpp>
#include <my_project_interfaces/msg/uart_telemetry_ui.hpp>

#include <runtime_logging_core/bounded_ring.hpp>
#include <runtime_logging_core/capture_record.hpp>
#include <runtime_logging_core/ipc_client.hpp>
#include <runtime_logging_core/log_shutdown.hpp>
#include <runtime_logging_core/producer_status.hpp>
#include <runtime_logging_core/whitelist.hpp>

#include <boost/asio.hpp>
#include <array>
#include <thread>
#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <mutex>
#include <sstream>
#include <random>
#include <cmath>
#include <cstdint>

using boost::asio::io_context;
using boost::asio::serial_port;
using boost::asio::serial_port_base;

using my_project_interfaces::msg::RsiHeartBeat;
using my_project_interfaces::msg::PrintHeadStatus;
using my_project_interfaces::msg::PlannedEvent;
using my_project_interfaces::msg::RuntimeLogConfig;
using my_project_interfaces::msg::UartObservation;
using my_project_interfaces::msg::UartTelemetryUi;

using runtime_logging::BoundedRing;
using runtime_logging::CaptureRecord;
using runtime_logging::CaptureType;
using runtime_logging::Direction;
using runtime_logging::IpcClient;
using runtime_logging::Whitelist;

namespace {

std::string make_producer_epoch()
{
  std::random_device rd;
  std::ostringstream os;
  os << std::hex << rd() << rd();
  return os.str();
}

}  // namespace

class UartNode : public rclcpp::Node
{
private:
  io_context io_;
  serial_port serial_;
  std::atomic<bool> running_;
  std::thread read_thread_;
  std::string port_;
  int baudrate_;
  rclcpp::Subscription<PlannedEvent>::SharedPtr event_sub_;
  rclcpp::Subscription<RsiHeartBeat>::SharedPtr hb_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr print_test_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr uart_raw_pub_;
  rclcpp::Publisher<PrintHeadStatus>::SharedPtr ready_pub_;
  rclcpp::Publisher<UartObservation>::SharedPtr observation_pub_;
  rclcpp::Publisher<UartTelemetryUi>::SharedPtr telemetry_ui_pub_;

  mutable std::mutex event_mutex_;
  std::optional<PlannedEvent> current_event_;
  bool current_event_ack_received_{false};
  bool current_event_done_received_{false};
  uint32_t current_event_wire_id_{0};
  uint32_t last_cut_wire_id_{0};
  bool cut_done_received_{false};
  uint32_t cut_done_wire_id_{0};
  std::atomic<uint32_t> next_wire_request_id_{1};
  std::mutex status_mutex_;
  bool ready_for_motion_{true};
  uint32_t ready_event_seq_{0};
  std::string ready_event_type_;

  struct Status
  {
    float current_temp_cf{0}, target_temp_cf{0};
    float current_temp_resin{0}, target_temp_resin{0};
    bool fan_ok_cf{false}, fan_ok_resin{false};
    int32_t current_tool{0};
    int32_t error_code{0};
    int32_t last_e_seq{-1};
    double last_e_abs{0.0};
    uint64_t last_e_us{0};
  };
  std::mutex status_cache_mutex_;
  Status status_;
  std::string recv_buf_;
  std::mutex serial_write_mutex_;
  std::mutex extrude_forward_mutex_;
  std::unique_ptr<uart_bridge::ExtrusionForwarder> extrusion_forwarder_;
  std::atomic<double> extrude_scale_{1.0};
  std::atomic<bool> paused_{false};
  std::atomic<bool> aborted_{false};

  struct UniformChannel
  {
    bool initialised{false};
    double e_abs{0.0};
    rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
  };
  std::array<UniformChannel, 3> uniform_channels_{};
  std::string dispatch_policy_{"seq_bound"};
  double uniform_rate_tool1_mm_per_s_{10.0};
  double uniform_rate_tool2_mm_per_s_{3.945};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // ---- runtime logging ----
  bool runtime_log_enabled_{true};
  std::string runtime_log_socket_;
  std::string run_id_;
  std::string producer_id_{"uart_node"};
  std::string producer_epoch_;
  uint64_t capture_seq_{0};
  uint64_t record_seq_{0};
  std::atomic<bool> relay_running_{false};
  std::thread relay_thread_;
  std::thread ui_thread_;
  std::unique_ptr<BoundedRing<CaptureRecord>> capture_ring_;
  std::atomic<uint64_t> capture_dropped_{0};
  Whitelist whitelist_;
  IpcClient ipc_client_;
  std::mutex ipc_mutex_;
  std::atomic<bool> runtime_log_ready_{false};
  rclcpp::Subscription<RuntimeLogConfig>::SharedPtr runtime_log_config_sub_;
  std::atomic<uint64_t> log_records_sent_{0};
  uint64_t first_log_record_seq_{0};
  int shutdown_drain_ms_{runtime_logging::kDefaultShutdownDrainMs};
  std::mutex ui_cache_mutex_;
  UartTelemetryUi ui_cache_;
  bool ui_cache_valid_{false};
  rclcpp::Time ui_cache_stamp_{0, 0, RCL_ROS_TIME};

public:
  UartNode()
  : Node("uart_node"), io_(), serial_(io_), running_(true)
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    extrude_scale_.store(declare_parameter<double>("extrude_scale", 1.0));
    rcl_interfaces::msg::ParameterDescriptor extrusion_wire_mode_descriptor;
    extrusion_wire_mode_descriptor.read_only = true;
    const auto extrusion_wire_mode = declare_parameter<std::string>(
      "extrusion_wire_mode", "canonical_v1", extrusion_wire_mode_descriptor);
    const auto extrusion_mode =
      uart_bridge::parse_extrusion_wire_mode(extrusion_wire_mode);
    extrusion_forwarder_ = std::make_unique<uart_bridge::ExtrusionForwarder>(extrusion_mode);
    rcl_interfaces::msg::ParameterDescriptor dispatch_policy_descriptor;
    dispatch_policy_descriptor.read_only = true;
    dispatch_policy_ = declare_parameter<std::string>(
      "dispatch_policy", "seq_bound", dispatch_policy_descriptor);
    rcl_interfaces::msg::ParameterDescriptor uniform_rate_descriptor;
    uniform_rate_descriptor.read_only = true;
    uniform_rate_tool1_mm_per_s_ = declare_parameter<double>(
      "uniform_rate_tool1_mm_per_s", 10.0, uniform_rate_descriptor);
    uniform_rate_tool2_mm_per_s_ = declare_parameter<double>(
      "uniform_rate_tool2_mm_per_s", 3.945, uniform_rate_descriptor);
    if (dispatch_policy_ != "seq_bound" &&
      dispatch_policy_ != "uniform_feedforward")
    {
      throw std::runtime_error(
              "unsupported dispatch_policy: " + dispatch_policy_);
    }
    if (dispatch_policy_ == "uniform_feedforward" &&
      (!(uniform_rate_tool1_mm_per_s_ > 0.0) ||
       !(uniform_rate_tool2_mm_per_s_ > 0.0)))
    {
      throw std::runtime_error(
              "uniform_rate_tool1_mm_per_s and uniform_rate_tool2_mm_per_s "
              "must be > 0 when dispatch_policy=uniform_feedforward");
    }
    RCLCPP_INFO(
      get_logger(),
      "UART dispatch policy: %s (tool1 %.3f mm/s, tool2 %.3f mm/s)",
      dispatch_policy_.c_str(),
      uniform_rate_tool1_mm_per_s_,
      uniform_rate_tool2_mm_per_s_);
    RCLCPP_INFO(
      get_logger(), "UART extrusion wire mode: %s",
      uart_bridge::extrusion_wire_mode_name(extrusion_mode));
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "extrude_scale") {
            const double v = p.as_double();
            if (!std::isfinite(v) || v <= 0.0) {
              rcl_interfaces::msg::SetParametersResult r;
              r.successful = false;
              r.reason = "extrude_scale must be finite and > 0";
              return r;
            }
            extrude_scale_.store(v);
          }
        }
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = true;
        return r;
      }
    );

    //订阅心跳包

    // ---- runtime logging parameters ----
    runtime_log_enabled_ = declare_parameter<bool>("runtime_log_enabled", true);
    run_id_ = declare_parameter<std::string>("run_id", "");
    producer_epoch_ = make_producer_epoch();
    runtime_log_socket_ = declare_parameter<std::string>("runtime_log_socket", "");
    shutdown_drain_ms_ = declare_parameter<int>(
      "runtime_log_shutdown_timeout_ms",
      runtime_logging::kDefaultShutdownDrainMs);
    const auto ring_capacity = declare_parameter<int>(
      "runtime_log_ring_capacity", 1024);
    capture_ring_ = std::make_unique<BoundedRing<CaptureRecord>>(
      static_cast<size_t>(ring_capacity));
    if (runtime_log_socket_.empty()) {
      if (runtime_log_enabled_) {
        runtime_log_config_sub_ = create_subscription<RuntimeLogConfig>(
          "/runtime/log_config",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
          [this](RuntimeLogConfig::SharedPtr msg) {
            if (runtime_log_ready_.load(std::memory_order_acquire) ||
            !msg->enabled || msg->socket_path.empty())
            {
              return;
            }
            run_id_ = msg->run_id;
            runtime_log_socket_ = msg->socket_path;
            runtime_log_ready_.store(true, std::memory_order_release);
            RCLCPP_INFO(
              get_logger(), "runtime log config received: run_id=%s socket=%s",
              run_id_.c_str(), runtime_log_socket_.c_str());
          });
      }
    } else {
      if (run_id_.empty()) {
        run_id_ = "uart_" + producer_epoch_;
      }
      runtime_log_ready_.store(true, std::memory_order_release);
    }
    if (runtime_log_enabled_) {
      relay_running_.store(true);
      relay_thread_ = std::thread(&UartNode::relay_loop, this);
      ui_thread_ = std::thread(&UartNode::ui_loop, this);
      RCLCPP_INFO(
        get_logger(), "runtime logging enabled, socket=%s",
        runtime_log_socket_.empty() ? "<waiting for logger broadcast>" :
        runtime_log_socket_.c_str());
    }

    auto hb_qos = rclcpp::QoS(2000).reliable();
    auto event_qos = rclcpp::QoS(200).reliable();
    auto pub_qos = rclcpp::QoS(200).reliable();
    auto ready_qos = rclcpp::QoS(200).reliable().transient_local();
    hb_sub_ = create_subscription<RsiHeartBeat>(
      "/rsi/heartbeat", hb_qos,
      [this](RsiHeartBeat::SharedPtr msg) { on_heartbeat(*msg); });
    event_sub_ = create_subscription<PlannedEvent>(
      "/rsi/triggered_event", event_qos,
      [this](PlannedEvent::SharedPtr msg) { on_triggered_event(*msg); });
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "/system/command", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        on_system_command(msg->data);
      });
    manual_cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "/uart/manual_command", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "收到手动控制命令: %s", msg->data.c_str());
        write_line(msg->data);
      });
    print_test_cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "/print_test/rsi_command", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        on_print_test_command(msg->data);
      });
    uart_raw_pub_ = create_publisher<std_msgs::msg::String>("/uart/raw", pub_qos);
    observation_pub_ = create_publisher<UartObservation>(
      "/uart/observation", rclcpp::QoS(512).reliable());
    telemetry_ui_pub_ = create_publisher<UartTelemetryUi>(
      "/uart/telemetry_ui", rclcpp::QoS(1).keep_last(1).best_effort());
    ready_pub_ = create_publisher<PrintHeadStatus>("/printhead/status", ready_qos);
    publish_ready_state("startup");
    open_serial();
    read_thread_ = std::thread(&UartNode::read_loop, this);
  }

  ~UartNode() override
  {
    running_.store(false);
    boost::system::error_code ec;
    serial_.cancel(ec);
    serial_.close(ec);
    io_.stop();
    if (read_thread_.joinable()) {read_thread_.join();}
    if (relay_running_.load()) {
      relay_running_.store(false);
      if (relay_thread_.joinable()) {relay_thread_.join();}
      if (ui_thread_.joinable()) {ui_thread_.join();}
    }
    std::lock_guard<std::mutex> lk(ipc_mutex_);
    ipc_client_.close();
  }

private:
  void open_serial()
  {
    boost::system::error_code ec;
    serial_.open(port_, ec);
    if (ec) {
      RCLCPP_FATAL(get_logger(), "打开%s失败：%s", port_.c_str(), ec.message().c_str());
      throw std::runtime_error("串口打开失败");
    }
    serial_.set_option(serial_port_base::baud_rate(baudrate_));
    serial_.set_option(serial_port_base::character_size(8));
    serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
  }

  void set_ready_state(bool ready_for_motion, uint32_t event_seq, const std::string & event_type)
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    ready_for_motion_ = ready_for_motion;
    ready_event_seq_ = event_seq;
    ready_event_type_ = event_type;
  }

  void publish_ready_state(const std::string & raw)
  {
    PrintHeadStatus st;
    st.stamp = now();
    {
      std::lock_guard<std::mutex> lk(status_mutex_);
      st.ready_for_motion = ready_for_motion_;
      st.ready_event_seq = ready_event_seq_;
      st.ready_event_type = ready_event_type_;
    }
    {
      std::lock_guard<std::mutex> lk(status_cache_mutex_);
      st.fan_ok_cf = status_.fan_ok_cf;
      st.fan_ok_resin = status_.fan_ok_resin;
      st.current_temp_cf = status_.current_temp_cf;
      st.target_temp_cf = status_.target_temp_cf;
      st.current_temp_resin = status_.current_temp_resin;
      st.target_temp_resin = status_.target_temp_resin;
      st.current_tool = status_.current_tool;
      st.error_code = status_.error_code;
    }
    st.raw = raw;
    ready_pub_->publish(st);
  }

  void on_triggered_event(const PlannedEvent & ev)
  {
    const uint32_t wire_request_id =
      next_wire_request_id_.fetch_add(1, std::memory_order_relaxed);
    const std::string arg = ev.payload.empty() ? "0" : ev.payload;
    std::ostringstream oss;
    {
      std::lock_guard<std::mutex> lk(event_mutex_);
      if (ev.event_type == "cut") {
        oss << "EV " << wire_request_id << " cut " << arg << "\n";
        last_cut_wire_id_ = wire_request_id;
        cut_done_received_ = false;
        cut_done_wire_id_ = 0;
      } else if (ev.event_type == "extrude_sync") {
        std::istringstream payload_stream(ev.payload);
        int tool_id = 0;
        double target_mm = 0.0;
        uint32_t timeout_ms = 0;
        if (!(payload_stream >> tool_id >> target_mm >> timeout_ms) ||
          (tool_id != 1 && tool_id != 2) || timeout_ms == 0U ||
          !std::isfinite(target_mm))
        {
          RCLCPP_ERROR(
            get_logger(), "extrude_sync payload 非法: '%s'", ev.payload.c_str());
          publish_ready_state("event_fault:invalid_sync_payload");
          return;
        }
        const int64_t target_um = static_cast<int64_t>(
          std::llround(target_mm * extrude_scale_.load() * 1000000.0));
        oss << "EV " << wire_request_id << " extrude_sync " << tool_id << " "
            << target_um << " " << timeout_ms << " " << last_cut_wire_id_ << "\n";
        current_event_ = ev;
        current_event_ack_received_ = false;
        current_event_done_received_ = false;
        current_event_wire_id_ = wire_request_id;
      } else {
        oss << "EV " << wire_request_id << " " << ev.event_type << " " << arg << "\n";
        current_event_ = ev;
        current_event_ack_received_ = false;
        current_event_done_received_ = false;
        current_event_wire_id_ = wire_request_id;
      }
    }
    if (ev.event_type == "heat_cf" || ev.event_type == "heat_resin") {
      try {
        float target = std::stof(ev.payload);
        std::lock_guard<std::mutex> lk(status_cache_mutex_);
        if (ev.event_type == "heat_cf") {
          status_.target_temp_cf = target;
        } else {
          status_.target_temp_resin = target;
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "温控参数解析失败：%s", e.what());
      }
    }
    write_line(oss.str());
    std::string ev_info = "event:" + ev.event_type;
    if (!ev.payload.empty()) {
      ev_info += " payload=" + ev.payload;
    }
    if (ev.event_type == "cut") {
      set_ready_state(true, ev.trigger_seq, ev.event_type);
      publish_ready_state(ev_info + " nonblocking");
      return;
    }
    set_ready_state(false, ev.trigger_seq, ev.event_type);
    publish_ready_state(ev_info);
  }

  bool has_pending_event() const
  {
    std::lock_guard<std::mutex> lk(event_mutex_);
    return current_event_.has_value();
  }

  void reset_extrude_forward_state()
  {
    std::lock_guard<std::mutex> lk(extrude_forward_mutex_);
    extrusion_forwarder_->reset();
    for (auto & ch : uniform_channels_) {
      ch = UniformChannel{};
    }
  }

  void on_heartbeat(const RsiHeartBeat & hb)
  {
    if (paused_.load() || aborted_.load() || has_pending_event()) {
      return;
    }
    const double scale = extrude_scale_.load();
    double e_for_line = hb.extrude_abs;
    if (dispatch_policy_ == "uniform_feedforward") {
      auto & ch = uniform_channels_[
        (hb.tool_id >= 0 && hb.tool_id <= 2) ? hb.tool_id : 0];
      if (!ch.initialised) {
        ch.e_abs = hb.extrude_abs;
        ch.last_stamp = rclcpp::Time(hb.stamp);
        ch.initialised = true;
      } else {
        const double dt =
          (rclcpp::Time(hb.stamp) - ch.last_stamp).seconds();
        if (dt > 0.0) {
          const double rate = (hb.tool_id == 2)
            ? uniform_rate_tool2_mm_per_s_
            : uniform_rate_tool1_mm_per_s_;
          ch.e_abs += rate * dt;
          ch.last_stamp = rclcpp::Time(hb.stamp);
        }
      }
      e_for_line = ch.e_abs;
    }
    const double scaled = e_for_line * scale;
    std::lock_guard<std::mutex> lk(extrude_forward_mutex_);
    const auto prepared = extrusion_forwarder_->prepare(hb.seq_used, hb.tool_id, scaled);
    if (
      prepared.decision == uart_bridge::ExtrusionDecision::RejectNonFinite ||
      prepared.decision == uart_bridge::ExtrusionDecision::RejectOutOfRange)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "拒绝挤出值：seq=%u tool=%d value=%.17g",
        hb.seq_used, hb.tool_id, scaled);
      return;
    }
    if (prepared.decision != uart_bridge::ExtrusionDecision::Send) {
      return;
    }
    const auto candidate = prepared.candidate.value();
    if (write_line(candidate.line)) {
      extrusion_forwarder_->commit(candidate);
    }
  }

  void read_loop()
  {
    std::array<char, 256> buf{};
    while (rclcpp::ok() && running_.load()) {
      boost::system::error_code ec;
      std::size_t n = serial_.read_some(boost::asio::buffer(buf), ec);
      if (ec) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "读取错误：%s", ec.message().c_str());
        continue;
      }
      recv_buf_.append(buf.data(), n);
      while (true) {
        auto pos = recv_buf_.find('\n');
        if (pos == std::string::npos) {break;}
        std::string line = recv_buf_.substr(0, pos);
        recv_buf_.erase(0, pos + 1);
        if (!line.empty() && line.back() == '\r') {line.pop_back();}
        // Capture first so the source timestamp is stamped before any
        // downstream processing; the ring push never blocks.
        capture_line(line, Direction::kRx, true, 0);
        handle_line(line);
      }
    }
  }

  void handle_line(const std::string & line)
  {
    publish_uart_log("RX", line);
    if (line.rfind("STAT", 0) == 0) {
      parse_stat(line.substr(4));
      publish_ready_state(line);
      check_event_completion();
      return;
    }
    if (line.rfind("EVACK ", 0) == 0) {
      handle_event_response(line, false);
      return;
    }
    if (line.rfind("EVDONE ", 0) == 0) {
      handle_event_response(line, true);
      return;
    }
  }

  void publish_uart_log(const std::string & direction, const std::string & line)
  {
    std_msgs::msg::String raw;
    raw.data = direction + " " + line;
    while (!raw.data.empty() && (raw.data.back() == '\n' || raw.data.back() == '\r')) {
      raw.data.pop_back();
    }
    uart_raw_pub_->publish(raw);
  }

  void handle_event_response(const std::string & line, bool done_response)
  {
    std::istringstream ss(line);
    std::string tag;
    std::string event_type;
    uint32_t event_seq{0};
    ss >> tag >> event_seq >> event_type;
    {
      std::lock_guard<std::mutex> lk(event_mutex_);
      if (event_type == "cut" && event_seq == last_cut_wire_id_) {
        if (done_response) {
          cut_done_received_ = true;
          cut_done_wire_id_ = event_seq;
        }
      } else if (current_event_ &&
        current_event_wire_id_ == event_seq &&
        current_event_->event_type == event_type)
      {
        if (done_response) {
          current_event_done_received_ = true;
        } else {
          current_event_ack_received_ = true;
        }
      }
    }
    publish_ready_state(line);
    check_event_completion();
  }

  void parse_stat(const std::string & payload)
  {
    std::istringstream ss(payload);
    std::string kv;
    std::lock_guard<std::mutex> lk(status_cache_mutex_);
    while (ss >> kv) {
      auto eq = kv.find('=');
      if (eq == std::string::npos) {continue;}
      auto key = kv.substr(0, eq);
      auto val = kv.substr(eq + 1);
      try {
        if (key == "temp_cf") {
          status_.current_temp_cf = std::stof(val);
        } else if (key == "temp_resin") {
          status_.current_temp_resin = std::stof(val);
        } else if (key == "target_cf") {
          status_.target_temp_cf = std::stof(val);
        } else if (key == "target_resin") {
          status_.target_temp_resin = std::stof(val);
        } else if (key == "fan_ok_cf") {
          status_.fan_ok_cf = (val == "1");
        } else if (key == "fan_ok_resin") {
          status_.fan_ok_resin = (val == "1");
        } else if (key == "tool") {status_.current_tool = std::stoi(val);} else if (key == "err") {
          status_.error_code = std::stoi(val);
        } else if (key == "last_e_seq") {
          status_.last_e_seq = std::stoi(val);
        } else if (key == "last_e_abs") {
          status_.last_e_abs = std::stod(val);
        } else if (key == "last_e_us") {
          status_.last_e_us = static_cast<uint64_t>(std::stoull(val));
        }
      } catch (...) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "字段解析失败：%s", kv.c_str());
      }
    }
  }

  void check_event_completion()
  {
    std::lock_guard<std::mutex> lk(event_mutex_);
    if (!current_event_) {return;}
    const auto & ev = *current_event_;
    bool done = false;
    constexpr float TEMP_EPS = 20.0f;
    Status snapshot;
    {
      std::lock_guard<std::mutex> sk(status_cache_mutex_);
      snapshot = status_;
    }
    if (ev.event_type == "heat_cf") {
      done = std::abs(snapshot.current_temp_cf - snapshot.target_temp_cf) <= TEMP_EPS;
    } else if (ev.event_type == "heat_resin") {
      done = std::abs(snapshot.current_temp_resin - snapshot.target_temp_resin) <= TEMP_EPS;
    } else if (ev.event_type == "tool_change_cf" || ev.event_type == "tool_change_resin") {
      try {
        int target_tool = std::stoi(ev.payload);
        done = snapshot.current_tool == target_tool;
      } catch (const std::exception &) {
        done = false;
      }
    } else if (ev.event_type == "fan_cf") {
      try {
        bool target = std::stoi(ev.payload) != 0;
        done = (snapshot.fan_ok_cf == target);
      } catch (...) {
        RCLCPP_WARN(get_logger(), "fan_cf事件payload格式错误: '%s'，跳过等待", ev.payload.c_str());
        done = true;
      }
    } else if (ev.event_type == "fan_resin") {
      try {
        bool target = std::stoi(ev.payload) != 0;
        done = (snapshot.fan_ok_resin == target);
      } catch (...) {
        RCLCPP_WARN(get_logger(), "fan_resin事件payload格式错误: '%s'，跳过等待", ev.payload.c_str());
        done = true;
      }
    } else if (ev.event_type == "extrude_reset") {
      done = current_event_done_received_;
    } else if (ev.event_type == "extrude_sync") {
      done = current_event_ack_received_ && current_event_done_received_ &&
        cut_done_received_ && cut_done_wire_id_ == last_cut_wire_id_;
    }

    if (done) {
      set_ready_state(true, ev.trigger_seq, ev.event_type);
      publish_ready_state("event_done");
      if (ev.event_type == "extrude_reset") {
        reset_extrude_forward_state();
      }
      current_event_.reset();
    }
  }

  bool write_line(const std::string & line)
  {
    std::lock_guard<std::mutex> lk(serial_write_mutex_);
    boost::system::error_code ec;
    const std::size_t written = boost::asio::write(serial_, boost::asio::buffer(line), ec);
    const bool ok = !ec && written == line.size();
    if (!ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "写入失败：%s (written=%zu expected=%zu)",
        ec ? ec.message().c_str() : "incomplete write",
        written, line.size());
    }
    // Capture the actual write result on the TX path; this is the sole
    // source of e_tx log records (no pre-write "attempt" lines).
    capture_line(
      line, Direction::kTx, ok, static_cast<uint32_t>(written));
    publish_uart_log("TX", line);
    return ok;
  }

  void on_print_test_command(const std::string & cmd)
  {
    if (cmd == "RESET") {
      {
        std::lock_guard<std::mutex> lk(event_mutex_);
        current_event_ack_received_ = false;
        current_event_done_received_ = false;
        cut_done_received_ = false;
        cut_done_wire_id_ = 0;
      }
      reset_extrude_forward_state();
      RCLCPP_INFO(get_logger(), "测试模式RESET：复位UART事件ACK/DONE临时状态");
    }
  }

  void on_system_command(const std::string & cmd)
  {
    if (cmd == "PAUSE") {
      paused_.store(true);
      RCLCPP_INFO(get_logger(), "uart_node收到PAUSE命令，停止挤出转发");
    } else if (cmd == "RESUME") {
      paused_.store(false);
      RCLCPP_INFO(get_logger(), "uart_node收到RESUME命令，恢复挤出转发");
    } else if (cmd == "ABORT") {
      aborted_.store(true);
      paused_.store(true);
      write_line("EV 0 heat_cf 0\n");
      write_line("EV 0 heat_resin 0\n");
      {
        std::lock_guard<std::mutex> lk(status_cache_mutex_);
        status_.target_temp_cf = 0.0f;
        status_.target_temp_resin = 0.0f;
      }
      RCLCPP_WARN(get_logger(), "uart_node收到ABORT命令，停止挤出并关闭加热");
    }
  }

  // ---- runtime logging internals ----

  void capture_line(
    const std::string & line, Direction dir, bool tx_ok,
    uint32_t bytes_written)
  {
    if (!runtime_log_enabled_) {
      return;
    }
    CaptureRecord rec;
    rec.capture_seq = ++capture_seq_;
    rec.host_mono_ns = runtime_logging::clock_monotonic_ns();
    rec.direction = dir;
    rec.type = runtime_logging::classify_line(line, dir);
    rec.tx_ok = tx_ok;
    rec.bytes_written = bytes_written;
    rec.line = line;
    while (!rec.line.empty() &&
      (rec.line.back() == '\n' || rec.line.back() == '\r'))
    {
      rec.line.pop_back();
    }
    if (!capture_ring_->try_push(std::move(rec))) {
      capture_dropped_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  void relay_loop()
  {
    auto next_connect = std::chrono::steady_clock::now();
    auto next_status = std::chrono::steady_clock::now();
    bool status_hello_sent = false;
    while (relay_running_.load()) {
      if (!runtime_log_ready_.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      bool did_work = false;
      while (auto item = capture_ring_->try_pop()) {
        did_work = true;
        process_capture(*item);
      }
      {
        std::lock_guard<std::mutex> lk(ipc_mutex_);
        if (!ipc_client_.connected()) {
          status_hello_sent = false;
          if (std::chrono::steady_clock::now() >= next_connect) {
            if (ipc_client_.connect_to(runtime_log_socket_)) {
              RCLCPP_INFO(
                get_logger(), "runtime log IPC connected: %s",
                runtime_log_socket_.c_str());
            }
            next_connect = std::chrono::steady_clock::now() +
              std::chrono::milliseconds(500);
          }
        }
        if (ipc_client_.connected()) {
          ipc_client_.drain();
        }
      }
      if (ipc_client_.connected() &&
        std::chrono::steady_clock::now() >= next_status)
      {
        send_producer_status(status_hello_sent ? "periodic" : "hello");
        status_hello_sent = true;
        next_status = std::chrono::steady_clock::now() +
          std::chrono::seconds(1);
      }
      if (!did_work) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
    if (runtime_log_ready_.load(std::memory_order_acquire)) {
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(shutdown_drain_ms_);
      while (std::chrono::steady_clock::now() < deadline) {
        auto item = capture_ring_->try_pop();
        if (!item) {break;}
        process_capture(*item);
      }
      const size_t leftover = capture_ring_->size();
      if (leftover > 0) {
        capture_dropped_.fetch_add(leftover);
        RCLCPP_WARN(
          get_logger(),
          "runtime log shutdown: %zu uart captures left in ring (dropped)",
          leftover);
      }
      send_final_producer_status(deadline);
    }
  }

  void process_capture(const CaptureRecord & rec)
  {
    auto records = whitelist_.project(rec, record_seq_);
    for (auto & w : records) {
      w.record_seq = record_seq_++;
      if (w.valid) {
        const std::string json = encode_record_json(
          w, run_id_, producer_id_, 1);
        std::lock_guard<std::mutex> lk(ipc_mutex_);
        if (ipc_client_.queue_message(json)) {
          if (log_records_sent_.load() == 0) {
            first_log_record_seq_ = w.record_seq;
          }
          ++log_records_sent_;
        }
      }
    }
    UartObservation obs;
    obs.host_mono_ns = rec.host_mono_ns;
    obs.host_ros_stamp = now();
    obs.capture_seq = rec.capture_seq;
    obs.producer_epoch = producer_epoch_;
    obs.direction = rec.direction == Direction::kTx ?
      UartObservation::DIRECTION_TX : UartObservation::DIRECTION_RX;
    obs.line = rec.line;
    obs.tx_ok = rec.tx_ok;
    observation_pub_->publish(obs);
    update_ui_cache(rec);
  }

  void send_producer_status(const char * phase)
  {
    runtime_logging::ProducerStatus st;
    st.producer_id = producer_id_;
    st.run_id = run_id_;
    st.producer_epoch = 1;
    st.phase = phase;
    st.first_record_seq = first_log_record_seq_;
    st.last_record_seq =
      log_records_sent_.load() == 0 ? 0 : record_seq_ - 1;
    st.sent_records = log_records_sent_.load();
    st.ring_dropped = capture_dropped_.load();
    st.ipc_dropped = ipc_client_.dropped();
    st.send_errors = ipc_client_.send_errors();
    st.pending_messages = ipc_client_.pending();
    st.pending_bytes = ipc_client_.pending_bytes();
    st.connected = ipc_client_.connected();
    const std::string json = runtime_logging::encode_producer_status_json(st);
    std::lock_guard<std::mutex> lk(ipc_mutex_);
    ipc_client_.queue_message(json);
    if (ipc_client_.connected()) {
      ipc_client_.drain();
    }
  }

  void send_final_producer_status(
    const std::chrono::steady_clock::time_point & deadline)
  {
    {
      std::lock_guard<std::mutex> lk(ipc_mutex_);
      if (!ipc_client_.connected()) {
        ipc_client_.connect_to(runtime_log_socket_);
      }
    }
    send_producer_status("final");
    while (std::chrono::steady_clock::now() < deadline) {
      bool empty = false;
      {
        std::lock_guard<std::mutex> lk(ipc_mutex_);
        if (!ipc_client_.connected()) {break;}
        ipc_client_.drain();
        empty = ipc_client_.pending() == 0;
      }
      if (empty) {break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::lock_guard<std::mutex> lk(ipc_mutex_);
    if (ipc_client_.pending() > 0 || !ipc_client_.connected()) {
      RCLCPP_WARN(
        get_logger(),
        "runtime log shutdown: final status not fully flushed "
        "(pending=%zu bytes, connected=%d, errno=%d)",
        ipc_client_.pending_bytes(), ipc_client_.connected() ? 1 : 0,
        ipc_client_.last_errno());
    }
  }

  void update_ui_cache(const CaptureRecord & rec)
  {
    std::lock_guard<std::mutex> lk(ui_cache_mutex_);
    if (rec.type == CaptureType::kStat) {
      parse_ui_stat(rec.line);
    } else if (rec.type == CaptureType::kEstat) {
      parse_ui_estat(rec.line, false);
    } else if (rec.type == CaptureType::kEstatHot) {
      parse_ui_estat(rec.line, true);
    } else if (rec.type == CaptureType::kUstat) {
      parse_ui_ustat(rec.line);
    }
    ui_cache_stamp_ = now();
    ui_cache_valid_ = true;
  }

  void parse_ui_stat(const std::string & line)
  {
    std::istringstream ss(line.substr(5));
    std::string kv;
    while (ss >> kv) {
      const auto eq = kv.find('=');
      if (eq == std::string::npos) {continue;}
      const auto key = kv.substr(0, eq);
      const auto val = kv.substr(eq + 1);
      try {
        if (key == "temp_cf") {ui_cache_.current_temp_cf = std::stof(val);}
        else if (key == "temp_resin") {ui_cache_.current_temp_resin = std::stof(val);}
        else if (key == "target_cf") {ui_cache_.target_temp_cf = std::stof(val);}
        else if (key == "target_resin") {ui_cache_.target_temp_resin = std::stof(val);}
        else if (key == "fan_ok_cf") {ui_cache_.fan_ok_cf = (val == "1");}
        else if (key == "fan_ok_resin") {ui_cache_.fan_ok_resin = (val == "1");}
        else if (key == "tool") {ui_cache_.current_tool = std::stoi(val);}
        else if (key == "err") {ui_cache_.error_code = std::stoi(val);}
        else if (key == "last_e_seq") {ui_cache_.last_e_seq = std::stoi(val);}
        else if (key == "last_e_tool") {ui_cache_.last_e_tool = std::stoi(val);}
        else if (key == "last_e_abs") {ui_cache_.last_e_abs = std::stod(val);}
        else if (key == "last_e_us") {ui_cache_.last_e_us = std::stoull(val);}
      } catch (...) {
      }
    }
    ui_cache_.stat_valid = true;
    ui_cache_.stat_age_s = 0.0;
  }

  void parse_ui_estat(const std::string & line, bool hot_only)
  {
    const size_t tag_len = hot_only ? 8 : 6;
    std::istringstream ss(line.substr(tag_len));
    std::string kv;
    while (ss >> kv) {
      const auto eq = kv.find('=');
      if (eq == std::string::npos) {continue;}
      const auto key = kv.substr(0, eq);
      const auto val = kv.substr(eq + 1);
      try {
        if (key == "tool") {ui_cache_.estat_tool = std::stoi(val);}
        else if (key == "state") {ui_cache_.estat_state = val;}
        else if (key == "target_steps") {ui_cache_.target_steps = std::stoll(val);}
        else if (key == "submitted_steps") {ui_cache_.submitted_steps = std::stoll(val);}
        else if (key == "completed_steps") {ui_cache_.completed_steps = std::stoll(val);}
        else if (key == "host_total_um") {ui_cache_.host_total_um = std::stoll(val);}
        else if (key == "last_cmd_us") {ui_cache_.last_cmd_us = std::stoull(val);}
        else if (key == "ff_until_us") {ui_cache_.ff_until_us = std::stoull(val);}
        else if (key == "ff_sps") {ui_cache_.ff_sps = std::stod(val);}
        else if (key == "v_sps") {ui_cache_.v_sps = std::stod(val);}
        else if (key == "phase") {ui_cache_.phase = std::stod(val);}
        else if (key == "last_seq") {ui_cache_.last_seq = std::stoul(val);}
        else if (key == "fault") {ui_cache_.fault = std::stoul(val, nullptr, 0);}
        else if (key == "hard_fault") {ui_cache_.hard_fault = std::stoul(val, nullptr, 0);}
        else if (key == "rmt_submit_err") {ui_cache_.rmt_submit_err = std::stoul(val);}
        else if (key == "rmt_done_err") {ui_cache_.rmt_done_err = std::stoul(val);}
        else if (key == "qdepth") {ui_cache_.qdepth = std::stoul(val);}
        else if (key == "dir") {ui_cache_.dir = std::stoi(val);}
        else if (key == "enabled") {ui_cache_.enabled = (val == "1");}
        else if (key == "hold_latched") {ui_cache_.hold_latched = (val == "1");}
        else if (key == "host_valid") {ui_cache_.host_valid = (val == "1");}
      } catch (...) {
      }
    }
    ui_cache_.estat_valid = true;
    ui_cache_.estat_age_s = 0.0;
  }

  void parse_ui_ustat(const std::string & line)
  {
    std::istringstream ss(line.substr(6));
    std::string kv;
    while (ss >> kv) {
      const auto eq = kv.find('=');
      if (eq == std::string::npos) {continue;}
      const auto key = kv.substr(0, eq);
      const auto val = kv.substr(eq + 1);
      try {
        if (key == "fifo_ovf") {ui_cache_.fifo_ovf = std::stoul(val);}
        else if (key == "buffer_full") {ui_cache_.buffer_full = std::stoul(val);}
        else if (key == "line_drop") {ui_cache_.line_drop = std::stoul(val);}
        else if (key == "cmd_drop") {ui_cache_.cmd_drop = std::stoul(val);}
        else if (key == "old_seq") {ui_cache_.old_seq = std::stoul(val);}
        else if (key == "seq_gap") {ui_cache_.seq_gap = std::stoul(val);}
        else if (key == "parse_err") {ui_cache_.parse_err = std::stoul(val);}
      } catch (...) {
      }
    }
    ui_cache_.ustat_valid = true;
    ui_cache_.ustat_age_s = 0.0;
  }

  void ui_loop()
  {
    while (relay_running_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      UartTelemetryUi out;
      {
        std::lock_guard<std::mutex> lk(ui_cache_mutex_);
        if (!ui_cache_valid_) {
          continue;
        }
        out = ui_cache_;
        const auto age = (now() - ui_cache_stamp_).seconds();
        out.stamp = now();
        out.stat_age_s = age;
        out.estat_age_s = age;
        out.ustat_age_s = age;
      }
      telemetry_ui_pub_->publish(out);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UartNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
