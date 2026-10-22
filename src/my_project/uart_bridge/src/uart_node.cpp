// =============================================================
// 描述：
// 单片机通信节点，UART串口协议
// 订阅RSI心跳（包含挤出）
// 发布接收到的打印头各模块状态消息，发布READY消息
// =============================================================

#include "uart_bridge/extrusion_wire.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <std_msgs/msg/string.hpp>

#include <my_project_interfaces/msg/planned_event.hpp>
#include <my_project_interfaces/msg/rsi_heart_beat.hpp>
#include <my_project_interfaces/msg/print_head_status.hpp>

#include <boost/asio.hpp>//串口
#include <array>
#include <thread>
#include <atomic>
#include <memory>
#include <optional>
#include <mutex>
#include <sstream>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <limits>
#include <vector>

using boost::asio::io_context;
using boost::asio::serial_port;
using boost::asio::serial_port_base;

using my_project_interfaces::msg::RsiHeartBeat;
using my_project_interfaces::msg::PrintHeadStatus;
using my_project_interfaces::msg::PlannedEvent;

class UartNode : public rclcpp::Node
{
private:
  //串口
  io_context io_;
  serial_port serial_;
  std::atomic<bool> running_;
  std::thread read_thread_;
  std::string port_;
  int baudrate_;
  //ROS
  rclcpp::Subscription<PlannedEvent>::SharedPtr event_sub_;
  rclcpp::Subscription<RsiHeartBeat>::SharedPtr hb_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr print_test_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr uart_raw_pub_;
  rclcpp::Publisher<PrintHeadStatus>::SharedPtr ready_pub_;

  // 事件使用独立线号关联 ACK/DONE；RSI 的 trigger_seq 只用于轨迹就绪对齐。
  struct PendingEvent
  {
    PlannedEvent planned;
    uint32_t wire_request_id{0};
    bool ack_received{false};
    bool done_received{false};
    bool failed{false};
    std::chrono::steady_clock::time_point sent_at;
    std::chrono::milliseconds timeout{60000};
    std::optional<int64_t> expected_target_um;
    std::optional<int> expected_tool;
    std::optional<uint32_t> cut_request_id;
  };
  struct PendingCut
  {
    uint32_t trigger_seq{0};
    uint32_t wire_request_id{0};
    bool ack_received{false};
    bool done_received{false};
    bool failed{false};
    uint32_t expected_sync_count{0};
    uint32_t sync_done_count{0};
    std::chrono::steady_clock::time_point sent_at;
    std::chrono::steady_clock::time_point last_progress_at;
    std::chrono::milliseconds timeout{30000};
  };
  mutable std::mutex event_mutex_;
  std::optional<PendingEvent> current_event_;
  std::optional<PendingCut> pending_cut_;
  bool event_fault_latched_{false};
  std::atomic<uint32_t> next_wire_request_id_;
  std::mutex status_mutex_;   // 保护 ready 状态
  bool ready_for_motion_{true};
  uint32_t ready_event_seq_{0};
  std::string ready_event_type_;

  //打印头状态
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
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr event_timeout_timer_;

public:
  UartNode()
  : Node("uart_node"), io_(), serial_(io_), running_(true),
    next_wire_request_id_(
      static_cast<uint32_t>(
        std::chrono::steady_clock::now().time_since_epoch().count()) | 1U)
  {
    //参数声明
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
    auto hb_qos = rclcpp::QoS(2000).reliable();
    auto event_qos = rclcpp::QoS(200).reliable();
    auto pub_qos = rclcpp::QoS(200).reliable();
    auto ready_qos = rclcpp::QoS(200).reliable().transient_local();
    hb_sub_ = create_subscription<RsiHeartBeat>(
      "/rsi/heartbeat",
      hb_qos,
      [this](RsiHeartBeat::SharedPtr msg) {
        on_heartbeat(*msg);
      }
    );
    //订阅事件
    event_sub_ = create_subscription<PlannedEvent>(
      "/rsi/triggered_event",
      event_qos,
      [this](PlannedEvent::SharedPtr msg) {
        on_triggered_event(*msg);
      }
    );
    //订阅系统命令
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "/system/command",
      rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        on_system_command(msg->data);
      }
    );
    //订阅UI手动控制命令
    manual_cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "/uart/manual_command",
      rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "收到手动控制命令: %s", msg->data.c_str());
        write_line(msg->data);
      }
    );
    //订阅测试模式RESET命令，清理可能遗留的事件等待状态
    print_test_cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "/print_test/rsi_command",
      rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        on_print_test_command(msg->data);
      }
    );
    //发布UART状态
    uart_raw_pub_ = create_publisher<std_msgs::msg::String>("/uart/raw", pub_qos);
    //发布ready状态
    ready_pub_ = create_publisher<PrintHeadStatus>("/printhead/status", ready_qos);

    //启动默认就绪(允许RSI从初始WAIT切入RUN)
    publish_ready_state("startup");

    open_serial();
    read_thread_ = std::thread(&UartNode::read_loop, this);
    event_timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {check_event_timeouts();});
  }

  ~UartNode() override
  {
    running_.store(false);
    boost::system::error_code ec;
    serial_.cancel(ec);
    serial_.close(ec);
    io_.stop();
    if (read_thread_.joinable()) {read_thread_.join();}
  }

private:
  uint32_t allocate_wire_request_id()
  {
    uint32_t value = next_wire_request_id_.fetch_add(1, std::memory_order_relaxed);
    if (value == 0U) {
      next_wire_request_id_.store(1U, std::memory_order_relaxed);
      value = 1U;
    }
    return value;
  }

  void mark_event_fault(
    const std::string & reason, uint32_t event_seq,
    const std::string & event_type)
  {
    {
      std::lock_guard<std::mutex> lk(event_mutex_);
      event_fault_latched_ = true;
      if (current_event_) {current_event_->failed = true;}
      if (pending_cut_) {pending_cut_->failed = true;}
    }
    set_ready_state(false, event_seq, event_type);
    publish_ready_state("event_fault:" + reason);
    RCLCPP_ERROR(
      get_logger(), "event safety barrier failed: type=%s seq=%u reason=%s",
      event_type.c_str(), event_seq, reason.c_str());
  }

  bool parse_integer_field(
    const std::vector<std::string> & tokens, const std::string & prefix,
    int64_t & value) const
  {
    for (const auto & token : tokens) {
      if (token.rfind(prefix, 0) != 0) {continue;}
      try {
        std::size_t used = 0;
        const long long parsed = std::stoll(token.substr(prefix.size()), &used);
        if (used != token.size() - prefix.size()) {return false;}
        value = static_cast<int64_t>(parsed);
        return true;
      } catch (...) {
        return false;
      }
    }
    return false;
  }

  bool parse_target_um(const std::vector<std::string> & tokens, int64_t & value) const
  {
    return parse_integer_field(tokens, "target_um=", value);
  }

  bool parse_cut_sync_contract(const std::string & payload, uint32_t & expected) const
  {
    expected = 0U;
    std::istringstream input(payload);
    std::string token;
    while (input >> token) {
      constexpr const char * PREFIX = "sync_v1:";
      const std::string prefix(PREFIX);
      if (token.rfind(prefix, 0) != 0) {continue;}
      try {
        std::size_t used = 0;
        const unsigned long parsed = std::stoul(token.substr(prefix.size()), &used);
        if (used != token.size() - prefix.size() || parsed != 2UL) {return false;}
        expected = static_cast<uint32_t>(parsed);
      } catch (...) {
        return false;
      }
    }
    return true;
  }

  void open_serial()
  {
    boost::system::error_code ec;
    serial_.open(port_, ec);
    if (ec) {
      RCLCPP_FATAL(get_logger(), "打开%s失败：%s", port_.c_str(), ec.message().c_str());
      throw std::runtime_error("串口打开失败");
    }
    //Boost.Asio串口配置
    serial_.set_option(serial_port_base::baud_rate(baudrate_));
    serial_.set_option(serial_port_base::character_size(8));     //数据长度
    serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));     //无奇偶校验
    serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));     //1个停止位
    serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));         //硬件流控关闭
  }

  void set_ready_state(bool ready_for_motion, uint32_t event_seq, const std::string & event_type)
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    ready_for_motion_ = ready_for_motion;
    ready_event_seq_ = event_seq;
    ready_event_type_ = event_type;
  }

  void publish_ready_state(const std::string & raw)  //
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
    const uint32_t wire_request_id = allocate_wire_request_id();
    std::string write_arg = ev.payload.empty() ? "0" : ev.payload;
    std::ostringstream oss;
    bool write_failed = false;
    std::string immediate_fault;

    {
      std::lock_guard<std::mutex> lk(event_mutex_);
      if (event_fault_latched_) {
        set_ready_state(false, ev.trigger_seq, ev.event_type);
        publish_ready_state("event_rejected:fault_latched");
        RCLCPP_ERROR(
          get_logger(), "拒绝事件 %s：上一完成屏障已锁存故障",
          ev.event_type.c_str());
        return;
      }

      if (ev.event_type == "cut") {
        if (current_event_ || pending_cut_) {
          immediate_fault = "cut_rejected_busy";
        } else {
          uint32_t expected_sync_count = 0U;
          if (!parse_cut_sync_contract(ev.payload, expected_sync_count)) {
            immediate_fault = "invalid_cut_sync_contract";
          } else {
            const auto now = std::chrono::steady_clock::now();
            pending_cut_ = PendingCut{
              ev.trigger_seq,
              wire_request_id,
              false,
              false,
              false,
              expected_sync_count,
              0U,
              now,
              now,
              std::chrono::milliseconds(30000)};
            oss << "EV " << wire_request_id << " cut " << write_arg << "\n";
          }
        }
      } else if (ev.event_type == "extrude_sync") {
        if (current_event_) {
          immediate_fault = "sync_rejected_busy";
        } else if (!pending_cut_ || pending_cut_->failed ||
          pending_cut_->expected_sync_count == 0U ||
          pending_cut_->sync_done_count >= pending_cut_->expected_sync_count)
        {
          immediate_fault = "missing_cut_owner";
        } else {
          std::istringstream payload(ev.payload);
          int tool_id = 0;
          double target_mm = 0.0;
          uint32_t timeout_ms = 0;
          std::string trailing;
          if (!(payload >> tool_id >> target_mm >> timeout_ms) ||
            (payload >> trailing) ||
            (tool_id != 1 && tool_id != 2) ||
            !std::isfinite(target_mm) ||
            timeout_ms == 0U || timeout_ms > 600000U)
          {
            immediate_fault = "invalid_sync_payload";
          } else {
            const double scaled_mm = target_mm * extrude_scale_.load();
            const double target_um_value = scaled_mm * 1000000.0;
            if (!std::isfinite(target_um_value) ||
              target_um_value < -1000000000000.0 ||
              target_um_value > 1000000000000.0)
            {
              immediate_fault = "sync_target_out_of_range";
            } else {
              const int64_t target_um =
                static_cast<int64_t>(std::llround(target_um_value));
              const auto now = std::chrono::steady_clock::now();
              pending_cut_->last_progress_at = now;
              current_event_ = PendingEvent{
                ev,
                wire_request_id,
                false,
                false,
                false,
                now,
                std::chrono::milliseconds(timeout_ms),
                target_um,
                tool_id,
                pending_cut_->wire_request_id};
              oss << "EV " << wire_request_id << " extrude_sync "
                  << tool_id << " " << target_um << " " << timeout_ms << " "
                  << pending_cut_->wire_request_id << "\n";
            }
          }
        }
      } else {
        if (current_event_) {
          immediate_fault = "event_rejected_busy";
        } else {
          current_event_ = PendingEvent{
            ev,
            wire_request_id,
            false,
            false,
            false,
            std::chrono::steady_clock::now(),
            std::chrono::milliseconds(60000),
            std::nullopt,
            std::nullopt,
            std::nullopt};
          oss << "EV " << wire_request_id << " " << ev.event_type << " "
              << write_arg << "\n";
        }
      }
    }

    if (!immediate_fault.empty()) {
      mark_event_fault(immediate_fault, ev.trigger_seq, ev.event_type);
      return;
    }

    if (ev.event_type == "heat_cf" || ev.event_type == "heat_resin") {
      try {
        const float target = std::stof(ev.payload);
        std::lock_guard<std::mutex> lk(status_cache_mutex_);
        if (ev.event_type == "heat_cf") {
          status_.target_temp_cf = target;
        } else {
          status_.target_temp_resin = target;
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_logger(), "温控参数解析失败：%s", e.what());
      }
    }

    write_failed = !write_line(oss.str());
    std::string ev_info = "event:" + ev.event_type;
    if (!ev.payload.empty()) {ev_info += " payload=" + ev.payload;}

    if (ev.event_type == "cut") {
      if (write_failed) {
        mark_event_fault("cut_write_failed", ev.trigger_seq, ev.event_type);
        return;
      }
      set_ready_state(true, ev.trigger_seq, ev.event_type);
      publish_ready_state(ev_info + " nonblocking; completion tracked");
      return;
    }

    if (write_failed) {
      mark_event_fault("event_write_failed", ev.trigger_seq, ev.event_type);
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
  }

  void on_heartbeat(const RsiHeartBeat & hb) //心跳触发挤出
  {
    // 与事件发送共用 event_mutex_，防止屏障前检查通过后又插入一条迟到 E。
    std::lock_guard<std::mutex> event_lk(event_mutex_);
    if (paused_.load() || aborted_.load() || event_fault_latched_ || current_event_) {
      return;
    }
    const double scale = extrude_scale_.load();
    const double scaled = hb.extrude_abs * scale;
    std::lock_guard<std::mutex> lk(extrude_forward_mutex_);
    const auto prepared = extrusion_forwarder_->prepare(hb.seq_used, hb.tool_id, scaled);
    if (
      prepared.decision == uart_bridge::ExtrusionDecision::RejectNonFinite ||
      prepared.decision == uart_bridge::ExtrusionDecision::RejectOutOfRange)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "拒绝挤出值：seq=%u tool=%d value=%.17g",
        hb.seq_used,
        hb.tool_id,
        scaled);
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

  void read_loop()  //阻塞读取MCU状态包
  {
    std::array<char, 256> buf{};
    while (rclcpp::ok() && running_.load()) {
      boost::system::error_code ec;
      std::size_t n = serial_.read_some(boost::asio::buffer(buf), ec);
      if (ec) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "读取错误：%s",
          ec.message().c_str());
        continue;
      }
      recv_buf_.append(buf.data(), n);
      while (true) {
        auto pos = recv_buf_.find('\n');
        if (pos == std::string::npos) {break;}
        std::string line = recv_buf_.substr(0, pos);
        recv_buf_.erase(0, pos + 1);
        if (!line.empty() && line.back() == '\r') {line.pop_back();}
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
    if (line.rfind("EVERR ", 0) == 0) {
      handle_event_failure(line);
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
    std::vector<std::string> tokens;
    std::string token;
    while (ss >> token) {tokens.push_back(token);}
    if (tokens.size() < 4 || tokens[0] != (done_response ? "EVDONE" : "EVACK")) {
      mark_event_fault("malformed_response", 0, "uart");
      return;
    }

    uint32_t event_seq = 0;
    try {
      std::size_t used = 0;
      const unsigned long parsed = std::stoul(tokens[1], &used);
      if (used != tokens[1].size() || parsed > std::numeric_limits<uint32_t>::max()) {
        throw std::out_of_range("event sequence");
      }
      event_seq = static_cast<uint32_t>(parsed);
    } catch (...) {
      mark_event_fault("invalid_response_seq", 0, "uart");
      return;
    }
    const std::string & event_type = tokens[2];
    const std::string & status = tokens[3];
    bool matched = false;
    bool identity_rejected = false;
    bool response_rejected = false;

    {
      std::lock_guard<std::mutex> lk(event_mutex_);
      if (pending_cut_ && pending_cut_->wire_request_id == event_seq &&
        event_type == "cut")
      {
        matched = true;
        if (done_response) {
          if (status == "done") {
            pending_cut_->done_received = true;
            pending_cut_->last_progress_at = std::chrono::steady_clock::now();
          } else {
            pending_cut_->failed = true;
            event_fault_latched_ = true;
            response_rejected = true;
          }
        } else if (status == "accepted") {
          pending_cut_->ack_received = true;
        } else {
          pending_cut_->failed = true;
          event_fault_latched_ = true;
          response_rejected = true;
        }
      }

      if (current_event_ && current_event_->wire_request_id == event_seq &&
        current_event_->planned.event_type == event_type)
      {
        matched = true;
        if (done_response) {
          if (status != "done") {
            current_event_->failed = true;
            event_fault_latched_ = true;
            response_rejected = true;
          } else if (event_type == "extrude_sync") {
            int64_t reported_target = 0;
            int64_t reported_tool = 0;
            int64_t reported_cut_request = 0;
            if (!parse_target_um(tokens, reported_target) ||
              !parse_integer_field(tokens, "tool=", reported_tool) ||
              !parse_integer_field(tokens, "cut_request_id=", reported_cut_request) ||
              !current_event_->expected_target_um ||
              !current_event_->expected_tool ||
              !current_event_->cut_request_id ||
              reported_target != *current_event_->expected_target_um ||
              reported_tool != *current_event_->expected_tool ||
              reported_cut_request != static_cast<int64_t>(*current_event_->cut_request_id))
            {
              current_event_->failed = true;
              event_fault_latched_ = true;
              identity_rejected = true;
            } else {
              current_event_->done_received = true;
            }
          } else {
            current_event_->done_received = true;
          }
        } else if (status == "accepted") {
          current_event_->ack_received = true;
        } else {
          current_event_->failed = true;
          event_fault_latched_ = true;
          response_rejected = true;
        }
      }
    }

    if (response_rejected) {
      mark_event_fault("invalid_response_status", event_seq, event_type);
      return;
    }
    if (!matched) {
      mark_event_fault("unexpected_response", event_seq, event_type);
      return;
    }
    if (identity_rejected) {
      mark_event_fault("sync_identity_mismatch", event_seq, event_type);
      return;
    }
    publish_ready_state(line);
    check_event_completion();
  }

  void handle_event_failure(const std::string & line)
  {
    std::istringstream ss(line);
    std::string tag;
    uint32_t event_seq{0};
    std::string event_type;
    std::string reason;
    if (!(ss >> tag >> event_seq >> event_type >> reason)) {
      mark_event_fault("malformed_error", 0, "uart");
      return;
    }
    mark_event_fault(reason, event_seq, event_type);
  }

  void check_event_timeouts()
  {
    const auto now = std::chrono::steady_clock::now();
    std::string reason;
    uint32_t seq = 0;
    std::string type;
    {
      std::lock_guard<std::mutex> lk(event_mutex_);
      const bool completed_sync_waiting_for_cut =
        current_event_ &&
        current_event_->planned.event_type == "extrude_sync" &&
        current_event_->ack_received && current_event_->done_received;
      if (current_event_ && !current_event_->failed &&
        !completed_sync_waiting_for_cut &&
        now - current_event_->sent_at > current_event_->timeout)
      {
        current_event_->failed = true;
        event_fault_latched_ = true;
        seq = current_event_->planned.trigger_seq;
        type = current_event_->planned.event_type;
        reason = "event_timeout";
      }
      if (reason.empty() && pending_cut_ && !pending_cut_->failed) {
        if (!pending_cut_->done_received &&
          now - pending_cut_->sent_at > pending_cut_->timeout)
        {
          pending_cut_->failed = true;
          event_fault_latched_ = true;
          seq = pending_cut_->trigger_seq;
          type = "cut";
          reason = "cut_timeout";
        } else if (pending_cut_->done_received &&
          pending_cut_->sync_done_count < pending_cut_->expected_sync_count &&
          now - pending_cut_->last_progress_at > std::chrono::milliseconds(60000))
        {
          pending_cut_->failed = true;
          event_fault_latched_ = true;
          seq = pending_cut_->trigger_seq;
          type = "cut";
          reason = "cut_sync_incomplete";
        }
      }
    }
    if (!reason.empty()) {
      set_ready_state(false, seq, type);
      publish_ready_state("event_fault:" + reason);
      RCLCPP_ERROR(
        get_logger(), "event safety barrier timeout: type=%s seq=%u",
        type.c_str(), seq);
    }
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
          get_logger(),
          *get_clock(),
          2000,
          "字段解析失败：%s",
          kv.c_str());
      }
    }
  }


  void check_event_completion()   //判断事件完成
  {
    constexpr float TEMP_EPS = 20.0f;
    bool ready_done = false;
    bool reset_forwarder = false;
    uint32_t done_seq = 0;
    std::string done_type;
    std::string completion_fault;

    {
      std::lock_guard<std::mutex> lk(event_mutex_);
      if (event_fault_latched_) {return;}

      if (!current_event_) {
        if (pending_cut_ && pending_cut_->expected_sync_count == 0U &&
          pending_cut_->ack_received && pending_cut_->done_received &&
          !pending_cut_->failed)
        {
          done_seq = pending_cut_->trigger_seq;
          done_type = "cut";
          pending_cut_.reset();
          ready_done = true;
        }
      } else if (!current_event_->failed) {
        auto & ev = *current_event_;
        bool done = false;
        Status snapshot;
        {
          std::lock_guard<std::mutex> sk(status_cache_mutex_);
          snapshot = status_;
        }

        if (ev.planned.event_type == "extrude_sync") {
          if (!pending_cut_ || pending_cut_->failed ||
            pending_cut_->expected_sync_count == 0U ||
            pending_cut_->sync_done_count >= pending_cut_->expected_sync_count)
          {
            ev.failed = true;
            event_fault_latched_ = true;
            completion_fault = "cut_owner_invalid";
          } else {
            // Every sync behind a CUT is a hard barrier.  The CUT is fired
            // while RSI is parked at the safe height and the motion that
            // resumes after the first sync is the safety retract, so the
            // retract may only start once the firmware has reported both
            // cutter strokes finished.  Gating on the last sync alone would
            // release the retract while the blade is still cutting.
            done = ev.ack_received && ev.done_received &&
              pending_cut_->ack_received && pending_cut_->done_received;
          }
        } else if (ev.planned.event_type == "extrude_reset") {
          done = ev.ack_received && ev.done_received;
        } else if (ev.planned.event_type == "heat_cf") {
          done = std::abs(snapshot.current_temp_cf - snapshot.target_temp_cf) <= TEMP_EPS;
        } else if (ev.planned.event_type == "heat_resin") {
          done = std::abs(snapshot.current_temp_resin - snapshot.target_temp_resin) <= TEMP_EPS;
        } else if (
          ev.planned.event_type == "tool_change_cf" ||
          ev.planned.event_type == "tool_change_resin")
        {
          try {
            const int target_tool = std::stoi(ev.planned.payload);
            done = snapshot.current_tool == target_tool;
          } catch (const std::exception &) {
            done = false;
          }
        } else if (ev.planned.event_type == "fan_cf") {
          try {
            const bool target = std::stoi(ev.planned.payload) != 0;
            done = snapshot.fan_ok_cf == target;
          } catch (...) {
            done = false;
          }
        } else if (ev.planned.event_type == "fan_resin") {
          try {
            const bool target = std::stoi(ev.planned.payload) != 0;
            done = snapshot.fan_ok_resin == target;
          } catch (...) {
            done = false;
          }
        } else {
          done = ev.ack_received && ev.done_received;
        }

        if (done) {
          done_seq = ev.planned.trigger_seq;
          done_type = ev.planned.event_type;
          ready_done = true;
          if (ev.planned.event_type == "extrude_sync") {
            ++pending_cut_->sync_done_count;
            pending_cut_->last_progress_at = std::chrono::steady_clock::now();
            if (pending_cut_->done_received &&
              pending_cut_->sync_done_count >= pending_cut_->expected_sync_count)
            {
              pending_cut_.reset();
            }
          }
          if (ev.planned.event_type == "extrude_reset") {
            reset_forwarder = true;
          }
          current_event_.reset();
        }
      }
    }

    if (!completion_fault.empty()) {
      mark_event_fault(completion_fault, done_seq, done_type);
      return;
    }
    if (ready_done) {
      set_ready_state(true, done_seq, done_type);
      publish_ready_state("event_done");
      if (reset_forwarder) {
        reset_extrude_forward_state();
      }
    }
  }

  bool write_line(const std::string & line)
  {
    publish_uart_log("TX", line);
    std::lock_guard<std::mutex> lk(serial_write_mutex_);
    boost::system::error_code ec;
    const std::size_t written = boost::asio::write(serial_, boost::asio::buffer(line), ec);
    if (ec || written != line.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "写入失败：%s (written=%zu expected=%zu)",
        ec ? ec.message().c_str() : "incomplete write",
        written,
        line.size());
      return false;
    }
    return true;
  }

  void on_print_test_command(const std::string & cmd)
  {
    if (cmd == "RESET") {
      {
        std::lock_guard<std::mutex> lk(event_mutex_);
        current_event_.reset();
        pending_cut_.reset();
        event_fault_latched_ = false;
      }
      reset_extrude_forward_state();
      RCLCPP_INFO(get_logger(), "测试模式RESET：清理UART事件屏障与转发状态");
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
      {
        std::lock_guard<std::mutex> lk(event_mutex_);
        current_event_.reset();
        pending_cut_.reset();
        event_fault_latched_ = true;
      }
      set_ready_state(false, 0, "abort");
      publish_ready_state("aborted");
      // 通过现有EV串口指令将两个喷头加热温度设为0°C
      write_line("EV 0 heat_cf 0\n");
      write_line("EV 0 heat_resin 0\n");
      // 更新本地温度缓存
      {
        std::lock_guard<std::mutex> lk(status_cache_mutex_);
        status_.target_temp_cf = 0.0f;
        status_.target_temp_resin = 0.0f;
      }
      RCLCPP_WARN(get_logger(), "uart_node收到ABORT命令，停止挤出并关闭加热");
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
