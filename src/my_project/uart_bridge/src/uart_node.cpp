// =============================================================
// 描述：
// 单片机通信节点，UART串口协议
// 订阅RSI心跳（包含挤出）
// 发布接收到的打印头各模块状态消息，发布READY消息
// =============================================================

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <std_msgs/msg/string.hpp>

#include <my_project_interfaces/msg/planned_event.hpp>
#include <my_project_interfaces/msg/rsi_heart_beat.hpp>
#include <my_project_interfaces/msg/print_head_status.hpp>

#include <boost/asio.hpp>//串口
#include <array>
#include <thread>
#include <atomic>
#include <optional>
#include <mutex>
#include <sstream>
#include <cmath>
#include <limits>

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

  // 缓存
  mutable std::mutex event_mutex_;
  std::optional<PlannedEvent> current_event_;
  bool current_event_ack_received_{false};
  bool current_event_done_received_{false};
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
  std::atomic<double> extrude_scale_{1.0};
  std::atomic<bool> paused_{false};
  std::atomic<bool> aborted_{false};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

public:
  UartNode()
  : Node("uart_node"), io_(), serial_(io_), running_(true)
  {
    //参数声明
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    extrude_scale_.store(declare_parameter<double>("extrude_scale", 1.0));
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
    {
      std::lock_guard<std::mutex> lk(event_mutex_);
      current_event_ = ev;
      current_event_ack_received_ = false;
      current_event_done_received_ = false;
    }

    // heat 事件：payload 为目标温度(℃)，写入缓存，供温差判定
    if (ev.event_type == "heat_cf" || ev.event_type == "heat_resin") {
      try {
        float target = std::stof(ev.payload);         // 摄氏度
        std::lock_guard<std::mutex> lk(status_cache_mutex_);
        if (ev.event_type == "heat_cf") {
          status_.target_temp_cf = target;
        } else {
          status_.target_temp_resin = target;
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "温控参数解析失败：%s",
          e.what());
      }
    }

    // 发送事件指令 EV <trigger_seq> <event_type> <arg>
    // current_event_ 已经设置，心跳线程会暂停 E 转发，避免 reset 未完成时刷 old_seq。
    std::string arg = ev.payload.empty() ? "0" : ev.payload;
    std::ostringstream oss;
    oss << "EV " << ev.trigger_seq << " " << ev.event_type << " " << arg << "\n";
    write_line(oss.str());

    // 收到事件后清除ready，避免上一事件ready粘住导致RSI误退出WAIT。
    set_ready_state(false, ev.trigger_seq, ev.event_type);
    std::string ev_info = "event:" + ev.event_type;
    if (!ev.payload.empty()) {
      ev_info += " payload=" + ev.payload;
    }
    publish_ready_state(ev_info);
    // 事件完成由 MCU 上报 STAT 后 check_event_completion 判定
  }

  bool has_pending_event() const
  {
    std::lock_guard<std::mutex> lk(event_mutex_);
    return current_event_.has_value();
  }

  void on_heartbeat(const RsiHeartBeat & hb) //心跳触发挤出
  {
    // PAUSE、ABORT 或事件等待时不转发挤出命令。
    // extrude_reset 未完成前继续发 E 会被固件 old_seq 拒绝，并污染时序。
    if (paused_.load() || aborted_.load() || has_pending_event()) {
      return;
    }
    const double scale = extrude_scale_.load();
    const double scaled = hb.extrude_abs * scale;
    send_extrude_command(hb.seq_used, hb.tool_id, static_cast<float>(scaled));
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
      if (current_event_ &&
        current_event_->trigger_seq == event_seq &&
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
        done = true;         // Fallback
      }
    } else if (ev.event_type == "fan_resin") {
      try {
        bool target = std::stoi(ev.payload) != 0;
        done = (snapshot.fan_ok_resin == target);
      } catch (...) {
        RCLCPP_WARN(get_logger(), "fan_resin事件payload格式错误: '%s'，跳过等待", ev.payload.c_str());
        done = true;         // Fallback
      }
    } else if (ev.event_type == "extrude_reset") {
      const bool stat_cleared =
        snapshot.last_e_seq == -1 &&
        std::abs(snapshot.last_e_abs) < 1e-9 &&
        snapshot.last_e_us == 0;
      done = current_event_ack_received_ && current_event_done_received_ && stat_cleared;
    }

    if (done) {
      set_ready_state(true, ev.trigger_seq, ev.event_type);
      publish_ready_state("event_done");
      current_event_.reset();
    }
  }

  void send_extrude_command(uint32_t seq_used, int tool_id, float extrude_abs)
  {
    std::ostringstream oss;
    oss << "E " << seq_used << " " << tool_id << " " << extrude_abs << "\n";     //  mm
    write_line(oss.str());
  }

  void write_line(const std::string & line)
  {
    publish_uart_log("TX", line);
    std::lock_guard<std::mutex> lk(serial_write_mutex_);
    boost::system::error_code ec;
    boost::asio::write(serial_, boost::asio::buffer(line), ec);
    if (ec) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "写入失败：%s",
        ec.message().c_str());
    }
  }

  void on_print_test_command(const std::string & cmd)
  {
    if (cmd == "RESET") {
      {
        std::lock_guard<std::mutex> lk(event_mutex_);
        current_event_ack_received_ = false;
        current_event_done_received_ = false;
      }
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
