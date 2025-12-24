// =============================================================
// 描述：
// RSI通信节点，以太网UDP协议
// 订阅中心节点发布的轨迹与事件消息，订阅打印头状态消息
// 发布KUKA端通过RSI发来的xml包中的消息
// 包含状态机等待逻辑
// =============================================================

#include <rclcpp/rclcpp.hpp>
//自定义消息头
#include <my_project_interfaces/msg/trajectory_point.hpp>
#include <my_project_interfaces/msg/planned_event.hpp>
#include <my_project_interfaces/msg/print_head_status.hpp>
#include <my_project_interfaces/msg/rsi_heart_beat.hpp>
//标准消息头
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
//socket相关头
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
//并发/容器
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <optional>
#include <deque>


//自定义消息接口
using my_project_interfaces::msg::TrajectoryPoint; //TCP轨迹点
using my_project_interfaces::msg::PlannedEvent; //打印事件
using my_project_interfaces::msg::PrintHeadStatus; //打印头状态
using my_project_interfaces::msg::RsiHeartBeat; //RSI心跳


class RSINode : public rclcpp::Node
{
private:
  enum class State {RUN, WAIT}; //状态机两种状态

  //订阅 
  rclcpp::Subscription<TrajectoryPoint>::SharedPtr traj_sub_; //轨迹点
  rclcpp::Subscription<PlannedEvent>::SharedPtr event_sub_; //事件
  rclcpp::Subscription<PrintHeadStatus>::SharedPtr status_sub_; //打印头状态
  //发布 
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr kuka_pub_;  //收到kuka xml
  rclcpp::Publisher<RsiHeartBeat>::SharedPtr heartbeat_pub_;  //RSI心跳
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr resync_pub_; //请求中心节点同步
  rclcpp::Publisher<PlannedEvent>::SharedPtr triggered_event_pub_; //转发给UART的事件

  //数据缓存
  //互斥锁
  std::mutex traj_mutex_;
  std::mutex event_mutex_;
  //队列
  std::deque<TrajectoryPoint> traj_queue_;
  std::deque<PlannedEvent> event_queue_;
  //用于等待时发送的上一帧轨迹点
  std::optional<TrajectoryPoint> last_sent_;
  //UART状态(用于WAIT解除绑定)
  struct ReadyAck
  {
    bool ready_for_motion{false};
    uint32_t ready_event_seq{0};
    std::string ready_event_type;
  };
  std::mutex ready_mutex_;
  ReadyAck ready_ack_;

  uint32_t next_seq_{0};
  bool resync_sent_{false};
  rclcpp::Time last_resync_time_;

  //状态机
  State state_{State::WAIT}; //默认状态
  std::optional<PlannedEvent> current_wait_;//当前事件

  //UDP
  std::string sen_type_;
  int decimal_precision_;
  std::string local_ip_;
  uint16_t local_port_;
  int sockfd_{-1}; //socket文件描述符
  std::atomic<bool> run_udp_{false}; //控制循环的原子开关
  std::thread udp_thread_;
  


public:
  RSINode(): Node("rsi_node")
  { 
    sen_type_ = declare_parameter<std::string>("sen_type", "PosCorr");  //与kuka的xml文件中规定一致
    decimal_precision_ = declare_parameter<int>("decimal_precision", 6);  //收发数据小数点位数，需在rsi上下文同步修改
    local_ip_ = declare_parameter<std::string>("local_ip", "192.168.1.1");  //ip
    local_port_ = declare_parameter<int>("local_port", 49152);  //端口

    last_sent_ = TrajectoryPoint();//全0，不进入队列，不影响对齐

    auto traj_qos = rclcpp::QoS(2000).reliable();//轨迹队列长度2000 = 收到消息的最多缓存条数 reliable协议保证丢包时顺序
    auto event_qos = rclcpp::QoS(200).reliable();//事件队列长度200 = 收到消息的最多缓存条数 reliable协议保证丢包时顺序

    //订阅中心节点轨迹消息
    traj_sub_ = create_subscription<TrajectoryPoint>(
      "/planned_trajectory", //话题名
      traj_qos, 
      [this](TrajectoryPoint::SharedPtr msg) 
      {
        std::lock_guard<std::mutex> lock(traj_mutex_); //加锁保护共享数据，防止UDP线程同时读取
        traj_queue_.push_back(*msg); //消息内容拷贝入队尾
      } //lamda回调函数
    );

    //订阅中心节点事件消息
    event_sub_ = create_subscription<PlannedEvent>(
      "/planned_events", 
      event_qos,
      [this](PlannedEvent::SharedPtr msg) 
      {
        std::lock_guard<std::mutex> lk(event_mutex_);
        event_queue_.push_back(*msg);
      }
    );

    //订阅UART状态
    status_sub_ = create_subscription<PrintHeadStatus>(
      "/printhead/status",
      traj_qos,
      [this](PrintHeadStatus::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lk(ready_mutex_);
        ready_ack_.ready_for_motion = msg->ready_for_motion;
        ready_ack_.ready_event_seq = msg->ready_event_seq;
        ready_ack_.ready_event_type = msg->ready_event_type;
      }
    );

    //发布kuka原始消息
    kuka_pub_ = create_publisher<std_msgs::msg::String>("/kuka/raw_xml", 10);

    //发布RSI心跳
    heartbeat_pub_ = create_publisher<RsiHeartBeat>("/rsi/heartbeat", 10);

    //发布同步请求标志
    resync_pub_ = create_publisher<std_msgs::msg::UInt32>("/rsi/resync_request", 10);

    //发布触发事件给UART
    triggered_event_pub_ = create_publisher<PlannedEvent>("/rsi/triggered_event", 10);

    //打开UDP
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0); //ipv4 udp套接字
    if (sockfd_ < 0 ){
      RCLCPP_FATAL(get_logger(), "创建socket失败。");
      throw std::runtime_error("socket");
    }
    sockaddr_in local{}; 
    local.sin_family = AF_INET; 
    local.sin_port = htons(local_port_);
    inet_pton(AF_INET, local_ip_.c_str(), &local.sin_addr); //字符串IP转为二进制
    if(bind(sockfd_, reinterpret_cast<sockaddr*>(&local),sizeof(local)) < 0 ) //绑定到本地IP+端口开始监听
    {
      RCLCPP_FATAL(get_logger(), " 绑定失败 %s:%d", local_ip_.c_str(), local_port_);
      throw std::runtime_error("bind");
    }

    //启动UDP循环 独立线程
    run_udp_.store(true);
    udp_thread_ = std::thread(&RSINode::udp_loop, this);
  }

~RSINode() override
{
  run_udp_.store(false);
  if (sockfd_ >= 0) close(sockfd_);
  if (udp_thread_.joinable()) udp_thread_.join();
}

private:
  void udp_loop()//核心循环，节拍 = kuka包
  {
    constexpr size_t BUF_SIZE = 4096;
    char buf[BUF_SIZE];
    sockaddr_in remote{};
    socklen_t remote_len = sizeof(remote);

    while (rclcpp::ok() && run_udp_.load()) //循环条件：ROS运行且run_udp_ = true
    {
      // 阻塞接收UDP
      ssize_t n = recvfrom(sockfd_, buf, BUF_SIZE - 1, 0, reinterpret_cast<sockaddr*>(&remote), &remote_len);

      if (n <= 0) continue;
      buf[n] = '\0';
      std::string recv_str(buf); //数据转字符串

      //发布原始xml 字符串消息
      std_msgs::msg::String raw_msg;
      raw_msg.data = recv_str;
      kuka_pub_ -> publish(raw_msg);     

      //解IPOC
      std::string ipoc = extract_ipoc(recv_str);
      
      //********默认发送上一帧********
      TrajectoryPoint to_send = *last_sent_;

      //状态机 严格对齐seq
      if (state_ == State::RUN)
      {
        if (should_enter_wait(next_seq_)) //事件触发了等待:判断trigger_seq是否到达当前traj序号
        {
          current_wait_ = pop_next_event();
          if (current_wait_)
          {
            triggered_event_pub_->publish(*current_wait_);
            state_ = State::WAIT;
          }
          //此时保持last_sent_
        }
        else
        {
          while (auto tp = pop_next_traj())
          {
            if(tp->seq < next_seq_) //队头比期望慢
            {
              continue; // 旧数据直接丢弃，保持last_sent_
            }
            if(tp->seq == next_seq_)//正常对齐情况
            {
              to_send = *tp;
              last_sent_ = to_send;
              ++next_seq_;
              resync_sent_ = false;
            }
            else //队头比期望快
            {
              RCLCPP_WARN(get_logger(), "RSI节点收发顺序错误： 期待 %u, 实际 %u, 正在重发上一帧", next_seq_, tp->seq);
              // 清理队列缓存准备重新同步
              {
                std::lock_guard<std::mutex> lk(traj_mutex_);
                traj_queue_.clear();
              }
              //触发 resync 请求
              auto nowt = now();
              if(!resync_sent_ || (nowt - last_resync_time_).seconds() > 0.1) //限流
              {
                std_msgs::msg::UInt32 req;
                req.data = next_seq_; //从next_seq_开始重播
                resync_pub_ -> publish(req);
                resync_sent_ = true;
                last_resync_time_ = nowt;
              }
            }
            break; //每个心跳只处理一条
          }
          //队列空则继续last_sent_
        }
      }
      else //State::WAIT
      {
        if(is_wait_cleared()) // UART当前事件就绪后解除等待
        {
          state_ = State::RUN;
          current_wait_.reset();
        }
        // WAIT期间始终重发 last_sent_
      }
      
      //发布RSI心跳包  携带本周期实际使用的seq 供UART对齐 
      RsiHeartBeat hb;
      hb.stamp = now();
      hb.ipoc = ipoc;
      hb.seq_used = to_send.seq;
      hb.tool_id = to_send.tool_id;
      hb.extrude_abs = to_send.e;
      heartbeat_pub_ -> publish(hb);

      //回复XML
      std::string reply = build_reply(ipoc, to_send);
      sendto(sockfd_, reply.c_str(), reply.size(), 0, reinterpret_cast<sockaddr*>(&remote), remote_len);

    }
  }

  std::optional<TrajectoryPoint> pop_next_traj() 
  {
    std::lock_guard<std::mutex> lk(traj_mutex_);
    if (traj_queue_.empty()) return std::nullopt;
    auto tp = traj_queue_.front();
    traj_queue_.pop_front();
    return tp;
  }

  std::optional<PlannedEvent> pop_next_event() 
  {
    std::lock_guard<std::mutex> lk(event_mutex_);
    if (event_queue_.empty()) return std::nullopt;
    auto ev = event_queue_.front();
    event_queue_.pop_front();
    return ev;
  }  

  bool is_wait_cleared()
  {
    std::lock_guard<std::mutex> lk(ready_mutex_);
    if (!ready_ack_.ready_for_motion) return false;
    if (!current_wait_) return true; // 初始WAIT：仅需全局ready
    if (ready_ack_.ready_event_seq != current_wait_->trigger_seq) return false;
    return ready_ack_.ready_event_type == current_wait_->event_type;
  }

  bool should_enter_wait(uint32_t next_seq)//进入事件判断
  {
    std::lock_guard<std::mutex> lk(event_mutex_);
    if (event_queue_.empty()) return false;
    //如果事件队头的trigger_seq已经到达或者超过当前轨迹序号，就进入等待
    return event_queue_.front().trigger_seq <= next_seq;
  }

  std::string extract_ipoc(const std::string &xml)//提取时间戳
  {
    const std::string tag_open = "<IPOC>";
    const std::string tag_close = "</IPOC>";
    auto s = xml.find(tag_open);
    if (s == std::string::npos) return "0";
    s += tag_open.size();
    auto e = xml.find(tag_close, s);
    if (e == std::string::npos) return "0";
    return xml.substr(s, e - s);
  }

  std::string build_reply(const std::string &ipoc, const TrajectoryPoint &tp) 
  {
    char xml[512];

    snprintf(xml, sizeof(xml),
      "<Sen Type=\"%s\">\r\n"
      "<PosCorr X=\"%.*f\" Y=\"%.*f\" Z=\"%.*f\" A=\"%.*f\" B=\"%.*f\" C=\"%.*f\"/>\r\n"
      "<Stop>0</Stop>\r\n"
      "<IPOC>%s</IPOC>\r\n"
      "</Sen>",
      sen_type_.c_str(),
      decimal_precision_, tp.x,
      decimal_precision_, tp.y,
      decimal_precision_, tp.z,
      decimal_precision_, tp.a,
      decimal_precision_, tp.b,
      decimal_precision_, tp.c,
      ipoc.c_str());
    return std::string(xml);
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RSINode>());
  rclcpp::shutdown();
  return 0;
}
