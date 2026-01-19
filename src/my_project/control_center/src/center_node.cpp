// =============================================================
// 描述：
// 中心节点
// 订阅RSI节点kuka状态消息，订阅打印头状态消息
// 发布gcode_planner处理好的轨迹+事件消息
// 包含实现同步校准
// =============================================================

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <my_project_interfaces/msg/kuka_status.hpp>
#include <my_project_interfaces/msg/planned_event.hpp>
#include <my_project_interfaces/msg/print_head_status.hpp>
#include <my_project_interfaces/msg/rsi_heart_beat.hpp>
#include <my_project_interfaces/msg/trajectory_point.hpp>

#include <chrono>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <memory>
#include <cmath>
#include <vector>

#include "control_center/npz_loader.hpp"
#include "control_center/queue_manager.hpp"

using std_msgs::msg::String;
using my_project_interfaces::msg::TrajectoryPoint; //TCP轨迹点
using my_project_interfaces::msg::PlannedEvent; //打印事件
using my_project_interfaces::msg::PrintHeadStatus; //打印头状态
using my_project_interfaces::msg::RsiHeartBeat; //RSI心跳
using my_project_interfaces::msg::KukaStatus; //kuka状态

class CenterNode : public rclcpp :: Node
{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kuka_raw_sub_;
    rclcpp::Subscription<KukaStatus>::SharedPtr kuka_status_sub_;
    rclcpp::Subscription<RsiHeartBeat>::SharedPtr hb_sub_;
    rclcpp::Subscription<PrintHeadStatus>::SharedPtr printhead_status_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr resync_sub_;

    rclcpp::Publisher<TrajectoryPoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<PlannedEvent>::SharedPtr event_pub_;
    std::mutex cache_mutex_;
    std::optional<PrintHeadStatus> last_printhead_status_;

    bool kuka_status_raw_{false};
    int plan_qos_depth_{2000};
    int traj_prefill_{1000};
    int traj_low_{500};
    int traj_high_{1500};
    std::optional<uint32_t> last_published_traj_seq_;
    std::optional<uint32_t> last_seq_used_;

    std::unique_ptr<control_center::NpzLoader> npz_loader_;
    std::unique_ptr<control_center::QueueManager> queue_manager_;
    std::mutex queue_mutex_;
    int xyzabc_decimals_{6};
    int e_decimals_{2};

public:
    CenterNode(): Node("center_node")
    {
        auto npz_path = declare_parameter<std::string>(
            "npz_path",
            "/home/jayson/kuka_ram_ws/data/output_npz/test.npz");
        auto npz_preload = declare_parameter<int>("npz_preload_chunks", 2);
        auto queue_low = declare_parameter<int>("queue_low", 1000);
        auto queue_high = declare_parameter<int>("queue_high", 2000);
        plan_qos_depth_ = declare_parameter<int>("plan_qos_depth", 2000);
        traj_prefill_ = declare_parameter<int>("traj_prefill", 1000);
        traj_low_ = declare_parameter<int>("traj_low", 500);
        traj_high_ = declare_parameter<int>("traj_high", 1500);
        xyzabc_decimals_ = declare_parameter<int>("xyzabc_decimals", 6);
        e_decimals_ = declare_parameter<int>("e_decimals", 2);

        npz_loader_ = std::make_unique<control_center::NpzLoader>(
            npz_path, static_cast<size_t>(npz_preload));
        if (!npz_loader_->ok()) {
            RCLCPP_ERROR(get_logger(), "NPZ加载器初始化失败：%s", npz_loader_->error().c_str());
        } else {
            RCLCPP_INFO(get_logger(), "NPZ加载器就绪：%s (预加载=%ld)",
                        npz_path.c_str(), static_cast<long>(npz_preload));
        }
        queue_manager_ = std::make_unique<control_center::QueueManager>(
            static_cast<size_t>(queue_low), static_cast<size_t>(queue_high));
        if (npz_loader_->ok()) {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            queue_manager_->fill(*npz_loader_);
        }

        kuka_status_raw_ = declare_parameter<bool>("kuka_status_raw", false);
        auto monitor_qos = rclcpp::QoS(10);
        auto plan_qos = rclcpp::QoS(plan_qos_depth_).reliable();

        //订阅RSI原始KUKA XML
        kuka_raw_sub_ = create_subscription<String>(
            "/kuka/raw_xml",
            monitor_qos,
            [this](std_msgs::msg::String::SharedPtr msg){
                if (kuka_status_raw_){
                    RCLCPP_DEBUG(get_logger(), "KUKA端原始XML长度=%zu", msg->data.size());
                }
            }
        );

        //订阅KUKA状态（保留话题连接，后续如需再用）
        kuka_status_sub_ = create_subscription<KukaStatus>(
            "kuka/status",
            monitor_qos,
            [](KukaStatus::SharedPtr) {}
        );

        //订阅RSI心跳包
        hb_sub_ = create_subscription<RsiHeartBeat>(
            "/rsi/heartbeat",
            monitor_qos,
            [this](RsiHeartBeat::SharedPtr msg){
                std::lock_guard<std::mutex> lk(cache_mutex_);
                last_seq_used_ = msg->seq_used;
                publish_from_queue();
            }
        );

        //订阅打印头状态
        printhead_status_sub_ = create_subscription<PrintHeadStatus>(
            "printhead/status",
            monitor_qos,
            [this](PrintHeadStatus::SharedPtr msg){
                std::lock_guard<std::mutex> lk(cache_mutex_);
                last_printhead_status_ = *msg;
            }
        );

        //订阅同步请求
        resync_sub_ = create_subscription<std_msgs::msg::UInt32>(
            "/rsi/resync_request",
            monitor_qos,
            [this](std_msgs::msg::UInt32::SharedPtr){
                {
                    std::lock_guard<std::mutex> qlk(queue_mutex_);
                    if (queue_manager_) {
                        queue_manager_->clear();
                    }
                    last_published_traj_seq_.reset();
                }
            }
        );

        //发布
        traj_pub_ = create_publisher<TrajectoryPoint>("/planned_trajectory", plan_qos);
        event_pub_ = create_publisher<PlannedEvent>("/planned_events", plan_qos);

        initial_prefill();

    }
private:
    void initial_prefill()
    {
        if (!npz_loader_ || !npz_loader_->ok() || !queue_manager_) {
            return;
        }
        std::vector<PlannedEvent> events;
        std::vector<TrajectoryPoint> trajs;
        {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            queue_manager_->fill(*npz_loader_);
            PlannedEvent ev;
            while (queue_manager_->pop_next_event(ev)) {
                events.push_back(ev);
            }
            TrajectoryPoint tp;
            while (trajs.size() < static_cast<size_t>(traj_prefill_) &&
                   queue_manager_->pop_next_traj(tp)) {
                trajs.push_back(tp);
            }
        }

        if (events.size() > static_cast<size_t>(plan_qos_depth_)) {
            RCLCPP_WARN(get_logger(),
                        "计划事件数量超过QoS深度 (%zu > %d)，消息可能丢失",
                        events.size(), plan_qos_depth_);
        }
        for (auto &ev : events) {
            ev.stamp = now();
            event_pub_->publish(ev);
            RCLCPP_DEBUG(get_logger(), "发布事件 序号=%u 类型=%s 内容=%s",
                        ev.trigger_seq,
                        ev.event_type.c_str(),
                        ev.payload.c_str());
        }
        for (auto &tp : trajs) {
            apply_precision(tp);
            tp.stamp = now();
            traj_pub_->publish(tp);
            last_published_traj_seq_ = tp.seq;
            RCLCPP_DEBUG(get_logger(), "预填充轨迹 序号=%u 工具=%d", tp.seq, tp.tool_id);
        }
    }

    void publish_from_queue()
    {
        if (!npz_loader_ || !npz_loader_->ok() || !queue_manager_) {
            return;
        }

        bool ready_for_motion = false;
        {
            std::lock_guard<std::mutex> lk(cache_mutex_);
            if (last_printhead_status_) {
                ready_for_motion = last_printhead_status_->ready_for_motion;
            }
        }

        std::optional<TrajectoryPoint> tp_to_pub;
        std::optional<uint32_t> backlog_to_report;

        {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            queue_manager_->fill(*npz_loader_);
            uint32_t next_traj_seq = 0;
            bool has_traj = queue_manager_->peek_next_traj_seq(next_traj_seq);
            if (has_traj && ready_for_motion) {
                uint32_t seq_used = last_seq_used_.value_or(0);
                uint32_t published_seq = last_published_traj_seq_.value_or(seq_used);
                uint32_t backlog = published_seq >= seq_used ? (published_seq - seq_used) : 0;
                backlog_to_report = backlog;
                TrajectoryPoint tp;
                if (queue_manager_->pop_next_traj(tp)) {
                    tp_to_pub = tp;
                }
            }
        }

        if (backlog_to_report) {
            if (*backlog_to_report < static_cast<uint32_t>(traj_low_)) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "轨迹积压过低：%u (<%d)",
                    *backlog_to_report,
                    traj_low_);
            } else if (*backlog_to_report > static_cast<uint32_t>(traj_high_)) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "轨迹积压过高：%u (>%d)",
                    *backlog_to_report,
                    traj_high_);
            }
        }

        if (tp_to_pub) {
            apply_precision(*tp_to_pub);
            tp_to_pub->stamp = now();
            traj_pub_->publish(*tp_to_pub);
            last_published_traj_seq_ = tp_to_pub->seq;
            RCLCPP_DEBUG(get_logger(), "发布轨迹 序号=%u 工具=%d", tp_to_pub->seq, tp_to_pub->tool_id);
        }
    }

    void apply_precision(TrajectoryPoint& tp)
    {
        const double xyz_scale = std::pow(10.0, xyzabc_decimals_);
        const double e_scale = std::pow(10.0, e_decimals_);
        auto round_n = [](double v, double scale) {
            return std::round(v * scale) / scale;
        };
        tp.x = round_n(tp.x, xyz_scale);
        tp.y = round_n(tp.y, xyz_scale);
        tp.z = round_n(tp.z, xyz_scale);
        tp.a = round_n(tp.a, xyz_scale);
        tp.b = round_n(tp.b, xyz_scale);
        tp.c = round_n(tp.c, xyz_scale);
        tp.e = round_n(tp.e, e_scale);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CenterNode>());
    rclcpp::shutdown();
    return 0;
}
