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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;

    rclcpp::TimerBase::SharedPtr summary_timer_;

    std::mutex cache_mutex_;
    std::optional<std::string> last_kuka_xml_;
    std::optional<KukaStatus> last_kuka_status_;
    std::optional<RsiHeartBeat> last_hb_;
    std::optional<PrintHeadStatus> last_printhead_status_;
    std::optional<uint32_t> last_resync_seq_;
    rclcpp::Time last_kuka_stamp_;
    rclcpp::Time last_kuka_status_stamp_;
    rclcpp::Time last_hb_stamp_;
    rclcpp::Time last_printhead_status_stamp_;
    rclcpp::Time last_resync_stamp_;

    int summary_period_ms_{1000};
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
            RCLCPP_ERROR(get_logger(), "NPZ loader init failed: %s", npz_loader_->error().c_str());
        } else {
            RCLCPP_INFO(get_logger(), "NPZ loader ready: %s (preload=%ld)",
                        npz_path.c_str(), static_cast<long>(npz_preload));
        }
        queue_manager_ = std::make_unique<control_center::QueueManager>(
            static_cast<size_t>(queue_low), static_cast<size_t>(queue_high));
        if (npz_loader_->ok()) {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            queue_manager_->fill(*npz_loader_);
        }

        kuka_status_raw_ = declare_parameter<bool>("kuka_status_raw", false);
        summary_period_ms_ = declare_parameter<int>("summary_period_ms", 200);

        auto monitor_qos = rclcpp::QoS(10);
        auto plan_qos = rclcpp::QoS(plan_qos_depth_).reliable();

        //订阅RSI原始KUKA XML
        kuka_raw_sub_ = create_subscription<String>(
            "/kuka/raw_xml",
            monitor_qos,
            [this](std_msgs::msg::String::SharedPtr msg){
                std::lock_guard<std::mutex> lk(cache_mutex_);
                last_kuka_xml_ = msg->data;
                last_kuka_stamp_ = now();
                if (kuka_status_raw_){
                    RCLCPP_INFO(get_logger(), "KUKA端原始xml长度=%zu", last_kuka_xml_ ->size());
                }
            }
        );

        //订阅KUKA状态
        kuka_status_sub_ = create_subscription<KukaStatus>(
            "kuka/status",
            monitor_qos,
            [this](KukaStatus::SharedPtr msg){
                std::lock_guard<std::mutex> lk(cache_mutex_);
                last_kuka_status_ = *msg;
                last_kuka_status_stamp_ = now();
            }
        );

        //订阅RSI心跳包
        hb_sub_ = create_subscription<RsiHeartBeat>(
            "/rsi/heartbeat",
            monitor_qos,
            [this](RsiHeartBeat::SharedPtr msg){
                {
                    std::lock_guard<std::mutex> lk(cache_mutex_);
                    last_hb_ = *msg;
                    last_hb_stamp_ = now();
                    last_seq_used_ = msg->seq_used;
                }
                publish_from_queue();
            }
        );

        //订阅打印头状态
        printhead_status_sub_ = create_subscription<PrintHeadStatus>(
            "printhead/status",
            monitor_qos,
            [this](PrintHeadStatus::SharedPtr msg){
                {
                    std::lock_guard<std::mutex> lk(cache_mutex_);
                    last_printhead_status_ = *msg;
                    last_printhead_status_stamp_ = now();
                }
            }
        );

        //订阅同步请求
        resync_sub_ = create_subscription<std_msgs::msg::UInt32>(
            "/rsi/resync_request",
            monitor_qos,
            [this](std_msgs::msg::UInt32::SharedPtr msg){
                std::lock_guard<std::mutex> lk(cache_mutex_);
                last_resync_seq_ = msg->data;
                last_resync_stamp_ = now();
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

        summary_pub_ = create_publisher<std_msgs::msg::String>("/control_center/summary",monitor_qos);

        summary_timer_ = create_wall_timer(
            std::chrono::milliseconds(summary_period_ms_),
            [this](){
                publish_summary(); 
            });
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
                        "planned_events exceeds QoS depth (%zu > %d), messages may drop",
                        events.size(), plan_qos_depth_);
        }
        for (auto &ev : events) {
            ev.stamp = now();
            event_pub_->publish(ev);
            RCLCPP_INFO(get_logger(), "publish event seq=%u type=%s payload=%s",
                        ev.trigger_seq,
                        ev.event_type.c_str(),
                        ev.payload.c_str());
        }
        for (auto &tp : trajs) {
            apply_precision(tp);
            tp.stamp = now();
            traj_pub_->publish(tp);
            last_published_traj_seq_ = tp.seq;
            RCLCPP_INFO(get_logger(), "prefill traj seq=%u tool=%d", tp.seq, tp.tool_id);
        }
    }

    //各类状态信息
    void publish_summary()
    {
        std::optional<std::string> kuka_xml;
        std::optional<KukaStatus> kuka_status;
        std::optional<RsiHeartBeat> hb;
        std::optional<PrintHeadStatus> printhead_status;
        std::optional<uint32_t> resync_seq;
        rclcpp::Time kuka_stamp, kuka_status_stamp, hb_stamp, printhead_status_stamp, resync_stamp;

        {
            std::lock_guard<std::mutex> lk(cache_mutex_);
            if(last_kuka_xml_) { kuka_xml = last_kuka_xml_; kuka_stamp = last_kuka_stamp_;}
            if(last_kuka_status_) { kuka_status = last_kuka_status_; kuka_status_stamp = last_kuka_status_stamp_;}
            if(last_hb_) { hb = last_hb_; hb_stamp = last_hb_stamp_;}
            if(last_printhead_status_) {printhead_status = last_printhead_status_; printhead_status_stamp = last_printhead_status_stamp_;}
            if(last_resync_seq_) { resync_seq = last_resync_seq_; resync_stamp = last_resync_stamp_; }
        }

        auto now_t = now();
        std::ostringstream oss;
        oss << "stamp=" << now_t.seconds();

        if(hb){
            oss << "seq_used=" << hb->seq_used
                << "ipoc=" << hb->ipoc
                << " tool=" << hb->tool_id
                << "extrude_abs=" << hb->extrude_abs
                << "hb_age_s=" << (now_t - hb_stamp).seconds();
        } else {
            oss << "seq_used=n/a ipoc=n/a tool=n/a extrude_abs=n/a";
        }

        if(printhead_status){
            oss << "ready=" << (printhead_status->ready_for_motion ? "1" : "0")
                << " event_seq=" << printhead_status->ready_event_seq
                << " event_type=" << printhead_status->ready_event_type
                << " temp_cf=" << printhead_status->current_temp_cf << "/" << printhead_status->target_temp_cf
                << " temp_resin=" << printhead_status->current_temp_resin << "/" << printhead_status->target_temp_resin
                << " fan_cf=" << (printhead_status->fan_ok_cf ? "1" : "0")
                << " fan_resin=" << (printhead_status->fan_ok_resin ? "1" : "0")
                << " tool=" << printhead_status->current_tool
                << " err=" << printhead_status->error_code
                << " printhead_status_age_s=" << (now_t - printhead_status_stamp).seconds();
        } else {
            oss << " ready=n/a event_seq=n/a event_type=n/a";
        }

        if(kuka_status){
            oss << "kuka_xyzabc="<<kuka_status->x <<","<<kuka_status->y<<","<<kuka_status->z<<","<<kuka_status->a<<","
                << kuka_status->b<<","<<kuka_status->c<<","<<"kuka_status_age_s" << (now_t - kuka_status_stamp).seconds();
        } else if (kuka_xml){
            oss << " kuka_xml_len=" << kuka_xml->size()
                << " kuka_xml_age_s=" << (now_t - kuka_stamp).seconds();
        } else {
            oss << "kuka_xml_len=n/a";
        }

        if (resync_seq) {
            oss << " resync_seq=" << *resync_seq
            << " resync_age_s=" << (now_t - resync_stamp).seconds();
        }
        
        std_msgs::msg::String out;
        out.data = oss.str();
        summary_pub_ -> publish(out);
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
                RCLCPP_WARN(get_logger(), "traj backlog low: %u (<%d)", *backlog_to_report, traj_low_);
            } else if (*backlog_to_report > static_cast<uint32_t>(traj_high_)) {
                RCLCPP_WARN(get_logger(), "traj backlog high: %u (>%d)", *backlog_to_report, traj_high_);
            }
        }

        if (tp_to_pub) {
            apply_precision(*tp_to_pub);
            tp_to_pub->stamp = now();
            traj_pub_->publish(*tp_to_pub);
            last_published_traj_seq_ = tp_to_pub->seq;
            RCLCPP_INFO(get_logger(), "publish traj seq=%u tool=%d", tp_to_pub->seq, tp_to_pub->tool_id);
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
