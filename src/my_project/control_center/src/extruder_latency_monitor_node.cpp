// =============================================================
// 描述：
// 监控机械臂 RSI 心跳与固件挤出 EACK/STAT 的相对延迟。
// 该节点只订阅和发布监控信息，不参与控制闭环。
// =============================================================

#include <rclcpp/rclcpp.hpp>

#include <my_project_interfaces/msg/extruder_latency_status.hpp>
#include <my_project_interfaces/msg/kuka_status.hpp>
#include <my_project_interfaces/msg/print_head_status.hpp>
#include <my_project_interfaces/msg/rsi_heart_beat.hpp>
#include <my_project_interfaces/msg/trajectory_point.hpp>

#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using my_project_interfaces::msg::ExtruderLatencyStatus;
using my_project_interfaces::msg::KukaStatus;
using my_project_interfaces::msg::PrintHeadStatus;
using my_project_interfaces::msg::RsiHeartBeat;
using my_project_interfaces::msg::TrajectoryPoint;

class ExtruderLatencyMonitorNode : public rclcpp::Node
{
public:
    ExtruderLatencyMonitorNode() : Node("extruder_latency_monitor_node")
    {
        rsi_period_ms_ = declare_parameter<double>("rsi_period_ms", 4.0);
        publish_period_ms_ = declare_parameter<int>("latency_publish_period_ms", 200);
        history_limit_ = declare_parameter<int>("latency_history_limit", 5000);
        robot_cache_back_ = declare_parameter<int>("robot_match_cache_back", 8000);
        robot_cache_forward_ = declare_parameter<int>("robot_match_cache_forward", 1000);
        robot_search_back_ = declare_parameter<int>("robot_match_search_back", 5000);
        robot_search_forward_ = declare_parameter<int>("robot_match_search_forward", 300);
        robot_max_error_mm_ = declare_parameter<double>("robot_match_max_error_mm", 1.0);
        robot_uncertainty_min_band_mm_ = declare_parameter<double>("robot_match_uncertainty_min_band_mm", 0.10);
        robot_uncertainty_spacing_multiplier_ = declare_parameter<double>("robot_match_uncertainty_spacing_multiplier", 3.0);
        nozzle_lever_mm_ = declare_parameter<double>("robot_match_nozzle_lever_mm", 100.0);
        stats_window_limit_ = declare_parameter<int>("latency_stats_window_limit", 5000);

        status_pub_ = create_publisher<ExtruderLatencyStatus>("/extruder/latency_status", 10);
        text_pub_ = create_publisher<std_msgs::msg::String>("/extruder/latency_text", 10);

        hb_sub_ = create_subscription<RsiHeartBeat>(
            "/rsi/heartbeat",
            rclcpp::QoS(2000).reliable(),
            [this](RsiHeartBeat::SharedPtr msg) {
                on_heartbeat(*msg);
            });

        uart_raw_sub_ = create_subscription<std_msgs::msg::String>(
            "/uart/raw",
            rclcpp::QoS(2000).reliable(),
            [this](std_msgs::msg::String::SharedPtr msg) {
                on_uart_raw(msg->data);
            });

        printhead_sub_ = create_subscription<PrintHeadStatus>(
            "/printhead/status",
            rclcpp::QoS(200).reliable(),
            [this](PrintHeadStatus::SharedPtr msg) {
                on_printhead_status(*msg);
            });

        trajectory_sub_ = create_subscription<TrajectoryPoint>(
            "/planned_trajectory",
            rclcpp::QoS(2000).reliable(),
            [this](TrajectoryPoint::SharedPtr msg) {
                on_trajectory(*msg);
            });

        kuka_status_sub_ = create_subscription<KukaStatus>(
            "/kuka/status",
            rclcpp::QoS(200).reliable(),
            [this](KukaStatus::SharedPtr msg) {
                on_kuka_status(*msg);
            });

        timer_ = create_wall_timer(
            std::chrono::milliseconds(publish_period_ms_),
            [this]() {
                publish_status();
            });

        RCLCPP_INFO(get_logger(), "extruder_latency_monitor_node 已启动");
    }

private:
    struct AckInfo
    {
        uint32_t seq{0};
        int32_t tool_id{0};
        double extrude_abs{0.0};
        uint64_t mcu_us{0};
        double delay_ms{std::numeric_limits<double>::quiet_NaN()};
    };

    struct StatInfo
    {
        uint32_t last_e_seq{0};
        int32_t last_e_tool{0};
        double last_e_abs{0.0};
        uint64_t last_e_us{0};
        bool valid{false};
    };

    struct PlannedPose
    {
        uint32_t seq{0};
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double a{0.0};
        double b{0.0};
        double c{0.0};
        double e{0.0};
        int32_t tool_id{0};
    };

    struct RobotMatch
    {
        bool valid{false};
        uint32_t seq{0};
        double error_mm{std::numeric_limits<double>::quiet_NaN()};
        double uncertainty_ms{0.0};
        uint32_t seq_min{0};
        uint32_t seq_max{0};
    };

    struct RollingStats
    {
        void add(double value, size_t limit)
        {
            if (!std::isfinite(value)) {
                return;
            }
            samples.push_back(value);
            while (samples.size() > limit) {
                samples.pop_front();
            }
        }

        double avg() const
        {
            if (samples.empty()) {
                return 0.0;
            }
            double sum = 0.0;
            for (double v : samples) {
                sum += v;
            }
            return sum / static_cast<double>(samples.size());
        }

        double percentile(double p) const
        {
            if (samples.empty()) {
                return 0.0;
            }
            std::vector<double> sorted(samples.begin(), samples.end());
            std::sort(sorted.begin(), sorted.end());
            const double pos = (p / 100.0) * static_cast<double>(sorted.size() - 1);
            const size_t lo = static_cast<size_t>(std::floor(pos));
            const size_t hi = static_cast<size_t>(std::ceil(pos));
            if (lo == hi) {
                return sorted[lo];
            }
            const double t = pos - static_cast<double>(lo);
            return sorted[lo] * (1.0 - t) + sorted[hi] * t;
        }

        double abs_percentile(double p) const
        {
            if (samples.empty()) {
                return 0.0;
            }
            std::vector<double> sorted;
            sorted.reserve(samples.size());
            for (double v : samples) {
                sorted.push_back(std::abs(v));
            }
            std::sort(sorted.begin(), sorted.end());
            const double pos = (p / 100.0) * static_cast<double>(sorted.size() - 1);
            const size_t lo = static_cast<size_t>(std::floor(pos));
            const size_t hi = static_cast<size_t>(std::ceil(pos));
            if (lo == hi) {
                return sorted[lo];
            }
            const double t = pos - static_cast<double>(lo);
            return sorted[lo] * (1.0 - t) + sorted[hi] * t;
        }

        size_t count() const { return samples.size(); }

        std::deque<double> samples;
    };

    void on_heartbeat(const RsiHeartBeat& hb)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        current_arm_seq_ = hb.seq_used;
        have_arm_seq_ = true;
        hb_times_[hb.seq_used] = rclcpp::Time(hb.stamp);
        hb_order_.push_back(hb.seq_used);
        trim_history();
        trim_trajectory_cache_locked();
    }

    static double angle_diff_deg(double from, double to)
    {
        double diff = std::fmod(from - to + 180.0, 360.0);
        if (diff < 0.0) {
            diff += 360.0;
        }
        return diff - 180.0;
    }

    double pose_error_mm(const PlannedPose& planned, const KukaStatus& actual) const
    {
        const double dx = actual.x - planned.x;
        const double dy = actual.y - planned.y;
        const double dz = actual.z - planned.z;
        const double xyz = std::sqrt(dx * dx + dy * dy + dz * dz);

        const double da = angle_diff_deg(actual.a, planned.a);
        const double db = angle_diff_deg(actual.b, planned.b);
        const double dc = angle_diff_deg(actual.c, planned.c);
        const double abc_deg = std::sqrt(da * da + db * db + dc * dc);
        const double orient = nozzle_lever_mm_ * std::sin(abc_deg * 3.14159265358979323846 / 180.0);
        return std::sqrt(xyz * xyz + orient * orient);
    }

    double local_spacing_mm(size_t best_index) const
    {
        if (trajectory_cache_.size() < 2) {
            return 0.04;
        }
        std::vector<double> distances;
        const size_t start = best_index > 20 ? best_index - 20 : 0;
        const size_t end = std::min(trajectory_cache_.size() - 1, best_index + 20);
        for (size_t i = start; i < end; ++i) {
            const auto& a = trajectory_cache_[i];
            const auto& b = trajectory_cache_[i + 1];
            if (b.seq != a.seq + 1) {
                continue;
            }
            const double dx = b.x - a.x;
            const double dy = b.y - a.y;
            const double dz = b.z - a.z;
            const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (d > 1e-6 && std::isfinite(d)) {
                distances.push_back(d);
            }
        }
        if (distances.empty()) {
            return 0.04;
        }
        std::sort(distances.begin(), distances.end());
        return distances[distances.size() / 2];
    }

    void on_trajectory(const TrajectoryPoint& point)
    {
        PlannedPose pose;
        pose.seq = point.seq;
        pose.x = point.x;
        pose.y = point.y;
        pose.z = point.z;
        pose.a = point.a;
        pose.b = point.b;
        pose.c = point.c;
        pose.e = point.e;
        pose.tool_id = point.tool_id;

        std::lock_guard<std::mutex> lk(mutex_);
        trajectory_cache_.push_back(pose);
        trim_trajectory_cache_locked();
    }

    void on_kuka_status(const KukaStatus& status)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        last_robot_match_ = match_robot_locked(status);
        if (!last_robot_match_.valid || !have_arm_seq_) {
            return;
        }

        const double linux_robot =
            (static_cast<double>(current_arm_seq_) - static_cast<double>(last_robot_match_.seq)) * rsi_period_ms_;
        linux_robot_stats_.add(linux_robot, stats_limit());

        if (last_ack_) {
            const double mcu_robot =
                (static_cast<double>(last_robot_match_.seq) - static_cast<double>(last_ack_->seq)) * rsi_period_ms_;
            mcu_robot_stats_.add(mcu_robot, stats_limit());
        }
    }

    RobotMatch match_robot_locked(const KukaStatus& status) const
    {
        RobotMatch result;
        if (trajectory_cache_.empty() || !have_arm_seq_) {
            return result;
        }

        const int64_t current = static_cast<int64_t>(current_arm_seq_);
        const int64_t min_seq = current - std::max(0, robot_search_back_);
        const int64_t max_seq = current + std::max(0, robot_search_forward_);

        double best_error = std::numeric_limits<double>::infinity();
        size_t best_index = 0;
        bool found = false;

        for (size_t i = 0; i < trajectory_cache_.size(); ++i) {
            const auto& pose = trajectory_cache_[i];
            const int64_t seq = static_cast<int64_t>(pose.seq);
            if (seq < min_seq || seq > max_seq) {
                continue;
            }
            const double error = pose_error_mm(pose, status);
            if (error < best_error) {
                best_error = error;
                best_index = i;
                found = true;
            }
        }

        if (!found) {
            return result;
        }

        const double spacing = local_spacing_mm(best_index);
        const double band = std::max(
            robot_uncertainty_min_band_mm_,
            robot_uncertainty_spacing_multiplier_ * spacing);
        uint32_t seq_min = trajectory_cache_[best_index].seq;
        uint32_t seq_max = seq_min;

        for (const auto& pose : trajectory_cache_) {
            const int64_t seq = static_cast<int64_t>(pose.seq);
            if (seq < min_seq || seq > max_seq) {
                continue;
            }
            const double error = pose_error_mm(pose, status);
            if (error <= best_error + band) {
                seq_min = std::min(seq_min, pose.seq);
                seq_max = std::max(seq_max, pose.seq);
            }
        }

        result.seq = trajectory_cache_[best_index].seq;
        result.error_mm = best_error;
        result.seq_min = seq_min;
        result.seq_max = seq_max;
        result.uncertainty_ms =
            (static_cast<double>(seq_max) - static_cast<double>(seq_min)) * rsi_period_ms_ * 0.5;
        result.valid = best_error <= robot_max_error_mm_;
        return result;
    }

    void trim_trajectory_cache_locked()
    {
        if (!have_arm_seq_) {
            const size_t max_size = static_cast<size_t>(std::max(1, robot_cache_back_ + robot_cache_forward_));
            while (trajectory_cache_.size() > max_size) {
                trajectory_cache_.pop_front();
            }
            return;
        }

        const int64_t min_seq = static_cast<int64_t>(current_arm_seq_) - std::max(0, robot_cache_back_);
        while (!trajectory_cache_.empty() && static_cast<int64_t>(trajectory_cache_.front().seq) < min_seq) {
            trajectory_cache_.pop_front();
        }

        const size_t max_size = static_cast<size_t>(std::max(1, robot_cache_back_ + robot_cache_forward_));
        while (trajectory_cache_.size() > max_size) {
            trajectory_cache_.pop_front();
        }
    }

    size_t stats_limit() const
    {
        return static_cast<size_t>(std::max(1, stats_window_limit_));
    }

    void on_uart_raw(const std::string& line)
    {
        if (line.rfind("EACK ", 0) == 0) {
            handle_eack(line);
        } else if (line.rfind("EWARN ", 0) == 0) {
            handle_ewarn(line);
        }
    }

    void on_printhead_status(const PrintHeadStatus& status)
    {
        if (status.raw.rfind("STAT", 0) != 0) {
            return;
        }
        handle_stat(status.raw);
    }

    void handle_eack(const std::string& line)
    {
        std::istringstream ss(line);
        std::string tag;
        AckInfo ack;
        ss >> tag >> ack.seq >> ack.tool_id >> ack.extrude_abs;
        if (!ss) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "EACK 解析失败：%s", line.c_str());
            return;
        }
        ss >> ack.mcu_us;

        const auto ack_receive_time = now();
        {
            std::lock_guard<std::mutex> lk(mutex_);
            auto it = hb_times_.find(ack.seq);
            if (it != hb_times_.end()) {
                ack.delay_ms = (ack_receive_time - it->second).seconds() * 1000.0;
                update_delay_stats(ack.delay_ms);
            }
            if (have_arm_seq_) {
                const double linux_mcu =
                    (static_cast<double>(current_arm_seq_) - static_cast<double>(ack.seq)) * rsi_period_ms_;
                linux_mcu_stats_.add(linux_mcu, stats_limit());
            }
            last_ack_ = ack;
            ++eack_count_;
            last_raw_ = line;
        }

        publish_status();
    }

    void handle_ewarn(const std::string& line)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        last_warn_ = line;
        last_raw_ = line;
        if (line.find("old_seq=") != std::string::npos) {
            ++old_seq_warn_count_;
        }
        if (line.find("gap ") != std::string::npos ||
            line.find("gap=") != std::string::npos) {
            ++gap_warn_count_;
        }
    }

    void handle_stat(const std::string& line)
    {
        StatInfo next = last_stat_;
        next.valid = true;

        std::istringstream ss(line);
        std::string tag;
        std::string kv;
        ss >> tag;
        while (ss >> kv) {
            auto eq = kv.find('=');
            if (eq == std::string::npos) {
                continue;
            }
            const auto key = kv.substr(0, eq);
            const auto value = kv.substr(eq + 1);
            try {
                if (key == "last_e_seq") {
                    next.last_e_seq = static_cast<uint32_t>(std::stoul(value));
                } else if (key == "last_e_tool") {
                    next.last_e_tool = std::stoi(value);
                } else if (key == "last_e_abs") {
                    next.last_e_abs = std::stod(value);
                } else if (key == "last_e_us") {
                    next.last_e_us = static_cast<uint64_t>(std::stoull(value));
                }
            } catch (const std::exception&) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "STAT 延迟字段解析失败：%s", kv.c_str());
            }
        }

        std::lock_guard<std::mutex> lk(mutex_);
        last_stat_ = next;
        last_raw_ = line;
    }

    void update_delay_stats(double delay_ms)
    {
        if (!std::isfinite(delay_ms)) {
            return;
        }
        if (delay_count_ == 0) {
            delay_min_ms_ = delay_ms;
            delay_max_ms_ = delay_ms;
        } else {
            delay_min_ms_ = std::min(delay_min_ms_, delay_ms);
            delay_max_ms_ = std::max(delay_max_ms_, delay_ms);
        }
        ++delay_count_;
        delay_sum_ms_ += delay_ms;
        delay_sum_sq_ms_ += delay_ms * delay_ms;
    }

    void trim_history()
    {
        const auto limit = static_cast<size_t>(std::max(1, history_limit_));
        while (hb_order_.size() > limit) {
            const auto seq = hb_order_.front();
            hb_order_.pop_front();
            hb_times_.erase(seq);
        }
    }

    ExtruderLatencyStatus make_status_locked()
    {
        ExtruderLatencyStatus out;
        out.stamp = now();
        out.arm_seq = current_arm_seq_;
        out.has_eack = last_ack_.has_value();
        out.has_stat = last_stat_.valid;
        out.last_eack_seq = last_ack_ ? last_ack_->seq : 0;
        out.stat_last_e_seq = last_stat_.valid ? last_stat_.last_e_seq : 0;

        uint32_t compare_seq = 0;
        if (last_stat_.valid) {
            compare_seq = last_stat_.last_e_seq;
        } else if (last_ack_) {
            compare_seq = last_ack_->seq;
        }

        if (have_arm_seq_ && compare_seq > 0) {
            out.seq_lag = static_cast<int64_t>(current_arm_seq_) - static_cast<int64_t>(compare_seq);
            out.cycle_lag_ms = static_cast<double>(out.seq_lag) * rsi_period_ms_;
        }

        if (have_arm_seq_ && last_ack_) {
            out.linux_mcu_delay_ms =
                (static_cast<double>(current_arm_seq_) - static_cast<double>(last_ack_->seq)) * rsi_period_ms_;
        }
        out.linux_mcu_p95_ms = linux_mcu_stats_.percentile(95.0);
        out.linux_mcu_p99_ms = linux_mcu_stats_.percentile(99.0);

        out.has_robot_match = last_robot_match_.valid;
        out.actual_robot_seq = last_robot_match_.seq;
        out.robot_match_error_mm = last_robot_match_.error_mm;
        out.robot_match_uncertainty_ms = last_robot_match_.uncertainty_ms;
        out.robot_match_max_error_mm = robot_max_error_mm_;
        if (last_robot_match_.valid && have_arm_seq_) {
            out.linux_robot_delay_ms =
                (static_cast<double>(current_arm_seq_) - static_cast<double>(last_robot_match_.seq)) * rsi_period_ms_;
            out.linux_robot_p95_ms = linux_robot_stats_.percentile(95.0);
            out.linux_robot_p99_ms = linux_robot_stats_.percentile(99.0);
            if (last_ack_) {
                out.mcu_robot_delay_ms =
                    (static_cast<double>(last_robot_match_.seq) - static_cast<double>(last_ack_->seq)) * rsi_period_ms_;
            }
        }
        out.mcu_robot_avg_ms = mcu_robot_stats_.avg();
        out.mcu_robot_abs_p95_ms = mcu_robot_stats_.abs_percentile(95.0);
        out.mcu_robot_abs_p99_ms = mcu_robot_stats_.abs_percentile(99.0);

        if (last_ack_) {
            out.ack_delay_ms = last_ack_->delay_ms;
            out.last_tool_id = last_ack_->tool_id;
            out.last_extrude_abs = last_ack_->extrude_abs;
            out.last_mcu_us = last_ack_->mcu_us;
        }
        if (last_stat_.valid) {
            out.last_tool_id = last_stat_.last_e_tool;
            out.last_extrude_abs = last_stat_.last_e_abs;
            out.last_mcu_us = last_stat_.last_e_us;
        }

        out.ack_delay_min_ms = delay_count_ > 0 ? delay_min_ms_ : 0.0;
        out.ack_delay_avg_ms = delay_count_ > 0 ? delay_sum_ms_ / static_cast<double>(delay_count_) : 0.0;
        out.ack_delay_max_ms = delay_count_ > 0 ? delay_max_ms_ : 0.0;
        if (delay_count_ > 0) {
            const double avg = out.ack_delay_avg_ms;
            const double variance = std::max(
                0.0,
                delay_sum_sq_ms_ / static_cast<double>(delay_count_) - avg * avg);
            out.ack_delay_jitter_ms = std::sqrt(variance);
        }

        out.eack_count = eack_count_;
        out.old_seq_warn_count = old_seq_warn_count_;
        out.gap_warn_count = gap_warn_count_;
        out.last_warn = last_warn_;
        out.last_raw = last_raw_;
        return out;
    }

    void publish_status()
    {
        ExtruderLatencyStatus status;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            status = make_status_locked();
        }

        status_pub_->publish(status);

        std_msgs::msg::String text;
        std::ostringstream oss;
        oss << "arm_seq=" << status.arm_seq
            << " last_eack_seq=" << status.last_eack_seq
            << " stat_last_e_seq=" << status.stat_last_e_seq
            << " seq_lag=" << status.seq_lag
            << " cycle_lag_ms=" << status.cycle_lag_ms
            << " ack_delay_ms=" << status.ack_delay_ms
            << " ack_avg_ms=" << status.ack_delay_avg_ms
            << " ack_jitter_ms=" << status.ack_delay_jitter_ms
            << " linux_mcu_ms=" << status.linux_mcu_delay_ms
            << " linux_robot_ms=" << status.linux_robot_delay_ms
            << " mcu_robot_ms=" << status.mcu_robot_delay_ms
            << " robot_uncertainty_ms=" << status.robot_match_uncertainty_ms
            << " robot_error_mm=" << status.robot_match_error_mm
            << " eack_count=" << status.eack_count
            << " warn_old=" << status.old_seq_warn_count
            << " warn_gap=" << status.gap_warn_count;
        if (!status.last_warn.empty()) {
            oss << " last_warn=\"" << status.last_warn << "\"";
        }
        text.data = oss.str();
        text_pub_->publish(text);
    }

    std::mutex mutex_;
    std::unordered_map<uint32_t, rclcpp::Time> hb_times_;
    std::deque<uint32_t> hb_order_;

    uint32_t current_arm_seq_{0};
    bool have_arm_seq_{false};
    std::optional<AckInfo> last_ack_;
    StatInfo last_stat_;
    RobotMatch last_robot_match_;
    std::deque<PlannedPose> trajectory_cache_;

    uint64_t eack_count_{0};
    uint64_t old_seq_warn_count_{0};
    uint64_t gap_warn_count_{0};

    RollingStats linux_mcu_stats_;
    RollingStats linux_robot_stats_;
    RollingStats mcu_robot_stats_;

    uint64_t delay_count_{0};
    double delay_sum_ms_{0.0};
    double delay_sum_sq_ms_{0.0};
    double delay_min_ms_{0.0};
    double delay_max_ms_{0.0};

    std::string last_warn_;
    std::string last_raw_;

    double rsi_period_ms_{4.0};
    int publish_period_ms_{200};
    int history_limit_{5000};
    int robot_cache_back_{8000};
    int robot_cache_forward_{1000};
    int robot_search_back_{5000};
    int robot_search_forward_{300};
    int stats_window_limit_{5000};
    double robot_max_error_mm_{1.0};
    double robot_uncertainty_min_band_mm_{0.10};
    double robot_uncertainty_spacing_multiplier_{3.0};
    double nozzle_lever_mm_{100.0};

    rclcpp::Subscription<RsiHeartBeat>::SharedPtr hb_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr uart_raw_sub_;
    rclcpp::Subscription<PrintHeadStatus>::SharedPtr printhead_sub_;
    rclcpp::Subscription<TrajectoryPoint>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<KukaStatus>::SharedPtr kuka_status_sub_;
    rclcpp::Publisher<ExtruderLatencyStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExtruderLatencyMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
