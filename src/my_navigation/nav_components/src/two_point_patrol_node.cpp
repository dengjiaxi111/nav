#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "nav_interfaces/action/navigate.hpp"

using Navigate = nav_interfaces::action::Navigate;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<Navigate>;

class TwoPointPatrolNode : public rclcpp::Node {
public:
    TwoPointPatrolNode()
        : Node("two_point_patrol") {
        declare_parameter("goal_topic", std::string("goal_pose"));
        declare_parameter("navigate_action_name", std::string("navigate"));
        declare_parameter("goal_frame", std::string("map"));
        declare_parameter("check_period_s", 0.1);
        declare_parameter("near_goal_distance_m", 0.25);
        declare_parameter("switch_delay_s", 0.8);
        declare_parameter("save_file", std::string("/tmp/nav_two_goal_points.txt"));
        declare_parameter("auto_start", false);

        goal_topic_ = get_parameter("goal_topic").as_string();
        action_name_ = get_parameter("navigate_action_name").as_string();
        goal_frame_ = get_parameter("goal_frame").as_string();
        check_period_s_ = get_parameter("check_period_s").as_double();
        near_goal_distance_m_ = get_parameter("near_goal_distance_m").as_double();
        switch_delay_s_ = get_parameter("switch_delay_s").as_double();
        save_file_ = get_parameter("save_file").as_string();
        auto_start_ = get_parameter("auto_start").as_bool();

        if (check_period_s_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "check_period_s=%.3f 非法，自动修正为0.1", check_period_s_);
            check_period_s_ = 0.1;
        }
        if (near_goal_distance_m_ <= 0.0) {
            RCLCPP_WARN(get_logger(),
                        "near_goal_distance_m=%.3f 非法，自动修正为0.25",
                        near_goal_distance_m_);
            near_goal_distance_m_ = 0.25;
        }
        if (switch_delay_s_ < 0.0) {
            RCLCPP_WARN(get_logger(), "switch_delay_s=%.3f 非法，自动修正为0.0", switch_delay_s_);
            switch_delay_s_ = 0.0;
        }

        action_client_ = rclcpp_action::create_client<Navigate>(this, action_name_);
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic_,
            10,
            std::bind(&TwoPointPatrolNode::onGoalPose, this, std::placeholders::_1));

        loadPoints();

        timer_ = create_wall_timer(
            std::chrono::duration<double>(check_period_s_),
            std::bind(&TwoPointPatrolNode::onTimer, this));

        input_thread_ = std::thread([this]() { inputLoop(); });

        RCLCPP_INFO(get_logger(), "two_point_patrol 已启动");
        RCLCPP_INFO(get_logger(), "- 在 RViz 用 2D Goal 点两个点，自动保存到: %s", save_file_.c_str());
        RCLCPP_INFO(get_logger(), "- 在该节点终端按 Enter 开始/暂停循环巡航");
        RCLCPP_INFO(get_logger(),
                "- 当前参数: near_goal_distance_m=%.2f, switch_delay_s=%.2f, check_period_s=%.2f",
                near_goal_distance_m_,
                switch_delay_s_,
                check_period_s_);

        if (auto_start_ && points_.size() == 2) {
            running_ = true;
            current_goal_idx_ = 0;
            goal_in_flight_ = false;
            RCLCPP_INFO(get_logger(), "auto_start=true，已自动进入巡航");
        }
    }

    ~TwoPointPatrolNode() override {
        stop_input_.store(true);
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    void inputLoop() {
        std::string line;
        while (rclcpp::ok() && !stop_input_.load()) {
            if (!std::getline(std::cin, line)) {
                return;
            }

            if (stop_input_.load()) {
                return;
            }

            std::lock_guard<std::mutex> lock(mutex_);
            if (points_.size() < 2) {
                RCLCPP_WARN(get_logger(), "尚未记录2个目标点，当前=%zu", points_.size());
                continue;
            }

            running_ = !running_;
            if (running_) {
                goal_in_flight_ = false;
                active_ticket_ = 0;
                current_goal_idx_ = 0;
                pending_switch_ = false;
                RCLCPP_INFO(get_logger(), "巡航已开启（从点0开始）");
            } else {
                pending_switch_ = false;
                RCLCPP_INFO(get_logger(), "巡航已暂停");
            }
        }
    }

    void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        geometry_msgs::msg::PoseStamped p = *msg;
        if (p.header.frame_id.empty()) {
            p.header.frame_id = goal_frame_;
        }

        if (points_.size() < 2) {
            points_.push_back(p);
            RCLCPP_INFO(get_logger(),
                        "记录目标点[%zu]: (%.3f, %.3f), frame=%s",
                        points_.size() - 1,
                        p.pose.position.x,
                        p.pose.position.y,
                        p.header.frame_id.c_str());
        } else {
            const size_t replace_idx = replace_idx_ % 2;
            points_[replace_idx] = p;
            ++replace_idx_;
            RCLCPP_INFO(get_logger(),
                        "已覆盖目标点[%zu]: (%.3f, %.3f), frame=%s",
                        replace_idx,
                        p.pose.position.x,
                        p.pose.position.y,
                        p.header.frame_id.c_str());
        }

        savePoints();
    }

    void onTimer() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!running_ || points_.size() < 2) {
            return;
        }

        if (!action_client_->action_server_is_ready()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                "等待 action server: %s", action_name_.c_str());
            return;
        }

        const auto now = this->now();

        if (pending_switch_) {
            if (now >= switch_ready_time_) {
                pending_switch_ = false;
                current_goal_idx_ = 1 - current_goal_idx_;
                goal_in_flight_ = false;
                RCLCPP_INFO(get_logger(), "延时结束，切换到目标点[%zu]", current_goal_idx_);
            } else {
                return;
            }
        }

        if (!goal_in_flight_) {
            sendCurrentGoalLocked(now);
        }
    }

    void scheduleSwitchLocked(const rclcpp::Time& now, const char* reason) {
        if (pending_switch_) {
            return;
        }

        switch_ready_time_ = now + rclcpp::Duration::from_seconds(switch_delay_s_);
        pending_switch_ = true;
        RCLCPP_INFO(get_logger(),
                    "目标点[%zu] 触发切换(%s)，%.2fs 后切到目标点[%zu]",
                    current_goal_idx_,
                    reason,
                    switch_delay_s_,
                    1 - current_goal_idx_);
    }

    void sendCurrentGoalLocked(const rclcpp::Time& now) {
        if (current_goal_idx_ > 1 || points_.size() < 2) {
            return;
        }

        Navigate::Goal goal;
        goal.goal_pose = points_[current_goal_idx_];
        goal.goal_pose.header.stamp = now;
        if (goal.goal_pose.header.frame_id.empty()) {
            goal.goal_pose.header.frame_id = goal_frame_;
        }

        ++active_ticket_;
        const uint64_t this_ticket = active_ticket_;
        const size_t this_goal_idx = current_goal_idx_;

        rclcpp_action::Client<Navigate>::SendGoalOptions options;
        options.goal_response_callback =
            [this, this_ticket, this_goal_idx](std::shared_ptr<GoalHandleNavigate> goal_handle) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (this_ticket != active_ticket_) {
                    return;
                }

                if (!goal_handle) {
                    goal_in_flight_ = false;
                    RCLCPP_WARN(get_logger(), "目标点[%zu] 发送被拒绝", this_goal_idx);
                }
            };

        options.feedback_callback =
            [this, this_ticket, this_goal_idx](
                GoalHandleNavigate::SharedPtr,
                const std::shared_ptr<const Navigate::Feedback> feedback) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (this_ticket != active_ticket_) {
                    return;
                }
                if (!feedback || pending_switch_) {
                    return;
                }

                if (feedback->distance_remaining <= near_goal_distance_m_) {
                    const auto now = this->now();
                    scheduleSwitchLocked(now, "near_goal");
                }
            };

        options.result_callback =
            [this, this_ticket, this_goal_idx](const GoalHandleNavigate::WrappedResult& result) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (this_ticket != active_ticket_) {
                    return;
                }

                goal_in_flight_ = false;

                if (pending_switch_) {
                    return;
                }

                if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
                    result.result->success) {
                    scheduleSwitchLocked(this->now(), "action_success");
                    return;
                }

                const char* code_str = "UNKNOWN";
                if (result.code == rclcpp_action::ResultCode::ABORTED) {
                    code_str = "ABORTED";
                } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                    code_str = "CANCELED";
                }

                const std::string msg = (result.result) ? result.result->message : std::string("<none>");
                RCLCPP_WARN(get_logger(),
                            "目标点[%zu] 结果=%s, message=%s；保持当前点等待下一次发送",
                            this_goal_idx,
                            code_str,
                            msg.c_str());
            };

        action_client_->async_send_goal(goal, options);
        goal_in_flight_ = true;

        RCLCPP_INFO(get_logger(),
                    "发送目标点[%zu]: (%.3f, %.3f), frame=%s",
                    this_goal_idx,
                    goal.goal_pose.pose.position.x,
                    goal.goal_pose.pose.position.y,
                    goal.goal_pose.header.frame_id.c_str());
    }

    void savePoints() {
        std::ofstream ofs(save_file_, std::ios::trunc);
        if (!ofs.is_open()) {
            RCLCPP_ERROR(get_logger(), "无法写入点位文件: %s", save_file_.c_str());
            return;
        }

        for (const auto& p : points_) {
            ofs << p.pose.position.x << ' '
                << p.pose.position.y << ' '
                << p.pose.position.z << ' '
                << p.pose.orientation.x << ' '
                << p.pose.orientation.y << ' '
                << p.pose.orientation.z << ' '
                << p.pose.orientation.w << ' '
                << (p.header.frame_id.empty() ? goal_frame_ : p.header.frame_id)
                << '\n';
        }

        RCLCPP_INFO(get_logger(), "点位已保存: %s", save_file_.c_str());
    }

    void loadPoints() {
        std::ifstream ifs(save_file_);
        if (!ifs.is_open()) {
            return;
        }

        std::vector<geometry_msgs::msg::PoseStamped> loaded;
        std::string line;
        while (std::getline(ifs, line)) {
            if (line.empty()) {
                continue;
            }

            std::istringstream iss(line);
            geometry_msgs::msg::PoseStamped p;
            if (!(iss >> p.pose.position.x >> p.pose.position.y >> p.pose.position.z >>
                  p.pose.orientation.x >> p.pose.orientation.y >> p.pose.orientation.z >>
                  p.pose.orientation.w >> p.header.frame_id)) {
                continue;
            }
            loaded.push_back(p);
            if (loaded.size() >= 2) {
                break;
            }
        }

        if (loaded.size() == 2) {
            points_ = loaded;
            RCLCPP_INFO(get_logger(),
                        "已加载2个目标点: [0](%.3f, %.3f), [1](%.3f, %.3f)",
                        points_[0].pose.position.x,
                        points_[0].pose.position.y,
                        points_[1].pose.position.x,
                        points_[1].pose.position.y);
        }
    }

private:
    std::string goal_topic_;
    std::string action_name_;
    std::string goal_frame_;
    std::string save_file_;

    double check_period_s_{0.1};
    double near_goal_distance_m_{0.25};
    double switch_delay_s_{0.8};
    bool auto_start_{false};

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp_action::Client<Navigate>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::PoseStamped> points_;
    size_t replace_idx_{0};
    size_t current_goal_idx_{0};
    uint64_t active_ticket_{0};

    bool running_{false};
    bool goal_in_flight_{false};
    bool pending_switch_{false};
    rclcpp::Time switch_ready_time_{0, 0, RCL_ROS_TIME};

    std::thread input_thread_;
    std::atomic<bool> stop_input_{false};
    std::mutex mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwoPointPatrolNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
