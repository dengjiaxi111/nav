// nav_components/include/nav_components/arc_backup_recovery.hpp
// 弧线后退恢复行为

#pragma once
#include <nav_core/recovery_base.hpp>
#include <chrono>
#include <functional>
#include <string>

namespace nav_components {

class ArcBackupRecovery : public nav_core::RecoveryBase {
public:
    using SafetyChecker = std::function<bool(const geometry_msgs::msg::PoseStamped&,
                                             double,
                                             double,
                                             double,
                                             std::string*)>;

    void initialize(rclcpp::Node* node, nav_core::VelPublisher vel_pub) override;
    void initializeConfigured(rclcpp::Node* node,
                              nav_core::VelPublisher vel_pub,
                              double backup_dist,
                              double linear_vel,
                              double angular_vel,
                              double timeout_s,
                              double min_progress,
                              double progress_timeout_s,
                              double safety_check_duration_s,
                              bool enable_safety_check,
                              bool allow_unsafe,
                              const std::string& name);
    void setSafetyChecker(SafetyChecker checker);
    void start(const geometry_msgs::msg::PoseStamped& current_pose) override;
    nav_core::RecoveryStatus update(const geometry_msgs::msg::PoseStamped& current_pose) override;
    void cancel() override;
    const char* name() const override { return name_.c_str(); }

private:
    rclcpp::Node* node_ = nullptr;
    nav_core::VelPublisher vel_pub_;
    nav_core::RecoveryStatus status_ = nav_core::RecoveryStatus::IDLE;

    geometry_msgs::msg::PoseStamped start_pose_;
    double backup_dist_ = 0.25;
    double linear_vel_ = -0.15;
    double angular_vel_ = 0.4;
    double traveled_ = 0.0;
    double timeout_s_ = 2.0;
    double min_progress_ = 0.02;
    double progress_timeout_s_ = 1.0;
    double safety_check_duration_s_ = 1.5;
    bool enable_safety_check_ = true;
    bool allow_unsafe_ = false;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_progress_time_;
    double last_progress_traveled_ = 0.0;
    SafetyChecker safety_checker_;
    std::string name_ = "arc_backup";
};

}  // namespace nav_components
