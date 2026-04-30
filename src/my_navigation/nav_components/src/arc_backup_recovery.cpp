// nav_components/src/arc_backup_recovery.cpp

#include "nav_components/arc_backup_recovery.hpp"
#include <cmath>
#include <utility>

namespace nav_components {

void ArcBackupRecovery::initialize(rclcpp::Node* node, nav_core::VelPublisher vel_pub) {
    node_ = node;
    vel_pub_ = vel_pub;
    backup_dist_ = node_->declare_parameter("recovery.arc_backup_dist", 0.25);
    linear_vel_ = node_->declare_parameter("recovery.arc_backup_vel", -0.15);
    angular_vel_ = node_->declare_parameter("recovery.arc_backup_angular_vel", 0.4);
    timeout_s_ = node_->declare_parameter("recovery.arc_backup_timeout", 2.0);
    min_progress_ = node_->declare_parameter("recovery.arc_backup_min_progress", 0.02);
    progress_timeout_s_ =
        node_->declare_parameter("recovery.arc_backup_progress_timeout", 1.0);
    safety_check_duration_s_ =
        node_->declare_parameter("recovery.arc_backup_safety_check_duration", 1.5);
    enable_safety_check_ =
        node_->declare_parameter("recovery.arc_backup_enable_safety_check", true);
    allow_unsafe_ = node_->declare_parameter("recovery.arc_backup_allow_unsafe", false);
}

void ArcBackupRecovery::initializeConfigured(rclcpp::Node* node,
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
                                             const std::string& name) {
    node_ = node;
    vel_pub_ = vel_pub;
    backup_dist_ = backup_dist;
    linear_vel_ = linear_vel;
    angular_vel_ = angular_vel;
    timeout_s_ = timeout_s;
    min_progress_ = min_progress;
    progress_timeout_s_ = progress_timeout_s;
    safety_check_duration_s_ = safety_check_duration_s;
    enable_safety_check_ = enable_safety_check;
    allow_unsafe_ = allow_unsafe;
    name_ = name;
}

void ArcBackupRecovery::setSafetyChecker(SafetyChecker checker) {
    safety_checker_ = std::move(checker);
}

void ArcBackupRecovery::start(const geometry_msgs::msg::PoseStamped& current_pose) {
    start_pose_ = current_pose;
    traveled_ = 0.0;
    last_progress_traveled_ = 0.0;
    start_time_ = std::chrono::steady_clock::now();
    last_progress_time_ = start_time_;
    status_ = nav_core::RecoveryStatus::RUNNING;
    RCLCPP_INFO(node_->get_logger(),
                "开始弧线后退恢复[%s]: dist=%.2fm v=%.2fm/s w=%.2frad/s timeout=%.1fs",
                name_.c_str(), backup_dist_, linear_vel_, angular_vel_, timeout_s_);
}

nav_core::RecoveryStatus ArcBackupRecovery::update(
    const geometry_msgs::msg::PoseStamped& current_pose) {
    if (status_ != nav_core::RecoveryStatus::RUNNING) {
        return status_;
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - start_time_).count();
    if (timeout_s_ > 0.0 && elapsed > timeout_s_) {
        geometry_msgs::msg::Twist stop;
        vel_pub_(stop);
        status_ = nav_core::RecoveryStatus::FAILED;
        RCLCPP_WARN(node_->get_logger(), "弧线后退恢复超时: %.2fs > %.2fs", elapsed,
                    timeout_s_);
        return status_;
    }

    const double dx = current_pose.pose.position.x - start_pose_.pose.position.x;
    const double dy = current_pose.pose.position.y - start_pose_.pose.position.y;
    traveled_ = std::hypot(dx, dy);

    if (min_progress_ > 0.0 && traveled_ - last_progress_traveled_ >= min_progress_) {
        last_progress_traveled_ = traveled_;
        last_progress_time_ = now;
    }

    const double no_progress_elapsed =
        std::chrono::duration<double>(now - last_progress_time_).count();
    if (progress_timeout_s_ > 0.0 && no_progress_elapsed > progress_timeout_s_) {
        geometry_msgs::msg::Twist stop;
        vel_pub_(stop);
        status_ = nav_core::RecoveryStatus::FAILED;
        RCLCPP_WARN(node_->get_logger(),
                    "弧线后退恢复无进展: %.2fs 内移动不足 %.3fm (traveled=%.3fm)",
                    no_progress_elapsed, min_progress_, traveled_);
        return status_;
    }

    if (traveled_ >= backup_dist_) {
        geometry_msgs::msg::Twist stop;
        vel_pub_(stop);
        status_ = nav_core::RecoveryStatus::SUCCEEDED;
        RCLCPP_INFO(node_->get_logger(), "弧线后退恢复完成[%s]", name_.c_str());
        return status_;
    }

    if (enable_safety_check_ && !allow_unsafe_ && safety_checker_) {
        std::string reason;
        if (!safety_checker_(current_pose, linear_vel_, angular_vel_,
                             safety_check_duration_s_, &reason)) {
            geometry_msgs::msg::Twist stop;
            vel_pub_(stop);
            status_ = nav_core::RecoveryStatus::FAILED;
            RCLCPP_WARN(node_->get_logger(), "弧线后退恢复安全检查失败: %s",
                        reason.empty() ? "弧线轨迹不可通行" : reason.c_str());
            return status_;
        }
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_vel_;
    cmd.angular.z = angular_vel_;
    vel_pub_(cmd);

    return status_;
}

void ArcBackupRecovery::cancel() {
    geometry_msgs::msg::Twist stop;
    vel_pub_(stop);
    status_ = nav_core::RecoveryStatus::IDLE;
}

}  // namespace nav_components
