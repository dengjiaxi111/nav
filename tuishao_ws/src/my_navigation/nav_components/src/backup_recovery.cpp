// nav_components/src/backup_recovery.cpp

#include "nav_components/backup_recovery.hpp"
#include <cmath>
#include <utility>

namespace nav_components {

void BackupRecovery::initialize(rclcpp::Node* node, nav_core::VelPublisher vel_pub) {
    node_ = node;
    vel_pub_ = vel_pub;
    backup_dist_ = node_->declare_parameter("recovery.backup_dist", 0.3);
    backup_vel_ = node_->declare_parameter("recovery.backup_vel", -0.2);
    timeout_s_ = node_->declare_parameter("recovery.backup_timeout", 2.0);
    min_progress_ = node_->declare_parameter("recovery.min_progress", 0.02);
    progress_timeout_s_ = node_->declare_parameter("recovery.progress_timeout", 1.0);
    safety_check_distance_ = node_->declare_parameter("recovery.safety_check_distance", 0.5);
    enable_safety_check_ = node_->declare_parameter("recovery.enable_safety_check", true);
    allow_unsafe_ = node_->declare_parameter("recovery.backup_allow_unsafe", false);
}

void BackupRecovery::setSafetyChecker(SafetyChecker checker) {
    safety_checker_ = std::move(checker);
}

void BackupRecovery::start(const geometry_msgs::msg::PoseStamped& current_pose) {
    start_pose_ = current_pose;
    traveled_ = 0.0;
    last_progress_traveled_ = 0.0;
    start_time_ = std::chrono::steady_clock::now();
    last_progress_time_ = start_time_;
    status_ = nav_core::RecoveryStatus::RUNNING;
    RCLCPP_INFO(node_->get_logger(),
                "开始后退恢复: dist=%.2fm vel=%.2fm/s timeout=%.1fs",
                backup_dist_, backup_vel_, timeout_s_);
}

nav_core::RecoveryStatus BackupRecovery::update(const geometry_msgs::msg::PoseStamped& current_pose) {
    if (status_ != nav_core::RecoveryStatus::RUNNING) {
        return status_;
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - start_time_).count();
    if (timeout_s_ > 0.0 && elapsed > timeout_s_) {
        geometry_msgs::msg::Twist stop;
        vel_pub_(stop);
        status_ = nav_core::RecoveryStatus::FAILED;
        RCLCPP_WARN(node_->get_logger(), "后退恢复超时: %.2fs > %.2fs", elapsed, timeout_s_);
        return status_;
    }
    
    // 计算已移动距离
    double dx = current_pose.pose.position.x - start_pose_.pose.position.x;
    double dy = current_pose.pose.position.y - start_pose_.pose.position.y;
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
                    "后退恢复无进展: %.2fs 内移动不足 %.3fm (traveled=%.3fm)",
                    no_progress_elapsed, min_progress_, traveled_);
        return status_;
    }
    
    if (traveled_ >= backup_dist_) {
        // 停止
        geometry_msgs::msg::Twist stop;
        vel_pub_(stop);
        status_ = nav_core::RecoveryStatus::SUCCEEDED;
        RCLCPP_INFO(node_->get_logger(), "后退恢复完成");
        return status_;
    }

    if (enable_safety_check_ && !allow_unsafe_ && safety_checker_) {
        std::string reason;
        if (!safety_checker_(current_pose, safety_check_distance_, &reason)) {
            geometry_msgs::msg::Twist stop;
            vel_pub_(stop);
            status_ = nav_core::RecoveryStatus::FAILED;
            RCLCPP_WARN(node_->get_logger(),
                        "后退恢复安全检查失败: %s",
                        reason.empty() ? "后方不可通行" : reason.c_str());
            return status_;
        }
    }
    
    // 继续后退
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = backup_vel_;
    vel_pub_(cmd);
    
    return status_;
}

void BackupRecovery::cancel() {
    geometry_msgs::msg::Twist stop;
    vel_pub_(stop);
    status_ = nav_core::RecoveryStatus::IDLE;
}

}  // namespace nav_components
