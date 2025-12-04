// nav_components/src/backup_recovery.cpp

#include "nav_components/backup_recovery.hpp"
#include <cmath>

namespace nav_components {

void BackupRecovery::initialize(rclcpp::Node* node, nav_core::VelPublisher vel_pub) {
    node_ = node;
    vel_pub_ = vel_pub;
    backup_dist_ = node_->declare_parameter("recovery.backup_dist", 0.3);
    backup_vel_ = node_->declare_parameter("recovery.backup_vel", -0.2);
}

void BackupRecovery::start(const geometry_msgs::msg::PoseStamped& current_pose) {
    start_pose_ = current_pose;
    traveled_ = 0.0;
    status_ = nav_core::RecoveryStatus::RUNNING;
    RCLCPP_INFO(node_->get_logger(), "开始后退恢复");
}

nav_core::RecoveryStatus BackupRecovery::update(const geometry_msgs::msg::PoseStamped& current_pose) {
    if (status_ != nav_core::RecoveryStatus::RUNNING) {
        return status_;
    }
    
    // 计算已移动距离
    double dx = current_pose.pose.position.x - start_pose_.pose.position.x;
    double dy = current_pose.pose.position.y - start_pose_.pose.position.y;
    traveled_ = std::hypot(dx, dy);
    
    if (traveled_ >= backup_dist_) {
        // 停止
        geometry_msgs::msg::Twist stop;
        vel_pub_(stop);
        status_ = nav_core::RecoveryStatus::SUCCEEDED;
        RCLCPP_INFO(node_->get_logger(), "后退恢复完成");
        return status_;
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
