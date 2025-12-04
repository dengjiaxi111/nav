// nav_components/src/spin_recovery.cpp

#include "nav_components/spin_recovery.hpp"
#include <cmath>

namespace nav_components {

double SpinRecovery::getYaw(const geometry_msgs::msg::Quaternion& q) {
    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
}

double SpinRecovery::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void SpinRecovery::initialize(rclcpp::Node* node, nav_core::VelPublisher vel_pub) {
    node_ = node;
    vel_pub_ = vel_pub;
    spin_angle_ = node_->declare_parameter("recovery.spin_angle", M_PI);
    spin_vel_ = node_->declare_parameter("recovery.spin_vel", 0.5);
}

void SpinRecovery::start(const geometry_msgs::msg::PoseStamped& current_pose) {
    start_yaw_ = getYaw(current_pose.pose.orientation);
    last_yaw_ = start_yaw_;
    rotated_ = 0.0;
    status_ = nav_core::RecoveryStatus::RUNNING;
    RCLCPP_INFO(node_->get_logger(), "开始旋转恢复");
}

nav_core::RecoveryStatus SpinRecovery::update(const geometry_msgs::msg::PoseStamped& current_pose) {
    if (status_ != nav_core::RecoveryStatus::RUNNING) {
        return status_;
    }
    
    double current_yaw = getYaw(current_pose.pose.orientation);
    double delta = normalizeAngle(current_yaw - last_yaw_);
    rotated_ += std::abs(delta);
    last_yaw_ = current_yaw;
    
    if (rotated_ >= spin_angle_) {
        geometry_msgs::msg::Twist stop;
        vel_pub_(stop);
        status_ = nav_core::RecoveryStatus::SUCCEEDED;
        RCLCPP_INFO(node_->get_logger(), "旋转恢复完成");
        return status_;
    }
    
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = spin_vel_;
    vel_pub_(cmd);
    
    return status_;
}

void SpinRecovery::cancel() {
    geometry_msgs::msg::Twist stop;
    vel_pub_(stop);
    status_ = nav_core::RecoveryStatus::IDLE;
}

}  // namespace nav_components
