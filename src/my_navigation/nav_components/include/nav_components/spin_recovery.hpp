// nav_components/include/nav_components/spin_recovery.hpp
// 原地旋转恢复行为

#pragma once
#include <nav_core/recovery_base.hpp>
#include <chrono>

namespace nav_components {

class SpinRecovery : public nav_core::RecoveryBase {
public:
    void initialize(rclcpp::Node* node, nav_core::VelPublisher vel_pub) override;
    void start(const geometry_msgs::msg::PoseStamped& current_pose) override;
    nav_core::RecoveryStatus update(const geometry_msgs::msg::PoseStamped& current_pose) override;
    void cancel() override;
    const char* name() const override { return "spin"; }

private:
    double getYaw(const geometry_msgs::msg::Quaternion& q);
    double normalizeAngle(double angle);
    
    rclcpp::Node* node_ = nullptr;
    nav_core::VelPublisher vel_pub_;
    nav_core::RecoveryStatus status_ = nav_core::RecoveryStatus::IDLE;
    
    double start_yaw_ = 0.0;
    double spin_angle_ = M_PI;       // 旋转角度(rad)
    double spin_vel_ = 0.5;          // 旋转速度
    double rotated_ = 0.0;
    double last_yaw_ = 0.0;
    double timeout_s_ = 5.0;
    double min_progress_ = 0.03;
    double progress_timeout_s_ = 1.5;
    double last_progress_rotated_ = 0.0;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_progress_time_;
};

}  // namespace nav_components
