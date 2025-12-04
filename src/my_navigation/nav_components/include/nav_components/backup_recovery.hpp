// nav_components/include/nav_components/backup_recovery.hpp
// 后退恢复行为

#pragma once
#include <nav_core/recovery_base.hpp>

namespace nav_components {

class BackupRecovery : public nav_core::RecoveryBase {
public:
    void initialize(rclcpp::Node* node, nav_core::VelPublisher vel_pub) override;
    void start(const geometry_msgs::msg::PoseStamped& current_pose) override;
    nav_core::RecoveryStatus update(const geometry_msgs::msg::PoseStamped& current_pose) override;
    void cancel() override;
    const char* name() const override { return "backup"; }

private:
    rclcpp::Node* node_ = nullptr;
    nav_core::VelPublisher vel_pub_;
    nav_core::RecoveryStatus status_ = nav_core::RecoveryStatus::IDLE;
    
    geometry_msgs::msg::PoseStamped start_pose_;
    double backup_dist_ = 0.3;     // 后退距离
    double backup_vel_ = -0.2;     // 后退速度
    double traveled_ = 0.0;
};

}  // namespace nav_components
