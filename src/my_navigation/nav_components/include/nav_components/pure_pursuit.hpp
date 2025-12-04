// nav_components/include/nav_components/pure_pursuit.hpp
// Pure Pursuit 路径跟踪控制器

#pragma once
#include <nav_core/controller_base.hpp>

namespace nav_components {

class PurePursuit : public nav_core::ControllerBase {
public:
    void initialize(rclcpp::Node* node) override;
    void setPath(const nav_msgs::msg::Path& path) override;
    nav_core::ControlResult computeVelocity(
        const geometry_msgs::msg::PoseStamped& current_pose,
        geometry_msgs::msg::Twist& cmd_vel) override;
    void setTolerance(double xy_tol, double yaw_tol) override;
    void reset() override;

private:
    // 查找前视点
    int findLookaheadPoint(const geometry_msgs::msg::Pose& pose);
    double normalizeAngle(double angle);
    
    rclcpp::Node* node_ = nullptr;
    nav_msgs::msg::Path path_;
    
    // 参数
    double lookahead_dist_ = 0.5;    // 前视距离
    double max_linear_vel_ = 0.5;    // 最大线速度
    double max_angular_vel_ = 1.0;   // 最大角速度
    double xy_tolerance_ = 0.1;
    double yaw_tolerance_ = 0.1;
    
    int current_idx_ = 0;  // 当前跟踪的路径点索引
};

}  // namespace nav_components
