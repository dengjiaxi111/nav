// nav_components/src/pure_pursuit.cpp
// Pure Pursuit 控制器实现

#include "nav_components/pure_pursuit.hpp"
#include <cmath>

namespace nav_components {

// 从四元数提取yaw角
static double getYaw(const geometry_msgs::msg::Quaternion& q) {
    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
}

void PurePursuit::initialize(rclcpp::Node* node) {
    node_ = node;
    lookahead_dist_ = node_->declare_parameter("controller.lookahead_dist", 0.5);
    max_linear_vel_ = node_->declare_parameter("controller.max_linear_vel", 0.5);
    max_angular_vel_ = node_->declare_parameter("controller.max_angular_vel", 1.0);
}

void PurePursuit::setPath(const nav_msgs::msg::Path& path) {
    path_ = path;
    current_idx_ = 0;
}

void PurePursuit::setTolerance(double xy_tol, double yaw_tol) {
    xy_tolerance_ = xy_tol;
    yaw_tolerance_ = yaw_tol;
}

void PurePursuit::reset() {
    path_.poses.clear();
    current_idx_ = 0;
}

double PurePursuit::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

int PurePursuit::findLookaheadPoint(const geometry_msgs::msg::Pose& pose) {
    double px = pose.position.x;
    double py = pose.position.y;
    
    // 从当前索引开始向前找
    for (size_t i = current_idx_; i < path_.poses.size(); i++) {
        double dx = path_.poses[i].pose.position.x - px;
        double dy = path_.poses[i].pose.position.y - py;
        double dist = std::hypot(dx, dy);
        
        if (dist >= lookahead_dist_) {
            return i;
        }
    }
    // 返回最后一个点
    return path_.poses.size() - 1;
}

nav_core::ControlResult PurePursuit::computeVelocity(
    const geometry_msgs::msg::PoseStamped& current_pose,
    geometry_msgs::msg::Twist& cmd_vel)
{
    if (path_.poses.empty()) {
        cmd_vel = geometry_msgs::msg::Twist();
        return nav_core::ControlResult::FAILED;
    }
    
    // 检查是否到达终点
    auto& goal = path_.poses.back().pose;
    double dx = goal.position.x - current_pose.pose.position.x;
    double dy = goal.position.y - current_pose.pose.position.y;
    double dist_to_goal = std::hypot(dx, dy);
    
    if (dist_to_goal < xy_tolerance_) {
        cmd_vel = geometry_msgs::msg::Twist();
        return nav_core::ControlResult::SUCCEEDED;
    }
    
    // 找前视点
    int lookahead_idx = findLookaheadPoint(current_pose.pose);
    current_idx_ = std::max(current_idx_, lookahead_idx - 5);  // 更新当前索引
    
    auto& target = path_.poses[lookahead_idx].pose;
    double tx = target.position.x - current_pose.pose.position.x;
    double ty = target.position.y - current_pose.pose.position.y;
    
    // 当前朝向
    double yaw = getYaw(current_pose.pose.orientation);
    
    // 目标角度
    double target_angle = std::atan2(ty, tx);
    double angle_error = normalizeAngle(target_angle - yaw);
    
    // Pure Pursuit 曲率计算
    double L = std::hypot(tx, ty);
    double curvature = 2.0 * std::sin(angle_error) / L;
    
    // 速度计算
    double linear_vel = max_linear_vel_;
    
    // 转弯时减速
    if (std::abs(angle_error) > 0.5) {
        linear_vel *= 0.5;
    }
    
    double angular_vel = linear_vel * curvature;
    angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
    
    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = angular_vel;
    
    return nav_core::ControlResult::RUNNING;
}

}  // namespace nav_components
