// nav_core/include/nav_core/controller_base.hpp
// 控制器基类

#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nav_core {

// 控制器返回状态
enum class ControlResult {
    RUNNING,     // 正常跟踪
    SUCCEEDED,   // 到达目标
    FAILED       // 跟踪失败
};

class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    
    virtual void initialize(rclcpp::Node* node) = 0;
    
    // 设置要跟踪的路径
    virtual void setPath(const nav_msgs::msg::Path& path) = 0;
    
    // 计算速度指令
    virtual ControlResult computeVelocity(
        const geometry_msgs::msg::PoseStamped& current_pose,
        geometry_msgs::msg::Twist& cmd_vel) = 0;
    
    // 设置目标容差
    virtual void setTolerance(double xy_tol, double yaw_tol) = 0;
    
    // 重置状态
    virtual void reset() = 0;
};

}  // namespace nav_core
