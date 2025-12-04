// nav_core/include/nav_core/recovery_base.hpp
// 恢复行为基类

#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include "nav_core/types.hpp"

namespace nav_core {

// 速度发布器类型
using VelPublisher = std::function<void(const geometry_msgs::msg::Twist&)>;

class RecoveryBase {
public:
    virtual ~RecoveryBase() = default;
    
    virtual void initialize(rclcpp::Node* node, VelPublisher vel_pub) = 0;
    
    // 开始恢复行为
    virtual void start(const geometry_msgs::msg::PoseStamped& current_pose) = 0;
    
    // 周期更新，返回当前状态
    virtual RecoveryStatus update(const geometry_msgs::msg::PoseStamped& current_pose) = 0;
    
    // 取消
    virtual void cancel() = 0;
    
    // 获取名称
    virtual const char* name() const = 0;
};

}  // namespace nav_core
