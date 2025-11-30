/**
 * @file rog_map_component.hpp
 * @brief ROG-Map ROS2 Component 包装器
 * 
 * 将 ROGMapROS 封装为可组合节点，支持：
 * 1. 进程内零拷贝通信 (intra-process communication)
 * 2. 与其他 component 在同一进程中运行
 * 3. 动态加载/卸载
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

// 前向声明，避免在头文件中包含重型头文件
namespace rog_map {
    class ROGMapROS;
}

namespace rog_map_ros2_node {

/**
 * @brief ROG-Map Component 节点
 * 
 * 这是一个轻量级包装器，将 ROGMapROS 封装为 ROS2 Component。
 * ROGMapROS 本身会创建所有的 publisher/subscriber，
 * 这个类只负责：
 *   1. 作为 rclcpp::Node 被 component container 管理
 *   2. 传递 shared_from_this() 给 ROGMapROS
 *   3. 管理 ROGMapROS 的生命周期
 */
class ROGMapComponent : public rclcpp::Node {
public:
    explicit ROGMapComponent(const rclcpp::NodeOptions& options);
    ~ROGMapComponent() override;

private:
    std::unique_ptr<rog_map::ROGMapROS> rog_map_ros_;
};

} // namespace rog_map_ros2_node
