/**
 * @file rog_map_component.cpp
 * @brief ROG-Map ROS2 Component 实现
 */

// USE_ROS2 已在 CMakeLists.txt 中通过 target_compile_definitions 定义
#include "rog_map_ros2_node/rog_map_component.hpp"
#include <rog_map_ros/rog_map_ros2.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace rog_map_ros2_node {

ROGMapComponent::ROGMapComponent(const rclcpp::NodeOptions& options)
    : Node("rog_map", options) {
    
    // 声明并获取配置文件路径参数
    declare_parameter("config_file", "");
    std::string config_file = get_parameter("config_file").as_string();
    
    if (config_file.empty()) {
        RCLCPP_ERROR(get_logger(), "config_file parameter not provided!");
        throw std::runtime_error("config_file parameter is required");
    }
    
    RCLCPP_INFO(get_logger(), "Loading ROG-Map config from: %s", config_file.c_str());
    
    // 创建 ROGMapROS 实例，传入当前节点的 shared_ptr
    // 注意：需要使用 shared_from_this()，但在构造函数中不能直接调用
    // 所以我们使用一个延迟初始化的方式
    auto self = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    
    try {
        rog_map_ros_ = std::make_unique<rog_map::ROGMapROS>(self, config_file);
        RCLCPP_INFO(get_logger(), "ROG-Map Component initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize ROGMapROS: %s", e.what());
        throw;
    }
}

ROGMapComponent::~ROGMapComponent() {
    RCLCPP_INFO(get_logger(), "ROG-Map Component shutting down");
}

} // namespace rog_map_ros2_node

// 注册为可组合节点
RCLCPP_COMPONENTS_REGISTER_NODE(rog_map_ros2_node::ROGMapComponent)
