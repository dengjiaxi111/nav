/**
 * @file rog_map_node.cpp
 * @brief ROG-Map standalone node (仅3D建图，不含2D投影和台阶检测)
 * 
 * 使用方式：
 * - 仅需要3D建图: ros2 run rog_map_ros2_node rog_map_node --ros-args -p config_file:=xxx.yaml
 * - 完整功能（含2D投影+台阶检测）: ros2 run rog_map_ros2_node integration_node
 */

#define USE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("rog_map_node");
    
    // Get config file path from parameter
    node->declare_parameter("config_file", "");
    std::string config_file = node->get_parameter("config_file").as_string();
    
    if (config_file.empty()) {
        RCLCPP_ERROR(node->get_logger(), "config_file parameter not provided");
        RCLCPP_ERROR(node->get_logger(), "Usage: ros2 run rog_map_ros2_node rog_map_node --ros-args -p config_file:=/path/to/config.yaml");
        return 1;
    }
    
    // Instantiate ROGMapROS which handles all ROS2 integration internally
    auto rog_map_ros = std::make_unique<rog_map::ROGMapROS>(node, config_file);
    
    RCLCPP_INFO(node->get_logger(), "========================================");
    RCLCPP_INFO(node->get_logger(), "ROG-Map Standalone Node Started");
    RCLCPP_INFO(node->get_logger(), "========================================");
    RCLCPP_INFO(node->get_logger(), "  Mode: 3D Occupancy Mapping ONLY");
    RCLCPP_INFO(node->get_logger(), "  Config: %s", config_file.c_str());
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "  Note: For full navigation features (2D projection + stair detection),");
    RCLCPP_INFO(node->get_logger(), "        use: ros2 run rog_map_ros2_node integration_node");
    RCLCPP_INFO(node->get_logger(), "========================================");
    
    // Use MultiThreadedExecutor to allow concurrent callback execution
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    
    return 0;
}

