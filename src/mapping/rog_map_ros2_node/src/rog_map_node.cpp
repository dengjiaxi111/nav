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
    return 1;
  }
  
  // Instantiate ROGMapROS which handles all ROS2 integration internally
  auto rog_map_ros = std::make_unique<rog_map::ROGMapROS>(node, config_file);
  
  RCLCPP_INFO(node->get_logger(), "ROG-Map ROS2 node started");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
