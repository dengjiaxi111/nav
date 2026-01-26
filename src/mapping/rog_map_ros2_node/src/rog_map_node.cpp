#define USE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>
#include "rog_map_ros2_node/map_2d_projector.hpp"

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
  
  // 2D地图投影配置（基于中科大爷的高程分析方案）
  map_2d_projector::ProjectorConfig projector_cfg;
  projector_cfg.robot_height = node->declare_parameter("projector.robot_height", 0.55f);
  projector_cfg.ground_clearance = node->declare_parameter("projector.ground_clearance", 0.03f);
  
  // 高程分析阈值
  projector_cfg.slope_height_max = node->declare_parameter("projector.slope_height_max", 0.10f);
  projector_cfg.step_height_min = node->declare_parameter("projector.step_height_min", 0.10f);
  projector_cfg.step_height_max = node->declare_parameter("projector.step_height_max", 0.20f);
  projector_cfg.obstacle_height_min = node->declare_parameter("projector.obstacle_height_min", 0.30f);
  projector_cfg.high_occupancy_thresh = node->declare_parameter("projector.high_occupancy_thresh", 0.5f);
  
  projector_cfg.map_range_x = node->declare_parameter("projector.map_range_x", 10.0f);
  projector_cfg.map_range_y = node->declare_parameter("projector.map_range_y", 10.0f);
  projector_cfg.resolution = node->declare_parameter("projector.resolution", 0.025f);
  projector_cfg.z_min_relative = node->declare_parameter("projector.z_min_relative", -0.5f);
  projector_cfg.z_max_relative = node->declare_parameter("projector.z_max_relative", 2.0f);
  projector_cfg.publish_rate = node->declare_parameter("projector.publish_rate", 20.0f);
  projector_cfg.frame_id = node->declare_parameter("projector.frame_id", std::string("odom"));
  projector_cfg.topic_name = node->declare_parameter("projector.topic_name", std::string("/rog_map/map_2d"));
  
  // 创建2D地图投影器
  auto map_projector = std::make_unique<map_2d_projector::Map2DProjector>(node, projector_cfg);
  
  RCLCPP_INFO(node->get_logger(), "ROG-Map ROS2 node started with MultiThreadedExecutor");
  RCLCPP_INFO(node->get_logger(), "  - 3D occupancy mapping: enabled");
  RCLCPP_INFO(node->get_logger(), "  - 2D map projection: H_slope<%.2f, %.2f<H_step<%.2f, H_obs>%.2f",
              projector_cfg.slope_height_max, projector_cfg.step_height_min, 
              projector_cfg.step_height_max, projector_cfg.obstacle_height_min);
  
  // Use MultiThreadedExecutor to allow concurrent callback execution
  // This prevents long-running updateCallback from blocking vizCallback
  // Each callback group (update_cbk_group, viz_reen_cbk_group) runs in separate threads
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  
  return 0;
}
