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
  
  // 2D地图投影配置（基于中科大的高程分析方案）
  map_2d_projector::ProjectorConfig projector_cfg;
  projector_cfg.robot_height = node->declare_parameter("projector.robot_height", 0.55f);
  projector_cfg.base_to_ground_default = node->declare_parameter("projector.base_to_ground", 0.10f);
  projector_cfg.ground_tolerance = node->declare_parameter("projector.ground_tolerance", 0.03f);
  
  // 动态腿长配置
  projector_cfg.enable_dynamic_leg_length = node->declare_parameter("projector.enable_dynamic_leg_length", false);
  projector_cfg.wheel_frame = node->declare_parameter("projector.wheel_frame", std::string("wheel_link"));
  projector_cfg.leg_length_min = node->declare_parameter("projector.leg_length_min", 0.08f);
  projector_cfg.leg_length_max = node->declare_parameter("projector.leg_length_max", 0.25f);
  
  // 高程分析阈值
  projector_cfg.slope_height_max = node->declare_parameter("projector.slope_height_max", 0.08f);
  projector_cfg.step_height_min = node->declare_parameter("projector.step_height_min", 0.12f);
  projector_cfg.step_height_max = node->declare_parameter("projector.step_height_max", 0.23f);
  projector_cfg.obstacle_height_min = node->declare_parameter("projector.obstacle_height_min", 0.25f);
  projector_cfg.high_occupancy_thresh = node->declare_parameter("projector.high_occupancy_thresh", 0.5f);
  
  // 法向量分析参数
  projector_cfg.normal_z_slope_thresh = node->declare_parameter("projector.normal_z_slope_thresh", 0.866f);
  projector_cfg.normal_z_wall_thresh = node->declare_parameter("projector.normal_z_wall_thresh", 0.5f);
  projector_cfg.normal_min_points = node->declare_parameter("projector.normal_min_points", 5);
  projector_cfg.planarity_thresh = node->declare_parameter("projector.planarity_thresh", 0.7f);
  
  // 台阶规格参数
  projector_cfg.step_15cm_min = node->declare_parameter("projector.step_15cm_min", 0.12f);
  projector_cfg.step_15cm_max = node->declare_parameter("projector.step_15cm_max", 0.18f);
  projector_cfg.step_20cm_min = node->declare_parameter("projector.step_20cm_min", 0.17f);
  projector_cfg.step_20cm_max = node->declare_parameter("projector.step_20cm_max", 0.23f);
  projector_cfg.step_continuity_thresh = node->declare_parameter("projector.step_continuity_thresh", 0.03f);
  projector_cfg.step_neighbor_min_count = node->declare_parameter("projector.step_neighbor_min_count", 2);
  
  // 悬崖/下行台阶检测参数
  projector_cfg.cliff_search_radius = node->declare_parameter("projector.cliff_search_radius", 5);
  projector_cfg.cliff_min_drop = node->declare_parameter("projector.cliff_min_drop", 0.10f);
  projector_cfg.cliff_min_lower_points = node->declare_parameter("projector.cliff_min_lower_points", 3);
  
  // 坡面检测参数（梯度连续性分析）
  projector_cfg.slope_gradient_thresh = node->declare_parameter("projector.slope_gradient_thresh", 0.577f);
  projector_cfg.slope_continuity_thresh = node->declare_parameter("projector.slope_continuity_thresh", 0.03f);
  projector_cfg.slope_min_continuous_count = node->declare_parameter("projector.slope_min_continuous_count", 3);
  
  // 地图范围和分辨率
  projector_cfg.map_range_x = node->declare_parameter("projector.map_range_x", 10.0f);
  projector_cfg.map_range_y = node->declare_parameter("projector.map_range_y", 10.0f);
  projector_cfg.resolution = node->declare_parameter("projector.resolution", 0.025f);
  projector_cfg.z_min_relative = node->declare_parameter("projector.z_min_relative", -0.5f);
  projector_cfg.z_max_relative = node->declare_parameter("projector.z_max_relative", 2.0f);
  projector_cfg.publish_rate = node->declare_parameter("projector.publish_rate", 20.0f);
  projector_cfg.frame_id = node->declare_parameter("projector.frame_id", std::string("odom"));
  projector_cfg.topic_name = node->declare_parameter("projector.topic_name", std::string("/rog_map/map_2d"));
  projector_cfg.enable_debug_log = node->declare_parameter("projector.enable_debug_log", false);
  projector_cfg.enable_step_debug_viz = node->declare_parameter("projector.enable_step_debug_viz", true);
  projector_cfg.step_debug_topic = node->declare_parameter("projector.step_debug_topic", std::string("rog_map/step_debug"));
  
  // 创建2D地图投影器
  auto map_projector = std::make_unique<map_2d_projector::Map2DProjector>(node, projector_cfg);
  
  RCLCPP_INFO(node->get_logger(), "ROG-Map ROS2 node started with MultiThreadedExecutor");
  RCLCPP_INFO(node->get_logger(), "  - 3D occupancy mapping: enabled");
  RCLCPP_INFO(node->get_logger(), "  - 2D map projection: H_slope<%.2f, %.2f<H_step<%.2f",
              projector_cfg.slope_height_max, projector_cfg.step_height_min, 
              projector_cfg.step_height_max);
  RCLCPP_INFO(node->get_logger(), "  - Normal analysis: |nz|>%.2f→slope, |nz|<%.2f→wall",
              projector_cfg.normal_z_slope_thresh, projector_cfg.normal_z_wall_thresh);
  
  // Use MultiThreadedExecutor to allow concurrent callback execution
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  
  return 0;
}
