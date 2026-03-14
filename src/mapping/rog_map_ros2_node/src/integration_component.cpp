/**
 * @file integration_component.cpp
 * @brief 集成组件实现 - 使用 executor 管理所有子节点
 */

#include "rog_map_ros2_node/integration_component.hpp"
#include "rog_map_ros2_node/rog_map_component.hpp"
#include "rog_map_ros2_node/map_2d_projector.hpp"
#include "rog_map_ros2_node/stair_detector.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

namespace rog_map_ros2_node {

IntegrationComponent::IntegrationComponent(const rclcpp::NodeOptions& options)
    : Node("integration_component", options) {
    
    RCLCPP_INFO(this->get_logger(), "=== Integration Component Initializing ===");
    
    // 创建多线程 executor（在构造函数中初始化，在 spin 线程中运行）
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
    // Step 1: 创建 ROGMapComponent（核心地图服务）
    rclcpp::NodeOptions rog_map_options;
    rog_map_options.use_intra_process_comms(true);  // 启用进程内通信
    
    // 从当前节点复制参数到子节点
    rog_map_options.append_parameter_override("config_file", 
        this->declare_parameter<std::string>("config_file", ""));
    
    rog_map_component_ = std::make_shared<ROGMapComponent>(rog_map_options);
    executor_->add_node(rog_map_component_);
    
    RCLCPP_INFO(this->get_logger(), "  ROGMapComponent created");
    // 等待 ROG-Map 初始化完成
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // 获取 ROGMapROS 指针
    rog_map::ROGMapROS* rog_map_ptr = rog_map_component_->getRogMapMutable();
    if (rog_map_ptr == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ROGMapROS pointer!");
        throw std::runtime_error("ROGMapComponent initialization failed");
    }
    
    RCLCPP_INFO(this->get_logger(), "  ROGMapROS pointer obtained: %p", 
                static_cast<void*>(rog_map_ptr));
    
    // Step 2: 创建 Map2DProjector（依赖注入 ROGMapROS*）
    rclcpp::NodeOptions projector_options;
    projector_options.use_intra_process_comms(true);
    
    // 直接从父节点声明并获取所有 projector 参数
    // 参数格式: projector.robot_height, projector.base_to_ground_default 等
    std::vector<std::string> projector_param_names = {
        "robot_height", "robot_width", "base_to_ground_default", "ground_tolerance",
        "enable_dynamic_leg_length", "wheel_frame", "leg_length_min", "leg_length_max",
        "slope_height_max", "step_height_min", "step_height_max", "obstacle_height_min",
        "high_occupancy_thresh", "normal_z_slope_thresh", "normal_z_wall_thresh",
        "normal_min_points", "planarity_thresh",
        "step_15cm_min", "step_15cm_max", "step_20cm_min", "step_20cm_max",
        "step_continuity_thresh", "step_neighbor_min_count", "step_edge_ratio_thresh",
        "cliff_search_radius", "cliff_min_drop", "cliff_min_lower_points",
        "slope_gradient_thresh", "slope_continuity_thresh", "slope_min_continuous_count",
        "map_range_x", "map_range_y", "resolution", "z_min_relative", "z_max_relative",
        "publish_rate", "frame_id", "topic_name",
        "enable_debug_log", "enable_step_debug_viz", "step_debug_topic"
    };
    
    // 从 launch 文件的 parameter overrides 中加载参数
    RCLCPP_INFO(this->get_logger(), "Loading Map2DProjector parameters:");
    
    // 获取所有以 "projector." 开头的参数
    auto param_overrides = this->get_node_parameters_interface()->get_parameter_overrides();
    
    for (const auto& param_name : projector_param_names) {
        std::string full_name = "projector." + param_name;
        
        // 检查参数是否在 overrides 中（即从 launch 文件传入）
        auto it = param_overrides.find(full_name);
        if (it != param_overrides.end()) {
            // 参数存在于 launch 文件中，传递给子节点
            projector_options.append_parameter_override(param_name, it->second);
            RCLCPP_INFO(this->get_logger(), "  ✓ %s = %s", 
                        full_name.c_str(), rclcpp::Parameter(full_name, it->second).value_to_string().c_str());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "  ○ %s not set (will use default)", full_name.c_str());
        }
    }
    
    map_2d_projector_ = std::make_shared<map_2d_projector::Map2DProjector>(
        projector_options, rog_map_ptr);
    executor_->add_node(map_2d_projector_);
    
    RCLCPP_INFO(this->get_logger(), "  Map2DProjector created");
    
    // Step 3: 创建 StairDetector（依赖注入 ROGMapROS*）
    rclcpp::NodeOptions detector_options;
    detector_options.use_intra_process_comms(true);
    
    // 直接声明并获取所有 stair_detector 参数
    std::vector<std::string> detector_param_names = {
        "roi_x_min", "roi_x_max", "roi_y_min", "roi_y_max", "roi_z_min", "roi_z_max",
        "cluster_tolerance", "min_cluster_size", "max_cluster_size",
        "single_stair_height", "double_stair_height", "height_tolerance",
        "min_stair_width", "min_stair_depth", "max_z_thickness",
        "cell_size_xy", "min_cell_points", "min_cell_height", "max_cell_height", "max_cell_top_z",
        "min_detection_frames", "lowpass_alpha", "blind_zone_distance",
        "input_cloud_topic", "output_target_topic", "output_marker_topic",
        "target_frame", "map_frame", "enable_visualization", "update_rate"
    };
    
    // 从 launch 文件的 parameter overrides 中加载参数
    RCLCPP_INFO(this->get_logger(), "Loading StairDetector parameters:");
    
    for (const auto& param_name : detector_param_names) {
        std::string full_name = "stair_detector." + param_name;
        
        // 检查参数是否在 overrides 中
        auto it = param_overrides.find(full_name);
        if (it != param_overrides.end()) {
            // 参数存在，传递给子节点（去掉 stair_detector. 前缀）
            detector_options.append_parameter_override(param_name, it->second);
            RCLCPP_INFO(this->get_logger(), "  ✓ %s = %s", 
                        full_name.c_str(), rclcpp::Parameter(full_name, it->second).value_to_string().c_str());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "  ○ %s not set (will use default)", full_name.c_str());
        }
    }
    
    stair_detector_ = std::make_shared<stair_detector::StairDetector>(
        detector_options, rog_map_ptr);
    executor_->add_node(stair_detector_);
    
    RCLCPP_INFO(this->get_logger(), "  StairDetector created");
    
    // Step 4: 启动 executor 线程
    executor_thread_ = std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "  Executor thread started");
        executor_->spin();
        RCLCPP_INFO(this->get_logger(), "  Executor thread stopped");
    });
    
    RCLCPP_INFO(this->get_logger(), "=== Integration Component Initialized Successfully ===");
    RCLCPP_INFO(this->get_logger(), "  Architecture: Component-based with direct method calls");
    RCLCPP_INFO(this->get_logger(), "  Performance: Zero-copy intra-process communication");
    RCLCPP_INFO(this->get_logger(), "  Executor: MultiThreadedExecutor running in separate thread");
}

IntegrationComponent::~IntegrationComponent() {
    RCLCPP_INFO(this->get_logger(), "Integration Component shutting down...");
    
    if (executor_) {
        executor_->cancel();
    }
    
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
    
    RCLCPP_INFO(this->get_logger(), "Integration Component destroyed");
}

} // namespace rog_map_ros2_node

// 注册为 ROS2 组件
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rog_map_ros2_node::IntegrationComponent)
