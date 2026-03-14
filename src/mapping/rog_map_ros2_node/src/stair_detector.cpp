/**
 * @file stair_detector.cpp
 * @brief 台阶检测节点实现（组件模式 + 直接调用）
 * 
 * **直接调用模式**: 通过 rog_map_ptr_ 直接访问 ROGMapROS boxSearchInflate()
 */

#include "rog_map_ros2_node/stair_detector.hpp"
#include <rog_map_ros/rog_map_ros2.hpp>  // 包含 ROGMapROS 定义
#include <pcl/kdtree/kdtree.h>
#include <limits>
#include <array>
#include <unordered_map>
#include <set>
#include <cmath>
#include <numeric>
#include <sstream>
#include <iomanip>

namespace stair_detector {

using namespace super_utils;  // 访问 vec_E 和其他工具

StairDetector::StairDetector(const rclcpp::NodeOptions& options,
                             rog_map::ROGMapROS* rog_map_ptr)
    : Node("stair_detector", options),
      rog_map_ptr_(rog_map_ptr),
      current_state_(DetectionState::IDLE),
      consecutive_detection_count_(0),
      in_blind_zone_(false) {
    
    // 声明并加载参数
    declare_parameter("roi_x_min", cfg_.roi_x_min);
    declare_parameter("roi_x_max", cfg_.roi_x_max);
    declare_parameter("roi_y_min", cfg_.roi_y_min);
    declare_parameter("roi_y_max", cfg_.roi_y_max);
    declare_parameter("roi_z_min", cfg_.roi_z_min);
    declare_parameter("roi_z_max", cfg_.roi_z_max);
    
    declare_parameter("cluster_tolerance", cfg_.cluster_tolerance);
    declare_parameter("min_cluster_size", cfg_.min_cluster_size);
    declare_parameter("max_cluster_size", cfg_.max_cluster_size);
    
    declare_parameter("single_stair_height", cfg_.single_stair_height);
    declare_parameter("double_stair_height", cfg_.double_stair_height);
    declare_parameter("height_tolerance", cfg_.height_tolerance);
    declare_parameter("min_stair_width", cfg_.min_stair_width);
    declare_parameter("min_stair_depth", cfg_.min_stair_depth);
    declare_parameter("max_z_thickness", cfg_.max_z_thickness);

    // 法向量估计参数
    declare_parameter("enable_normal_estimation", cfg_.enable_normal_estimation);
    declare_parameter("min_planarity", cfg_.min_planarity);
    declare_parameter("horizontal_normal_z_min", cfg_.horizontal_normal_z_min);
    declare_parameter("horizontal_points_ratio_min", cfg_.horizontal_points_ratio_min);
    declare_parameter("normal_min_points", cfg_.normal_min_points);

    // 多平面分割参数
    declare_parameter("enable_plane_segmentation", cfg_.enable_plane_segmentation);
    declare_parameter("ransac_distance_threshold", cfg_.ransac_distance_threshold);
    declare_parameter("ransac_max_iterations", cfg_.ransac_max_iterations);
    declare_parameter("min_plane_points", cfg_.min_plane_points);
    declare_parameter("ground_plane_z_tolerance", cfg_.ground_plane_z_tolerance);
    declare_parameter("max_planes", cfg_.max_planes);

    declare_parameter("cell_size_xy", cfg_.cell_size_xy);
    declare_parameter("min_cell_points", cfg_.min_cell_points);
    declare_parameter("min_cell_height", cfg_.min_cell_height);
    declare_parameter("max_cell_height", cfg_.max_cell_height);
    declare_parameter("max_cell_top_z", cfg_.max_cell_top_z);
    
    declare_parameter("min_detection_frames", cfg_.min_detection_frames);
    declare_parameter("lowpass_alpha", cfg_.lowpass_alpha);
    declare_parameter("blind_zone_distance", cfg_.blind_zone_distance);
    
    declare_parameter("input_cloud_topic", cfg_.input_cloud_topic);
    declare_parameter("output_target_topic", cfg_.output_target_topic);
    declare_parameter("output_marker_topic", cfg_.output_marker_topic);
    declare_parameter("target_frame", cfg_.target_frame);
    declare_parameter("map_frame", cfg_.map_frame);
    declare_parameter("enable_visualization", cfg_.enable_visualization);
    declare_parameter("update_rate", cfg_.update_rate);
    
    cfg_.roi_x_min = get_parameter("roi_x_min").as_double();
    cfg_.roi_x_max = get_parameter("roi_x_max").as_double();
    cfg_.roi_y_min = get_parameter("roi_y_min").as_double();
    cfg_.roi_y_max = get_parameter("roi_y_max").as_double();
    cfg_.roi_z_min = get_parameter("roi_z_min").as_double();
    cfg_.roi_z_max = get_parameter("roi_z_max").as_double();
    
    cfg_.cluster_tolerance = get_parameter("cluster_tolerance").as_double();
    cfg_.min_cluster_size = get_parameter("min_cluster_size").as_int();
    cfg_.max_cluster_size = get_parameter("max_cluster_size").as_int();
    
    cfg_.single_stair_height = get_parameter("single_stair_height").as_double();
    cfg_.double_stair_height = get_parameter("double_stair_height").as_double();
    cfg_.height_tolerance = get_parameter("height_tolerance").as_double();
    cfg_.min_stair_width = get_parameter("min_stair_width").as_double();
    cfg_.min_stair_depth = get_parameter("min_stair_depth").as_double();
    cfg_.max_z_thickness = get_parameter("max_z_thickness").as_double();

    cfg_.enable_normal_estimation = get_parameter("enable_normal_estimation").as_bool();
    cfg_.min_planarity = get_parameter("min_planarity").as_double();
    cfg_.horizontal_normal_z_min = get_parameter("horizontal_normal_z_min").as_double();
    cfg_.horizontal_points_ratio_min = get_parameter("horizontal_points_ratio_min").as_double();
    cfg_.normal_min_points = get_parameter("normal_min_points").as_int();

    cfg_.enable_plane_segmentation = get_parameter("enable_plane_segmentation").as_bool();
    cfg_.ransac_distance_threshold = get_parameter("ransac_distance_threshold").as_double();
    cfg_.ransac_max_iterations = get_parameter("ransac_max_iterations").as_int();
    cfg_.min_plane_points = get_parameter("min_plane_points").as_int();
    cfg_.ground_plane_z_tolerance = get_parameter("ground_plane_z_tolerance").as_double();
    cfg_.max_planes = get_parameter("max_planes").as_int();

    cfg_.cell_size_xy = get_parameter("cell_size_xy").as_double();
    cfg_.min_cell_points = get_parameter("min_cell_points").as_int();
    cfg_.min_cell_height = get_parameter("min_cell_height").as_double();
    cfg_.max_cell_height = get_parameter("max_cell_height").as_double();
    cfg_.max_cell_top_z = get_parameter("max_cell_top_z").as_double();
    
    cfg_.min_detection_frames = get_parameter("min_detection_frames").as_int();
    cfg_.lowpass_alpha = get_parameter("lowpass_alpha").as_double();
    cfg_.blind_zone_distance = get_parameter("blind_zone_distance").as_double();
    
    cfg_.input_cloud_topic = get_parameter("input_cloud_topic").as_string();
    cfg_.output_target_topic = get_parameter("output_target_topic").as_string();
    cfg_.output_marker_topic = get_parameter("output_marker_topic").as_string();
    cfg_.target_frame = get_parameter("target_frame").as_string();
    cfg_.map_frame = get_parameter("map_frame").as_string();
    cfg_.enable_visualization = get_parameter("enable_visualization").as_bool();
    cfg_.update_rate = get_parameter("update_rate").as_double();
    
    // 初始化 TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // QoS 配置
    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1));
    
    // 移除 topic 订阅，改用定时器直接调用 ROG-Map
    // 定时更新器（直接调用 boxSearchInflate）
    int update_period_ms = static_cast<int>(1000.0 / cfg_.update_rate);
    update_timer_ = create_wall_timer(
        std::chrono::milliseconds(update_period_ms),
        std::bind(&StairDetector::updateTimerCallback, this));
    
    // 发布台阶目标
    target_pub_ = create_publisher<rog_map_ros2_node::msg::StairTarget>(
        cfg_.output_target_topic, 10);
    
    // 发布可视化 Marker
    if (cfg_.enable_visualization) {
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            cfg_.output_marker_topic, 10);
        
        // 发布预筛选后的点云
        prefilter_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/stair_detector/prefilter_cloud", 10);
    }
    
    RCLCPP_INFO(get_logger(), "=== Stair Detector Initialized (Component + Direct-Call) ===");
    RCLCPP_INFO(get_logger(), "  Direct ROG-Map access: %s", rog_map_ptr_ ? "ENABLED" : "PENDING");
    RCLCPP_INFO(get_logger(), "  Output: %s", cfg_.output_target_topic.c_str());
    RCLCPP_INFO(get_logger(), "  ROI: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]",
                cfg_.roi_x_min, cfg_.roi_x_max,
                cfg_.roi_y_min, cfg_.roi_y_max,
                cfg_.roi_z_min, cfg_.roi_z_max);
    RCLCPP_INFO(get_logger(), "  Stair Heights: Single=%.2fm, Double=%.2fm (±%.2fm)",
                cfg_.single_stair_height, cfg_.double_stair_height, cfg_.height_tolerance);
    RCLCPP_INFO(get_logger(), "  Cluster: tolerance=%.3fm, size=[%d, %d]",
                cfg_.cluster_tolerance, cfg_.min_cluster_size, cfg_.max_cluster_size);
    RCLCPP_INFO(get_logger(), "  [NEW] Normal Estimation: %s (planarity>%.2f, nz>%.2f)",
                cfg_.enable_normal_estimation ? "ENABLED" : "DISABLED",
                cfg_.min_planarity, cfg_.horizontal_normal_z_min);
    RCLCPP_INFO(get_logger(), "  [NEW] Plane Segmentation: %s (RANSAC dist=%.3fm, iter=%d)",
                cfg_.enable_plane_segmentation ? "ENABLED" : "DISABLED",
                cfg_.ransac_distance_threshold, cfg_.ransac_max_iterations);
    
    // 注册参数变化回调
    param_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&StairDetector::parametersCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "  Dynamic parameter reconfiguration: ENABLED");
    RCLCPP_INFO(get_logger(), "  Use 'ros2 param set' or rqt_reconfigure to adjust parameters");
    
    last_process_time_ = now();
}

void StairDetector::updateTimerCallback() {
    // 检查 ROG-Map 指针
    if (rog_map_ptr_ == nullptr) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "ROG-Map pointer not set, skipping update");
        return;
    }
    
    auto t_start = std::chrono::high_resolution_clock::now();
    
    // 获取机器人位姿（base_link在odom系中的位置）
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_->lookupTransform(
            cfg_.map_frame, cfg_.target_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Failed to get transform from %s to %s: %s",
            cfg_.target_frame.c_str(), cfg_.map_frame.c_str(), ex.what());
        return;
    }
    
    Eigen::Vector3f robot_pos(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z
    );
    
    // === 直接调用 ROG-Map boxSearch() ===
    // ROI 范围（相对于 base_link）
    Eigen::Vector3d box_min(
        robot_pos.x() + cfg_.roi_x_min,
        robot_pos.y() + cfg_.roi_y_min,
        robot_pos.z() + cfg_.roi_z_min
    );
    Eigen::Vector3d box_max(
        robot_pos.x() + cfg_.roi_x_max,
        robot_pos.y() + cfg_.roi_y_max,
        robot_pos.z() + cfg_.roi_z_max
    );
    
    // 台阶检测使用原始占据点云（不需要膨胀）
    rog_map::vec_E<Eigen::Vector3d> occ_points;
    rog_map_ptr_->boxSearch(box_min, box_max, rog_map::OCCUPIED, occ_points);
    
    if (occ_points.empty()) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
            "No occupied points in ROG-Map query range");
        return;
    }
    
    // 转换为 PCL 点云（odom 系）
    PointCloud::Ptr cloud_odom(new PointCloud);
    cloud_odom->reserve(occ_points.size());
    for (const auto& pt : occ_points) {
        cloud_odom->push_back(PointT(pt.x(), pt.y(), pt.z()));
    }
    
    // 转换到 base_link 坐标系
    PointCloud::Ptr cloud_base(new PointCloud);
    if (!transformPointCloud(cloud_odom, cloud_base, cfg_.target_frame)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Failed to transform point cloud to %s", cfg_.target_frame.c_str());
        return;
    }
    
    RCLCPP_DEBUG(get_logger(), "Direct query: %zu points → %zu after transform",
                 occ_points.size(), cloud_base->size());
    
    // 执行检测
    processPointCloud(cloud_base);
    
    // 发布结果
    publishStairTarget();
    
    if (cfg_.enable_visualization) {
        publishVisualization();
        publishPrefilterCloud();  // 发布预筛选点云
    }
    
    // 性能统计
    auto t_end = std::chrono::high_resolution_clock::now();
    double dt_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    static int count = 0;
    static double sum_ms = 0.0, max_ms = 0.0, min_ms = 1e6;
    sum_ms += dt_ms;
    max_ms = std::max(max_ms, dt_ms);
    min_ms = std::min(min_ms, dt_ms);
    count++;
    
    if (count % 50 == 0) {
        RCLCPP_INFO(get_logger(), "[Perf] Avg: %.2fms, Min: %.2fms, Max: %.2fms (n=%d)",
                    sum_ms / count, min_ms, max_ms, count);
    }
    
    last_process_time_ = now();
}

void StairDetector::processPointCloud(const PointCloud::Ptr& cloud_in) {
    // Step 1: ROI 裁剪
    auto cloud_roi = roiFilter(cloud_in);
    last_roi_cloud_ = cloud_roi;
    if (cloud_roi->empty()) {
        last_prefilter_cloud_.reset();
        last_clusters_.clear();
        last_candidates_.clear();
        last_rejected_.clear();
        last_planes_.clear();
        // 未检测到任何点云，重置状态
        if (current_state_ != DetectionState::IDLE) {
            RCLCPP_INFO(get_logger(), "Lost sight of stair, resetting to IDLE");
            current_state_ = DetectionState::IDLE;
            consecutive_detection_count_ = 0;
        }
        return;
    }
    
    // Step 1.5: 预筛选（网格竖直占据率）
    auto cloud_prefilter = stairLikeFilter(cloud_roi);
    last_prefilter_cloud_ = cloud_prefilter;
    if (cloud_prefilter->empty()) {
        last_clusters_.clear();
        last_candidates_.clear();
        last_rejected_.clear();
        last_planes_.clear();
        if (current_state_ != DetectionState::IDLE) {
            RCLCPP_WARN(get_logger(), "No stair-like cells found, resetting");
            current_state_ = DetectionState::IDLE;
            consecutive_detection_count_ = 0;
        }
        return;
    }

    // === 结合平面分割和法向量验证 ===
    std::vector<StairCandidate> candidates;
    
    // Step 2: 多平面分割 (方案2优先)
    if (cfg_.enable_plane_segmentation) {
        auto planes = extractMultiplePlanes(cloud_prefilter);
        last_planes_ = planes;
        
        if (planes.size() >= 2) {
            // 找到地面和台阶平面
            PlaneModel ground_plane = planes[0];  // 最低平面
            
            for (size_t i = 1; i < planes.size(); ++i) {
                PlaneModel& step_plane = planes[i];
                float step_height = step_plane.height_from_ground - ground_plane.height_from_ground;
                
                // 验证台阶高度
                bool is_valid_single = std::abs(step_height - cfg_.single_stair_height) < cfg_.height_tolerance;
                bool is_valid_double = std::abs(step_height - cfg_.double_stair_height) < cfg_.height_tolerance;
                
                if (is_valid_single || is_valid_double) {
                    RCLCPP_DEBUG(get_logger(), 
                        "[Plane-Based] Found valid step plane: height=%.3fm (ground=%.3fm)", 
                        step_height, ground_plane.height_from_ground);
                    
                    // 将台阶平面作为候选点云进行聚类
                    std::vector<PointCloud::Ptr> step_clusters = euclideanClustering(step_plane.inliers);
                    auto plane_candidates = filterStairCandidates(step_clusters);
                    candidates.insert(candidates.end(), plane_candidates.begin(), plane_candidates.end());
                }
            }
        }
    }
    
    // Step 2.5: 区域生长聚类（方案3：考虑法向量一致性）
    if (candidates.empty()) {
        RCLCPP_DEBUG(get_logger(), "[Clustering] Using region growing with normal consistency");
        auto clusters = regionGrowingClustering(cloud_prefilter);
        
        // 方案1：对过大簇进行分割
        std::vector<PointCloud::Ptr> refined_clusters;
        for (auto& cluster : clusters) {
            if (cluster->size() > static_cast<size_t>(cfg_.max_cluster_size)) {
                RCLCPP_DEBUG(get_logger(), "[Clustering] Oversized cluster detected: %zu points", 
                            cluster->size());
                auto sub_clusters = subdivideOversizedCluster(cluster);
                refined_clusters.insert(refined_clusters.end(), 
                                       sub_clusters.begin(), sub_clusters.end());
            } else {
                refined_clusters.push_back(cluster);
            }
        }
        
        last_clusters_ = refined_clusters;
        
        if (refined_clusters.empty()) {
            last_candidates_.clear();
            last_rejected_.clear();
            if (current_state_ != DetectionState::IDLE) {
                RCLCPP_WARN(get_logger(), "No clusters found, resetting");
                current_state_ = DetectionState::IDLE;
                consecutive_detection_count_ = 0;
            }
            return;
        }
        
        // Step 3: 硬约束筛选 + 法向量验证
        candidates = filterStairCandidates(clusters);
    }

    // Step 4: 重叠候选筛选（保留综合表现最佳）
    auto filtered_candidates = resolveOverlappingCandidates(candidates);
    last_candidates_ = filtered_candidates;
    
    // Step 5: 精炼候选并更新跟踪
    updateTracking(filtered_candidates);
}

PointCloud::Ptr StairDetector::roiFilter(const PointCloud::Ptr& cloud_in) {
    PointCloud::Ptr cloud_filtered(new PointCloud);
    
    // X 轴过滤
    pcl::PassThrough<PointT> pass_x;
    pass_x.setInputCloud(cloud_in);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(cfg_.roi_x_min, cfg_.roi_x_max);
    pass_x.filter(*cloud_filtered);
    
    // Y 轴过滤
    PointCloud::Ptr cloud_temp(new PointCloud);
    pcl::PassThrough<PointT> pass_y;
    pass_y.setInputCloud(cloud_filtered);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(cfg_.roi_y_min, cfg_.roi_y_max);
    pass_y.filter(*cloud_temp);
    
    // Z 轴过滤（仅切掉地面，不限制上限）
    pcl::PassThrough<PointT> pass_z;
    pass_z.setInputCloud(cloud_temp);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(cfg_.roi_z_min, 100.0);  // 上限设为极大值，不实际限制
    pass_z.filter(*cloud_filtered);
    
    return cloud_filtered;
}

PointCloud::Ptr StairDetector::stairLikeFilter(const PointCloud::Ptr& cloud_in) {
    struct CellStats {
        int count = 0;
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::lowest();
        std::set<int> occupied_z_layers;  // 记录哪些 Z 层有点
    };

    const float cell = std::max(0.01f, cfg_.cell_size_xy);
    const float z_layer_size = 0.025f;  // Z 轴分层大小：2.5cm
    
    std::unordered_map<long long, CellStats> cells;
    cells.reserve(cloud_in->size());

    auto key_from = [cell](float x, float y) {
        const int ix = static_cast<int>(std::floor(x / cell));
        const int iy = static_cast<int>(std::floor(y / cell));
        return (static_cast<long long>(ix) << 32) ^ (static_cast<unsigned int>(iy));
    };

    // 第一次遍历：统计每个网格的完整 Z 轴分布
    for (const auto& pt : cloud_in->points) {
        const long long key = key_from(pt.x, pt.y);
        auto& stat = cells[key];
        stat.count += 1;
        stat.min_z = std::min(stat.min_z, pt.z);
        stat.max_z = std::max(stat.max_z, pt.z);
        
        // 记录该点所在的 Z 层
        int z_layer = static_cast<int>(std::floor(pt.z / z_layer_size));
        stat.occupied_z_layers.insert(z_layer);
    }

    PointCloud::Ptr filtered(new PointCloud);
    filtered->reserve(cloud_in->size());

    // 第二次遍历：根据网格特征过滤
    for (const auto& pt : cloud_in->points) {
        const long long key = key_from(pt.x, pt.y);
        const auto& stat = cells[key];
        const float height = stat.max_z - stat.min_z;
        
        // 条件 1: 点数太少，可能是噪声
        if (stat.count < cfg_.min_cell_points) {
            continue;
        }
        
        // 条件 2: 必须有一定的垂直跨度（排除完全平坦的地面）
        if (height < cfg_.min_cell_height) {
            continue;
        }
        
        // 条件 3: 顶部不能太高（排除天花板、高处悬挡物）
        if (stat.max_z > cfg_.max_cell_top_z) {
            continue;
        }
        
        // 条件 4: Z轴占据率检查（新增）
        // 计算理论上应该有多少个 Z 层
        int expected_layers = std::max(1, static_cast<int>(std::ceil(height / z_layer_size)));
        // 实际占据的层数
        int actual_layers = static_cast<int>(stat.occupied_z_layers.size());
        // 占据率
        float z_occupancy = static_cast<float>(actual_layers) / static_cast<float>(expected_layers);
        
        // 要求至少 50% 的 Z 层有点
        if (z_occupancy < 0.5f) {
            continue;
        }
        
        // 通过所有条件，保留该网格的所有点
        filtered->push_back(pt);
    }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = true;
    return filtered;
}

std::vector<PlaneModel> StairDetector::extractMultiplePlanes(const PointCloud::Ptr& cloud_in) {
    /**
     * 多平面分割 (RANSAC)
     * 目标: 分离地面、台阶面、立面
     */
    std::vector<PlaneModel> planes;
    PointCloud::Ptr remaining(new PointCloud(*cloud_in));
    
    for (int plane_id = 0; plane_id < cfg_.max_planes && 
         remaining->size() > static_cast<size_t>(cfg_.min_plane_points); ++plane_id) {
        // RANSAC 平面拟合
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(cfg_.ransac_distance_threshold);
        seg.setMaxIterations(cfg_.ransac_max_iterations);
        seg.setInputCloud(remaining);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() < static_cast<size_t>(cfg_.min_plane_points)) {
            RCLCPP_DEBUG(get_logger(), "[RANSAC] Plane %d: insufficient inliers (%zu < %d)", 
                         plane_id, inliers->indices.size(), cfg_.min_plane_points);
            break;
        }
        
        // 构建平面模型
        PlaneModel plane;
        plane.normal = Vec3f(coefficients->values[0], 
                            coefficients->values[1], 
                            coefficients->values[2]);
        plane.d = coefficients->values[3];
        plane.point_count = static_cast<int>(inliers->indices.size());
        
        // 强约束：仅接受水平平面 (|nz| > 0.9)
        plane.is_horizontal = std::abs(plane.normal.z()) > 0.9f;
        if (!plane.is_horizontal) {
            RCLCPP_DEBUG(get_logger(), "[RANSAC] Plane %d: not horizontal (nz=%.3f), skipping", 
                         plane_id, plane.normal.z());
            // 不移除点，继续下一个平面
            break;
        }
        
        // 提取内点
        plane.inliers.reset(new PointCloud);
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(remaining);
        extract.setIndices(inliers);
        extract.filter(*plane.inliers);
        
        // 计算平面高度 (取中位数，更鲁棒)
        std::vector<float> heights;
        heights.reserve(plane.inliers->size());
        for (const auto& pt : plane.inliers->points) {
            heights.push_back(pt.z);
        }
        std::sort(heights.begin(), heights.end());
        plane.height_from_ground = heights[heights.size() / 2];
        
        RCLCPP_DEBUG(get_logger(), "[RANSAC] Plane %d: points=%d, height=%.3fm, nz=%.3f", 
                     plane_id, plane.point_count, plane.height_from_ground, plane.normal.z());
        
        planes.push_back(plane);
        
        // 移除已提取的内点
        extract.setNegative(true);
        PointCloud::Ptr temp(new PointCloud);
        extract.filter(*temp);
        remaining = temp;
    }
    
    // 按高度排序：地面 < 台阶
    std::sort(planes.begin(), planes.end(), 
              [](const PlaneModel& a, const PlaneModel& b) {
                  return a.height_from_ground < b.height_from_ground;
              });
    
    if (!planes.empty()) {
        RCLCPP_DEBUG(get_logger(), "[RANSAC] Extracted %zu horizontal planes", planes.size());
    }
    
    return planes;
}

Vec3f StairDetector::computePointNormal(const PointCloud::Ptr& cloud, 
                                         const pcl::search::KdTree<PointT>::Ptr& tree,
                                         int point_idx, int k_neighbors) {
    /**
     * 快速法向量估计：使用 k 近邻 PCA
     */
    std::vector<int> indices;
    std::vector<float> distances;
    
    if (tree->nearestKSearch(cloud->points[point_idx], k_neighbors, indices, distances) < 3) {
        return Vec3f(0, 0, 1);  // 默认向上
    }
    
    // 计算质心
    Vec3f centroid = Vec3f::Zero();
    for (int idx : indices) {
        const auto& pt = cloud->points[idx];
        centroid += Vec3f(pt.x, pt.y, pt.z);
    }
    centroid /= static_cast<float>(indices.size());
    
    // 计算协方差矩阵
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int idx : indices) {
        const auto& pt = cloud->points[idx];
        Vec3f d(pt.x - centroid.x(), pt.y - centroid.y(), pt.z - centroid.z());
        cov += d * d.transpose();
    }
    cov /= static_cast<float>(indices.size());
    
    // 特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success) {
        return Vec3f(0, 0, 1);
    }
    
    // 法向量 = 最小特征值对应的特征向量
    Vec3f normal = solver.eigenvectors().col(0);
    normal.normalize();
    
    return normal;
}

std::vector<PointCloud::Ptr> StairDetector::regionGrowingClustering(
    const PointCloud::Ptr& cloud_filtered) {
    /**
     * 区域生长聚类：考虑法向量一致性
     * 只有距离近 AND 法向量相似的点才会被聚到一起
     * 这样可以分离共面但法向量不同的区域（台阶边缘 vs 墙面）
     */
    
    if (cloud_filtered->empty()) {
        return {};
    }
    
    // 构建 KdTree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_filtered);
    
    // 预计算所有点的法向量
    std::vector<Vec3f> normals(cloud_filtered->size());
    const int k_neighbors = 10;  // 用于法向量估计的邻居数
    
    for (size_t i = 0; i < cloud_filtered->size(); ++i) {
        normals[i] = computePointNormal(cloud_filtered, tree, i, k_neighbors);
    }
    
    // 区域生长
    std::vector<bool> processed(cloud_filtered->size(), false);
    std::vector<PointCloud::Ptr> clusters;
    
    const float distance_tolerance = cfg_.cluster_tolerance;
    const float normal_angle_threshold = 10.0f * M_PI / 180.0f;  // 10度
    
    for (size_t seed_idx = 0; seed_idx < cloud_filtered->size(); ++seed_idx) {
        if (processed[seed_idx]) continue;
        
        // 开始新簇
        PointCloud::Ptr cluster(new PointCloud);
        std::vector<int> seed_queue;
        seed_queue.push_back(seed_idx);
        processed[seed_idx] = true;
        
        while (!seed_queue.empty()) {
            int current_idx = seed_queue.back();
            seed_queue.pop_back();
            
            cluster->push_back(cloud_filtered->points[current_idx]);
            
            // 查找邻居
            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_distances;
            tree->radiusSearch(cloud_filtered->points[current_idx], 
                              distance_tolerance, 
                              neighbor_indices, 
                              neighbor_distances);
            
            for (int neighbor_idx : neighbor_indices) {
                if (processed[neighbor_idx]) continue;
                
                // 关键：检查法向量夹角
                float dot_product = normals[current_idx].dot(normals[neighbor_idx]);
                dot_product = std::max(-1.0f, std::min(1.0f, dot_product));
                float angle = std::acos(std::abs(dot_product));  // 使用绝对值（方向可能相反）
                
                if (angle < normal_angle_threshold) {
                    processed[neighbor_idx] = true;
                    seed_queue.push_back(neighbor_idx);
                }
            }
        }
        
        // 尺寸检查
        if (cluster->size() >= static_cast<size_t>(cfg_.min_cluster_size) &&
            cluster->size() <= static_cast<size_t>(cfg_.max_cluster_size)) {
            cluster->width = cluster->size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }
    }
    
    RCLCPP_DEBUG(get_logger(), "[RegionGrowing] Generated %zu clusters from %zu points",
                 clusters.size(), cloud_filtered->size());
    
    return clusters;
}

std::vector<PointCloud::Ptr> StairDetector::subdivideOversizedCluster(
    const PointCloud::Ptr& cluster) {
    /**
     * 方案1：过大簇分割
     * 将横向尺寸过大的簇沿长边切分成多个子簇
     */
    
    std::vector<PointCloud::Ptr> sub_clusters;
    
    // 计算边界框
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    
    float width_x = max_pt.x - min_pt.x;
    float width_y = max_pt.y - min_pt.y;
    float width_z = max_pt.z - min_pt.z;
    
    const float max_horizontal_size = 1.0f;  // 最大横向尺寸 1m
    
    // 如果横向尺寸不超标，不切分
    if (width_x <= max_horizontal_size && width_y <= max_horizontal_size) {
        sub_clusters.push_back(cluster);
        return sub_clusters;
    }
    
    // 确定切分方向和数量
    bool split_along_x = (width_x > width_y);
    float split_size = split_along_x ? width_x : width_y;
    int num_segments = std::ceil(split_size / max_horizontal_size);
    float segment_size = split_size / num_segments;
    
    // 按段分配点
    std::vector<PointCloud::Ptr> segments(num_segments);
    for (auto& seg : segments) {
        seg.reset(new PointCloud);
    }
    
    for (const auto& pt : cluster->points) {
        float coord = split_along_x ? pt.x : pt.y;
        float min_coord = split_along_x ? min_pt.x : min_pt.y;
        int segment_idx = static_cast<int>((coord - min_coord) / segment_size);
        segment_idx = std::min(segment_idx, num_segments - 1);
        segments[segment_idx]->push_back(pt);
    }
    
    // 保留有效段
    for (auto& seg : segments) {
        if (seg->size() >= static_cast<size_t>(cfg_.min_cluster_size)) {
            seg->width = seg->size();
            seg->height = 1;
            seg->is_dense = true;
            sub_clusters.push_back(seg);
        }
    }
    
    RCLCPP_DEBUG(get_logger(), "[Subdivide] Split cluster (%zu pts, %.2fm x %.2fm) into %zu segments",
                 cluster->size(), width_x, width_y, sub_clusters.size());
    
    return sub_clusters;
}

std::vector<PointCloud::Ptr> StairDetector::euclideanClustering(
    const PointCloud::Ptr& cloud_filtered) {
    
    std::vector<PointCloud::Ptr> clusters;
    
    // 创建 KdTree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_filtered);
    
    // 执行聚类
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cfg_.cluster_tolerance);
    ec.setMinClusterSize(cfg_.min_cluster_size);
    ec.setMaxClusterSize(cfg_.max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);
    
    // 提取每个聚类
    for (const auto& indices : cluster_indices) {
        PointCloud::Ptr cluster(new PointCloud);
        for (const auto& idx : indices.indices) {
            cluster->push_back((*cloud_filtered)[idx]);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }

    if (clusters.size() <= 1) {
        return clusters;
    }

    // 合并重叠度高的聚类（AABB overlap > 0.7）
    std::vector<PointT> min_pts(clusters.size());
    std::vector<PointT> max_pts(clusters.size());
    for (size_t i = 0; i < clusters.size(); ++i) {
        pcl::getMinMax3D(*clusters[i], min_pts[i], max_pts[i]);
    }

    auto overlap_ratio = [&](size_t a, size_t b) {
        const float ax_min = min_pts[a].x;
        const float ax_max = max_pts[a].x;
        const float ay_min = min_pts[a].y;
        const float ay_max = max_pts[a].y;

        const float bx_min = min_pts[b].x;
        const float bx_max = max_pts[b].x;
        const float by_min = min_pts[b].y;
        const float by_max = max_pts[b].y;

        const float ix = std::max(0.0f, std::min(ax_max, bx_max) - std::max(ax_min, bx_min));
        const float iy = std::max(0.0f, std::min(ay_max, by_max) - std::max(ay_min, by_min));
        const float inter = ix * iy;
        if (inter <= 0.0f) {
            return 0.0f;
        }
        const float area_a = std::max(1e-3f, (ax_max - ax_min) * (ay_max - ay_min));
        const float area_b = std::max(1e-3f, (bx_max - bx_min) * (by_max - by_min));
        return inter / std::min(area_a, area_b);
    };

    std::vector<size_t> parent(clusters.size());
    std::iota(parent.begin(), parent.end(), 0);

    auto find_root = [&](size_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };

    auto unite = [&](size_t a, size_t b) {
        size_t ra = find_root(a);
        size_t rb = find_root(b);
        if (ra != rb) {
            parent[rb] = ra;
        }
    };

    const float overlap_thresh = 0.7f;
    for (size_t i = 0; i < clusters.size(); ++i) {
        for (size_t j = i + 1; j < clusters.size(); ++j) {
            if (overlap_ratio(i, j) > overlap_thresh) {
                unite(i, j);
            }
        }
    }

    std::unordered_map<size_t, PointCloud::Ptr> merged_map;
    merged_map.reserve(clusters.size());
    for (size_t i = 0; i < clusters.size(); ++i) {
        size_t root = find_root(i);
        auto& merged = merged_map[root];
        if (!merged) {
            merged.reset(new PointCloud);
        }
        merged->insert(merged->end(), clusters[i]->begin(), clusters[i]->end());
    }

    std::vector<PointCloud::Ptr> merged_clusters;
    merged_clusters.reserve(merged_map.size());
    for (auto& entry : merged_map) {
        auto& merged = entry.second;
        merged->width = merged->size();
        merged->height = 1;
        merged->is_dense = true;
        merged_clusters.push_back(merged);
    }

    return merged_clusters;
}

std::vector<StairCandidate> StairDetector::filterStairCandidates(
    const std::vector<PointCloud::Ptr>& clusters) {
    
    std::vector<StairCandidate> candidates;
    last_rejected_.clear();
    
    for (const auto& cluster : clusters) {
        StairCandidate candidate;
        candidate.cloud = cluster;
        
        // 计算包围盒
        PointT min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);
        
        candidate.bbox_min = Vec3f(min_pt.x, min_pt.y, min_pt.z);
        candidate.bbox_max = Vec3f(max_pt.x, max_pt.y, max_pt.z);
        
    // 几何特征
    candidate.top_z = max_pt.z;

        // 计算 yaw-only OBB（仅在 XY 平面做 PCA，Z 轴与 base_link 对齐）
        Eigen::Vector2f mean_xy(0.0f, 0.0f);
        for (const auto& pt : cluster->points) {
            mean_xy.x() += pt.x;
            mean_xy.y() += pt.y;
        }
        mean_xy /= static_cast<float>(cluster->points.size());

        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        for (const auto& pt : cluster->points) {
            Eigen::Vector2f d(pt.x - mean_xy.x(), pt.y - mean_xy.y());
            cov += d * d.transpose();
        }
        cov /= std::max(1.0f, static_cast<float>(cluster->points.size()));

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);
        Eigen::Vector2f axis_major = solver.eigenvectors().col(1).normalized();
        Eigen::Vector2f axis_minor(-axis_major.y(), axis_major.x());

        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();

        for (const auto& pt : cluster->points) {
            Eigen::Vector2f d(pt.x - mean_xy.x(), pt.y - mean_xy.y());
            float local_x = d.dot(axis_major);
            float local_y = d.dot(axis_minor);
            min_x = std::min(min_x, local_x);
            max_x = std::max(max_x, local_x);
            min_y = std::min(min_y, local_y);
            max_y = std::max(max_y, local_y);
        }

        const float yaw = std::atan2(axis_major.y(), axis_major.x());
        const Eigen::Quaternionf yaw_q(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));

        const float depth_xy = max_x - min_x;
        const float width_xy = max_y - min_y;
        const float z_thickness = max_pt.z - min_pt.z;
        candidate.height = z_thickness;  // 使用真实台阶高度

        Eigen::Vector2f center_local(0.5f * (min_x + max_x), 0.5f * (min_y + max_y));
        Eigen::Vector2f center_xy = mean_xy + axis_major * center_local.x() + axis_minor * center_local.y();

        candidate.obb_center = Vec3f(center_xy.x(), center_xy.y(), 0.5f * (min_pt.z + max_pt.z));
        candidate.obb_orientation = yaw_q;
        candidate.obb_dims = Vec3f(depth_xy, width_xy, z_thickness);

    // 语义化：宽度=较长边，深度=较短边（符合台阶定义）
        candidate.width = std::max(depth_xy, width_xy);
        candidate.depth = std::min(depth_xy, width_xy);
        
        // === 硬约束判定 ===
        
        // 1. 高度校验：仅匹配一级台阶（禁用二级台阶判断，避免误识别）
        bool is_single = std::abs(candidate.height - cfg_.single_stair_height) < cfg_.height_tolerance;
        
        if (!is_single) {
            RejectedCandidate rejected;
            rejected.bbox_min = candidate.bbox_min;
            rejected.bbox_max = candidate.bbox_max;
            rejected.height = candidate.height;
            rejected.top_z = candidate.top_z;
            rejected.width = candidate.width;
            rejected.depth = candidate.depth;
            rejected.z_thickness = z_thickness;
            rejected.obb_center = candidate.obb_center;
            rejected.obb_orientation = candidate.obb_orientation;
            rejected.obb_dims = candidate.obb_dims;
            rejected.reason = "height_mismatch";
            last_rejected_.push_back(rejected);
            continue;  // 高度不匹配，丢弃
        }
        
        candidate.type = StairType::SINGLE;  // 仅识别单级台阶
        
        // 2. 形态校验：宽度、深度、厚度
        if (candidate.width < cfg_.min_stair_width) {
            RejectedCandidate rejected;
            rejected.bbox_min = candidate.bbox_min;
            rejected.bbox_max = candidate.bbox_max;
            rejected.height = candidate.height;
            rejected.top_z = candidate.top_z;
            rejected.width = candidate.width;
            rejected.depth = candidate.depth;
            rejected.z_thickness = z_thickness;
            rejected.obb_center = candidate.obb_center;
            rejected.obb_orientation = candidate.obb_orientation;
            rejected.obb_dims = candidate.obb_dims;
            rejected.reason = "too_narrow";
            last_rejected_.push_back(rejected);
            continue;  // 太窄
        }
        
        if (candidate.depth < cfg_.min_stair_depth) {
            RejectedCandidate rejected;
            rejected.bbox_min = candidate.bbox_min;
            rejected.bbox_max = candidate.bbox_max;
            rejected.height = candidate.height;
            rejected.top_z = candidate.top_z;
            rejected.width = candidate.width;
            rejected.depth = candidate.depth;
            rejected.z_thickness = z_thickness;
            rejected.obb_center = candidate.obb_center;
            rejected.obb_orientation = candidate.obb_orientation;
            rejected.obb_dims = candidate.obb_dims;
            rejected.reason = "too_shallow";
            last_rejected_.push_back(rejected);
            continue;  // 太浅
        }
        
        if (z_thickness > cfg_.max_z_thickness) {
            RejectedCandidate rejected;
            rejected.bbox_min = candidate.bbox_min;
            rejected.bbox_max = candidate.bbox_max;
            rejected.height = candidate.height;
            rejected.top_z = candidate.top_z;
            rejected.width = candidate.width;
            rejected.depth = candidate.depth;
            rejected.z_thickness = z_thickness;
            rejected.obb_center = candidate.obb_center;
            rejected.obb_orientation = candidate.obb_orientation;
            rejected.obb_dims = candidate.obb_dims;
            rejected.reason = "too_thick";
            last_rejected_.push_back(rejected);
            continue;  // 太厚，可能是墙壁
        }
        
        // === 方案1: 法向量估计与验证 ===
        if (cfg_.enable_normal_estimation) {
            computeSurfaceNormals(candidate);
            
            std::string reject_reason;
            if (!validateNormalFeatures(candidate, reject_reason)) {
                RejectedCandidate rejected;
                rejected.bbox_min = candidate.bbox_min;
                rejected.bbox_max = candidate.bbox_max;
                rejected.height = candidate.height;
                rejected.top_z = candidate.top_z;
                rejected.width = candidate.width;
                rejected.depth = candidate.depth;
                rejected.z_thickness = z_thickness;
                rejected.obb_center = candidate.obb_center;
                rejected.obb_orientation = candidate.obb_orientation;
                rejected.obb_dims = candidate.obb_dims;
                rejected.reason = reject_reason;
                last_rejected_.push_back(rejected);
                continue;  // 法向量验证失败
            }
        }
        
        // 3. 精炼边缘和中心
        refineStairCandidate(candidate);
        
        candidates.push_back(candidate);
    }
    
    return candidates;
}

std::vector<StairCandidate> StairDetector::resolveOverlappingCandidates(
    const std::vector<StairCandidate>& candidates) {

    if (candidates.size() <= 1) {
        return candidates;
    }

    auto overlap_ratio = [](const StairCandidate& a, const StairCandidate& b) {
        const float ax_min = a.bbox_min.x();
        const float ax_max = a.bbox_max.x();
        const float ay_min = a.bbox_min.y();
        const float ay_max = a.bbox_max.y();

        const float bx_min = b.bbox_min.x();
        const float bx_max = b.bbox_max.x();
        const float by_min = b.bbox_min.y();
        const float by_max = b.bbox_max.y();

        const float ix = std::max(0.0f, std::min(ax_max, bx_max) - std::max(ax_min, bx_min));
        const float iy = std::max(0.0f, std::min(ay_max, by_max) - std::max(ay_min, by_min));
        const float inter = ix * iy;
        if (inter <= 0.0f) {
            return 0.0f;
        }
        const float area_a = std::max(1e-3f, (ax_max - ax_min) * (ay_max - ay_min));
        const float area_b = std::max(1e-3f, (bx_max - bx_min) * (by_max - by_min));
        return inter / std::min(area_a, area_b);
    };

    auto score = [&](const StairCandidate& c) {
        const float thickness = std::max(1e-3f, c.obb_dims.z());
        float s = c.width - 0.6f * thickness;  // 宽度优先，厚度惩罚
        if (current_state_ == DetectionState::LOCKED) {
            const float locked_overlap = overlap_ratio(c, locked_stair_);
            if (locked_overlap > 0.2f) {
                s += 1.0f;  // 置信度高的锁定目标优先保留
            }
        }
        return s;
    };

    std::vector<size_t> indices(candidates.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
              [&](size_t a, size_t b) { return score(candidates[a]) > score(candidates[b]); });

    std::vector<StairCandidate> kept;
    kept.reserve(candidates.size());

    const float overlap_thresh = 0.35f;
    for (size_t idx : indices) {
        const auto& cand = candidates[idx];
        bool overlapped = false;
        for (const auto& existing : kept) {
            if (overlap_ratio(cand, existing) > overlap_thresh) {
                overlapped = true;
                break;
            }
        }
        if (!overlapped) {
            kept.push_back(cand);
        }
    }

    return kept;
}

void StairDetector::computeSurfaceNormals(StairCandidate& candidate) {
    /**
     * 法向量估计 (方案1: PCA方法)
     * 使用点云协方差矩阵的最小特征值对应的特征向量作为法向量
     */
    
    candidate.normal_valid = false;
    candidate.horizontal_points = 0;
    candidate.vertical_points = 0;
    candidate.planarity = 0.0f;
    
    // 点数不足，跳过
    if (candidate.cloud->points.size() < static_cast<size_t>(cfg_.normal_min_points)) {
        return;
    }
    
    // 1. 计算质心
    Vec3f centroid = Vec3f::Zero();
    for (const auto& pt : candidate.cloud->points) {
        centroid += Vec3f(pt.x, pt.y, pt.z);
    }
    centroid /= static_cast<float>(candidate.cloud->points.size());
    
    // 2. 计算协方差矩阵
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& pt : candidate.cloud->points) {
        Vec3f d(pt.x - centroid.x(), pt.y - centroid.y(), pt.z - centroid.z());
        cov += d * d.transpose();
    }
    cov /= static_cast<float>(candidate.cloud->points.size());
    
    // 3. 特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success) {
        return;
    }
    
    // 特征值从小到大: λ0 < λ1 < λ2
    Vec3f eigenvalues = solver.eigenvalues();
    Eigen::Matrix3f eigenvectors = solver.eigenvectors();
    
    // 法向量 = 最小特征值对应的特征向量
    candidate.surface_normal = eigenvectors.col(0);
    
    // 不强制法向量朝上，保持原始方向
    // （竖直平面的法向量应该是水平的）
    
    // 4. 计算平面性 = 1 - (λ0/λ1)
    if (eigenvalues(1) > 1e-6f) {
        candidate.planarity = 1.0f - eigenvalues(0) / eigenvalues(1);
    }
    
    // 5. 逐点分类：竖直面 vs 水平面
    // 台阶的竖直边缘：法向量水平朝外 (|nz| ≈ 0)
    const float total_points = static_cast<float>(candidate.cloud->points.size());
    const float nz_abs = std::abs(candidate.surface_normal.z());
    
    if (nz_abs < 0.2f) {
        // 法向量接近水平 → 竖直平面（台阶边缘）
        candidate.vertical_points = static_cast<int>(total_points);
        candidate.horizontal_points = 0;
    } else if (nz_abs > 0.9f) {
        // 法向量接近竖直 → 水平平面
        candidate.horizontal_points = static_cast<int>(total_points);
        candidate.vertical_points = 0;
    }
    
    candidate.normal_valid = true;
}

bool StairDetector::validateNormalFeatures(const StairCandidate& candidate, std::string& reject_reason) {
    /**
     * 法向量特征验证 (方案1)
     * 检查平面性、法向量方向、水平点占比
     */
    
    if (!candidate.normal_valid) {
        reject_reason = "normal_invalid";
        return true;  // 法向量无效时不拒绝，由几何约束决定
    }
    
    // 1. 平面性检查
    if (candidate.planarity < cfg_.min_planarity) {
        reject_reason = "surface_not_planar";
        RCLCPP_DEBUG(get_logger(), "[Normal] Rejected: planarity=%.3f < %.3f", 
                     candidate.planarity, cfg_.min_planarity);
        return false;
    }
    
    // 2. 法向量朝向检查（修正：检查竖直平面，不是水平平面）
    // 台阶竖直边缘的法向量应该是水平的（nz ≈ 0）
    float nz_abs = std::abs(candidate.surface_normal.z());
    if (nz_abs > cfg_.horizontal_normal_z_min) {
        // nz 太大 → 这是水平面，不是台阶边缘 → 拒绝
        reject_reason = "normal_not_vertical";
        RCLCPP_DEBUG(get_logger(), "[Normal] Rejected: |nz|=%.3f > %.3f (expected vertical plane)", 
                     nz_abs, cfg_.horizontal_normal_z_min);
        return false;
    }
    
    // 3. 竖直点占比检查（修正：检查竖直点，不是水平点）
    float vertical_ratio = candidate.vertical_points / 
                          static_cast<float>(candidate.cloud->points.size());
    if (vertical_ratio < cfg_.horizontal_points_ratio_min) {
        reject_reason = "insufficient_vertical_surface";
        RCLCPP_DEBUG(get_logger(), "[Normal] Rejected: vertical_ratio=%.3f < %.3f", 
                     vertical_ratio, cfg_.horizontal_points_ratio_min);
        return false;
    }
    // if (horizontal_ratio < cfg_.horizontal_points_ratio_min) {
    //     reject_reason = "insufficient_horizontal_surface";
    //     RCLCPP_DEBUG(get_logger(), "[Normal] Rejected: horizontal_ratio=%.3f < %.3f", 
    //                  horizontal_ratio, cfg_.horizontal_points_ratio_min);
    //     return false;
    // }
    
    // RCLCPP_DEBUG(get_logger(), "[Normal] Passed: planarity=%.3f, nz=%.3f, h_ratio=%.3f", 
    //              candidate.planarity, nz_abs, horizontal_ratio);
    return true;
}

void StairDetector::refineStairCandidate(StairCandidate& candidate) {
    // 计算中心（使用 X/Y 的中点，Z 使用顶面高度）
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*candidate.cloud, centroid);
    
    candidate.center = Vec3f(centroid[0], centroid[1], candidate.top_z);
    
    // 提取前沿：找到 X 坐标最小的 10% 点
    std::vector<float> x_coords;
    for (const auto& pt : candidate.cloud->points) {
        x_coords.push_back(pt.x);
    }
    std::sort(x_coords.begin(), x_coords.end());
    
    int front_count = std::max(1, static_cast<int>(x_coords.size() * 0.1));
    float edge_sum = 0.0f;
    for (int i = 0; i < front_count; ++i) {
        edge_sum += x_coords[i];
    }
    candidate.edge_x = edge_sum / front_count;
}

void StairDetector::updateTracking(const std::vector<StairCandidate>& candidates) {
    if (candidates.empty()) {
        // 未检测到候选台阶
        consecutive_detection_count_ = std::max(0, consecutive_detection_count_ - 1);
        
        if (consecutive_detection_count_ == 0 && current_state_ != DetectionState::IDLE) {
            RCLCPP_INFO(get_logger(), "Lost tracking, resetting to IDLE");
            current_state_ = DetectionState::IDLE;
            in_blind_zone_ = false;
        }
        return;
    }
    
    // 选择最近的候选（最可能是目标台阶）
    StairCandidate best_candidate = candidates[0];
    float min_dist = computeDistanceToRobot(best_candidate.center);
    
    for (const auto& cand : candidates) {
        float dist = computeDistanceToRobot(cand.center);
        if (dist < min_dist) {
            min_dist = dist;
            best_candidate = cand;
        }
    }
    
    // 更新连续检测计数
    consecutive_detection_count_++;
    
    // 状态机更新
    if (current_state_ == DetectionState::IDLE) {
        if (consecutive_detection_count_ >= 1) {
            current_state_ = DetectionState::DETECTED;
            RCLCPP_INFO(get_logger(), "Stair DETECTED (type=%d, height=%.2fm)",
                        static_cast<int>(best_candidate.type), best_candidate.height);
        }
    }
    
    if (current_state_ == DetectionState::DETECTED) {
        if (consecutive_detection_count_ >= cfg_.min_detection_frames) {
            current_state_ = DetectionState::LOCKED;
            RCLCPP_INFO(get_logger(), "Stair LOCKED! Ready for climbing");

            try {
                geometry_msgs::msg::TransformStamped transform_stamped =
                    tf_buffer_->lookupTransform(
                        cfg_.map_frame,
                        cfg_.target_frame,
                        tf2::TimePointZero);

                auto& t = transform_stamped.transform.translation;
                auto& q = transform_stamped.transform.rotation;
                Eigen::Quaternionf map_q(q.w, q.x, q.y, q.z);
                Eigen::Vector3f map_t(t.x, t.y, t.z);

                locked_stair_map_ = locked_stair_;
                locked_stair_map_.center = map_q * locked_stair_.center + map_t;
                locked_stair_map_.obb_center = map_q * locked_stair_.obb_center + map_t;
                locked_stair_map_.obb_orientation = map_q * locked_stair_.obb_orientation;

                Eigen::Vector3f top_local(locked_stair_.center.x(),
                                           locked_stair_.center.y(),
                                           locked_stair_.top_z);
                Eigen::Vector3f top_world = map_q * top_local + map_t;
                locked_stair_map_.top_z = top_world.z();
            } catch (const tf2::TransformException& ex) {
                locked_stair_map_ = locked_stair_;
            }
        }
    }
    
    // 低通滤波平滑位置（仅在 LOCKED 状态）
    if (current_state_ == DetectionState::LOCKED) {
        if (locked_stair_.cloud == nullptr) {
            // 首次锁定
            locked_stair_ = best_candidate;
        } else {
            // 平滑更新
            locked_stair_.center = cfg_.lowpass_alpha * best_candidate.center +
                                    (1.0f - cfg_.lowpass_alpha) * locked_stair_.center;
            locked_stair_.edge_x = cfg_.lowpass_alpha * best_candidate.edge_x +
                                    (1.0f - cfg_.lowpass_alpha) * locked_stair_.edge_x;
        }
        
        // 盲区判定
        float edge_distance = locked_stair_.edge_x;  // 在 base_link 系下，X 即距离
        if (edge_distance < cfg_.blind_zone_distance && !in_blind_zone_) {
            in_blind_zone_ = true;
            last_known_position_ = locked_stair_.center;
            RCLCPP_WARN(get_logger(), "Entering BLIND_ZONE mode (distance=%.2fm)", edge_distance);
        }
    } else {
        locked_stair_ = best_candidate;
    }
}

bool StairDetector::validateStairGeometry(const StairCandidate& candidate) {
    // 额外的几何验证（可选，当前已在 filter 中实现）
    return true;
}

float StairDetector::computeDistanceToRobot(const Vec3f& point) {
    // 在 base_link 系下，机器人在原点
    return std::sqrt(point.x() * point.x() + point.y() * point.y());
}

void StairDetector::publishVisualization() {
    if (!marker_pub_) return;
    
    visualization_msgs::msg::MarkerArray markers;
    
    // 清除旧的 Marker
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = cfg_.target_frame;
    delete_marker.header.stamp = now();
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    // 调试可视化：ROI / 聚类（可选）/ 被拒绝候选
    addRoiMarker(markers);
    // addClusterMarkers(markers);  // 调试时才开启
    addRejectedMarkers(markers);

    // 状态机驱动的可视化逻辑（避免重复绘制）
    if (current_state_ == DetectionState::IDLE) {
        // IDLE 状态：显示所有通过硬约束的候选（便于调试）
        const size_t max_candidates = 20;
        for (size_t i = 0; i < last_candidates_.size() && i < max_candidates; ++i) {
            addCandidateMarker(markers, last_candidates_[i], 100 + static_cast<int>(i), false,
                               cfg_.target_frame);
        }
    } else if (current_state_ == DetectionState::DETECTED) {
        // DETECTED 状态：仅显示正在追踪的目标（黄色，base_link 系）
        addCandidateMarker(markers, locked_stair_, 0, false, cfg_.target_frame);
    } else if (current_state_ == DetectionState::LOCKED) {
        // LOCKED 状态：仅显示锁定目标（绿色，map 系）
        addCandidateMarker(markers, locked_stair_map_, 0, true, cfg_.map_frame);
    }
    
    marker_pub_->publish(markers);
}

void StairDetector::addRoiMarker(visualization_msgs::msg::MarkerArray& markers) {
    visualization_msgs::msg::Marker roi;
    roi.header.frame_id = cfg_.target_frame;
    roi.header.stamp = now();
    roi.ns = "stair_roi";
    roi.id = 0;
    roi.type = visualization_msgs::msg::Marker::LINE_LIST;
    roi.action = visualization_msgs::msg::Marker::ADD;
    roi.scale.x = 0.02;
    roi.color.r = 0.0f;
    roi.color.g = 1.0f;
    roi.color.b = 1.0f;
    roi.color.a = 0.8f;
    roi.lifetime = rclcpp::Duration::from_seconds(0.5);

    const double x_min = cfg_.roi_x_min;
    const double x_max = cfg_.roi_x_max;
    const double y_min = cfg_.roi_y_min;
    const double y_max = cfg_.roi_y_max;
    const double z_min = cfg_.roi_z_min;
    const double z_max = cfg_.roi_z_max;

    auto add_edge = [&roi](double x1, double y1, double z1,
                           double x2, double y2, double z2) {
        geometry_msgs::msg::Point a;
        a.x = x1; a.y = y1; a.z = z1;
        geometry_msgs::msg::Point b;
        b.x = x2; b.y = y2; b.z = z2;
        roi.points.push_back(a);
        roi.points.push_back(b);
    };

    // 底面
    add_edge(x_min, y_min, z_min, x_max, y_min, z_min);
    add_edge(x_max, y_min, z_min, x_max, y_max, z_min);
    add_edge(x_max, y_max, z_min, x_min, y_max, z_min);
    add_edge(x_min, y_max, z_min, x_min, y_min, z_min);
    // 顶面
    add_edge(x_min, y_min, z_max, x_max, y_min, z_max);
    add_edge(x_max, y_min, z_max, x_max, y_max, z_max);
    add_edge(x_max, y_max, z_max, x_min, y_max, z_max);
    add_edge(x_min, y_max, z_max, x_min, y_min, z_max);
    // 立柱
    add_edge(x_min, y_min, z_min, x_min, y_min, z_max);
    add_edge(x_max, y_min, z_min, x_max, y_min, z_max);
    add_edge(x_max, y_max, z_min, x_max, y_max, z_max);
    add_edge(x_min, y_max, z_min, x_min, y_max, z_max);

    markers.markers.push_back(roi);
}

void StairDetector::addClusterMarkers(visualization_msgs::msg::MarkerArray& markers) {
    const size_t max_clusters = 30;
    size_t count = 0;
    for (const auto& cluster : last_clusters_) {
        if (count >= max_clusters) break;

        PointT min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);

        visualization_msgs::msg::Marker bbox;
        bbox.header.frame_id = cfg_.target_frame;
        bbox.header.stamp = now();
        bbox.ns = "stair_cluster";
        bbox.id = static_cast<int>(count);
        bbox.type = visualization_msgs::msg::Marker::CUBE;
        bbox.action = visualization_msgs::msg::Marker::ADD;

        bbox.pose.position.x = (min_pt.x + max_pt.x) * 0.5;
        bbox.pose.position.y = (min_pt.y + max_pt.y) * 0.5;
        bbox.pose.position.z = (min_pt.z + max_pt.z) * 0.5;
        bbox.pose.orientation.w = 1.0;

        bbox.scale.x = std::max(0.03f, max_pt.x - min_pt.x);
        bbox.scale.y = std::max(0.03f, max_pt.y - min_pt.y);
        bbox.scale.z = std::max(0.03f, max_pt.z - min_pt.z);

        bbox.color.r = 0.0f;
        bbox.color.g = 0.5f;
        bbox.color.b = 1.0f;
        bbox.color.a = 0.25f;
        bbox.lifetime = rclcpp::Duration::from_seconds(0.5);

        markers.markers.push_back(bbox);
        ++count;
    }
}

void StairDetector::addRejectedMarkers(visualization_msgs::msg::MarkerArray& markers) {
    const size_t max_rejected = 30;
    for (size_t i = 0; i < last_rejected_.size() && i < max_rejected; ++i) {
        const auto& rejected = last_rejected_[i];

        visualization_msgs::msg::Marker bbox;
        bbox.header.frame_id = cfg_.target_frame;
        bbox.header.stamp = now();
        bbox.ns = "stair_rejected";
        bbox.id = static_cast<int>(i);
        bbox.type = visualization_msgs::msg::Marker::CUBE;
        bbox.action = visualization_msgs::msg::Marker::ADD;

    bbox.pose.position.x = rejected.obb_center.x();
    bbox.pose.position.y = rejected.obb_center.y();
    bbox.pose.position.z = rejected.obb_center.z();
    bbox.pose.orientation.x = rejected.obb_orientation.x();
    bbox.pose.orientation.y = rejected.obb_orientation.y();
    bbox.pose.orientation.z = rejected.obb_orientation.z();
    bbox.pose.orientation.w = rejected.obb_orientation.w();
        
    bbox.scale.x = std::max(0.03f, rejected.obb_dims.x());
    bbox.scale.y = std::max(0.03f, rejected.obb_dims.y());
    bbox.scale.z = std::max(0.03f, rejected.obb_dims.z());

        bbox.color.r = 1.0f;
        bbox.color.g = 0.0f;
        bbox.color.b = 0.0f;
        bbox.color.a = 0.2f;
        bbox.lifetime = rclcpp::Duration::from_seconds(0.5);

        markers.markers.push_back(bbox);

        visualization_msgs::msg::Marker text;
        text.header = bbox.header;
        text.ns = "stair_rejected_text";
        text.id = static_cast<int>(i);
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = bbox.pose.position.x;
        text.pose.position.y = bbox.pose.position.y;
        text.pose.position.z = bbox.pose.position.z + 0.1;
        text.pose.orientation.w = 1.0;
        if (rejected.reason == "height_mismatch") {
            std::ostringstream oss;
            oss << rejected.reason << " h=" << std::fixed << std::setprecision(2)
                << rejected.height;
            text.text = oss.str();
        } else {
            text.text = rejected.reason;
        }
        text.scale.z = 0.12f;
        text.color.r = 1.0f;
        text.color.g = 0.2f;
        text.color.b = 0.2f;
        text.color.a = 1.0f;
        text.lifetime = bbox.lifetime;

        markers.markers.push_back(text);
    }
}

void StairDetector::addCandidateMarker(
    visualization_msgs::msg::MarkerArray& markers,
    const StairCandidate& candidate,
    int id,
    bool is_locked,
    const std::string& frame_id) {
    
    // 包围盒
    visualization_msgs::msg::Marker bbox;
    bbox.header.frame_id = frame_id;
    bbox.header.stamp = now();
    bbox.ns = "stair_bbox";
    bbox.id = id;
    bbox.type = visualization_msgs::msg::Marker::CUBE;
    bbox.action = visualization_msgs::msg::Marker::ADD;
    
    bbox.pose.position.x = candidate.obb_center.x();
    bbox.pose.position.y = candidate.obb_center.y();
    bbox.pose.position.z = candidate.obb_center.z();
    bbox.pose.orientation.x = candidate.obb_orientation.x();
    bbox.pose.orientation.y = candidate.obb_orientation.y();
    bbox.pose.orientation.z = candidate.obb_orientation.z();
    bbox.pose.orientation.w = candidate.obb_orientation.w();
    
    bbox.scale.x = std::max(0.05f, candidate.obb_dims.x());
    bbox.scale.y = std::max(0.05f, candidate.obb_dims.y());
    bbox.scale.z = std::max(0.05f, candidate.obb_dims.z());
    
    if (is_locked) {
        bbox.color.r = 0.0f;
        bbox.color.g = 1.0f;  // 绿色：已锁定
        bbox.color.b = 0.0f;
    } else {
        bbox.color.r = 1.0f;  // 黄色：检测中
        bbox.color.g = 1.0f;
        bbox.color.b = 0.0f;
    }
    bbox.color.a = 0.7f;
    bbox.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    markers.markers.push_back(bbox);
    
    // 中心点
    visualization_msgs::msg::Marker center;
    center.header = bbox.header;
    center.ns = "stair_center";
    center.id = id;
    center.type = visualization_msgs::msg::Marker::SPHERE;
    center.action = visualization_msgs::msg::Marker::ADD;
    
    center.pose.position.x = candidate.center.x();
    center.pose.position.y = candidate.center.y();
    center.pose.position.z = candidate.center.z();
    center.pose.orientation.w = 1.0;
    
    center.scale.x = center.scale.y = center.scale.z = 0.2;
    center.color = bbox.color;
    center.color.a = 1.0f;
    center.lifetime = bbox.lifetime;
    
    markers.markers.push_back(center);
    
    // 前沿线
    visualization_msgs::msg::Marker edge_line;
    edge_line.header = bbox.header;
    edge_line.ns = "stair_edge";
    edge_line.id = id;
    edge_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    edge_line.action = visualization_msgs::msg::Marker::ADD;
    
    geometry_msgs::msg::Point p1, p2;
    p1.x = candidate.edge_x;
    p1.y = candidate.bbox_min.y();
    p1.z = candidate.top_z;
    
    p2.x = candidate.edge_x;
    p2.y = candidate.bbox_max.y();
    p2.z = candidate.top_z;
    
    edge_line.points.push_back(p1);
    edge_line.points.push_back(p2);
    
    edge_line.scale.x = 0.05;  // 线宽
    edge_line.color.r = 1.0f;  // 红色
    edge_line.color.g = 0.0f;
    edge_line.color.b = 0.0f;
    edge_line.color.a = 1.0f;
    edge_line.lifetime = bbox.lifetime;
    
    markers.markers.push_back(edge_line);
    
    // 文字标签
    visualization_msgs::msg::Marker text;
    text.header = bbox.header;
    text.ns = "stair_label";
    text.id = id;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    
    text.pose.position.x = candidate.center.x();
    text.pose.position.y = candidate.center.y();
    text.pose.position.z = candidate.top_z + 0.4;
    text.pose.orientation.w = 1.0;
    
    text.text = (candidate.type == StairType::SINGLE) ? "Single " : "Double";
    text.scale.z = 0.3;
    text.color.r = text.color.g = text.color.b = 1.0f;
    text.color.a = 1.0f;
    text.lifetime = bbox.lifetime;
    
    markers.markers.push_back(text);

    // 包围盒线框（更醒目）
    visualization_msgs::msg::Marker wire;
    wire.header = bbox.header;
    wire.ns = "stair_bbox_wire";
    wire.id = id;
    wire.type = visualization_msgs::msg::Marker::LINE_LIST;
    wire.action = visualization_msgs::msg::Marker::ADD;
    wire.scale.x = 0.03;
    wire.color.r = 1.0f;
    wire.color.g = 0.0f;
    wire.color.b = 1.0f;  // 紫色线框
    wire.color.a = 1.0f;
    wire.lifetime = bbox.lifetime;

    const Eigen::Vector3f half = 0.5f * candidate.obb_dims;
    const Eigen::Matrix3f rot = candidate.obb_orientation.toRotationMatrix();
    const Eigen::Vector3f obb_center = candidate.obb_center;

    std::array<Eigen::Vector3f, 8> corners = {
        Eigen::Vector3f(-half.x(), -half.y(), -half.z()),
        Eigen::Vector3f( half.x(), -half.y(), -half.z()),
        Eigen::Vector3f( half.x(),  half.y(), -half.z()),
        Eigen::Vector3f(-half.x(),  half.y(), -half.z()),
        Eigen::Vector3f(-half.x(), -half.y(),  half.z()),
        Eigen::Vector3f( half.x(), -half.y(),  half.z()),
        Eigen::Vector3f( half.x(),  half.y(),  half.z()),
        Eigen::Vector3f(-half.x(),  half.y(),  half.z())
    };

    auto to_point = [&](const Eigen::Vector3f& local) {
        Eigen::Vector3f p = (rot * local).eval();
        p += obb_center;
        geometry_msgs::msg::Point pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        return pt;
    };

    auto add_edge = [&wire, &to_point](int i, int j,
                                       const std::array<Eigen::Vector3f, 8>& c) {
        wire.points.push_back(to_point(c[i]));
        wire.points.push_back(to_point(c[j]));
    };

    // 底面
    add_edge(0, 1, corners);
    add_edge(1, 2, corners);
    add_edge(2, 3, corners);
    add_edge(3, 0, corners);
    // 顶面
    add_edge(4, 5, corners);
    add_edge(5, 6, corners);
    add_edge(6, 7, corners);
    add_edge(7, 4, corners);
    // 立柱
    add_edge(0, 4, corners);
    add_edge(1, 5, corners);
    add_edge(2, 6, corners);
    add_edge(3, 7, corners);

    markers.markers.push_back(wire);
}

void StairDetector::publishStairTarget() {
    rog_map_ros2_node::msg::StairTarget msg;
    msg.header.stamp = now();
    msg.header.frame_id = cfg_.target_frame;
    
    // 根据状态填充消息
    if (current_state_ == DetectionState::IDLE) {
        msg.status = rog_map_ros2_node::msg::StairTarget::STATUS_IDLE;
        msg.confidence = 0.0f;
    } else if (current_state_ == DetectionState::DETECTED) {
        msg.status = rog_map_ros2_node::msg::StairTarget::STATUS_DETECTED;
        msg.center.x = locked_stair_.center.x();
        msg.center.y = locked_stair_.center.y();
        msg.center.z = locked_stair_.center.z();
        msg.edge_distance = locked_stair_.edge_x;
        msg.height = locked_stair_.height;
        msg.width = locked_stair_.width;
        msg.type = static_cast<uint8_t>(locked_stair_.type);
        msg.confidence = std::min(1.0f, consecutive_detection_count_ / 
                                   static_cast<float>(cfg_.min_detection_frames));
    } else if (current_state_ == DetectionState::LOCKED) {
        msg.status = in_blind_zone_ ? 
            rog_map_ros2_node::msg::StairTarget::STATUS_BLIND_ZONE :
            rog_map_ros2_node::msg::StairTarget::STATUS_LOCKED;
        
        msg.center.x = locked_stair_.center.x();
        msg.center.y = locked_stair_.center.y();
        msg.center.z = locked_stair_.center.z();
        msg.edge_distance = locked_stair_.edge_x;
        msg.height = locked_stair_.height;
        msg.width = locked_stair_.width;
        msg.type = static_cast<uint8_t>(locked_stair_.type);
        msg.confidence = 1.0f;
    }
    
    target_pub_->publish(msg);
}

bool StairDetector::transformPointCloud(
    const PointCloud::Ptr& cloud_in,
    PointCloud::Ptr& cloud_out,
    const std::string& target_frame) {
    
    try {
        // 查询 TF 变换
        geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer_->lookupTransform(
                target_frame,
                cfg_.map_frame,
                tf2::TimePointZero);
        
        // 手动转换点云
        cloud_out->points.resize(cloud_in->size());
        cloud_out->width = cloud_in->width;
        cloud_out->height = cloud_in->height;
        cloud_out->is_dense = cloud_in->is_dense;
        
        // 提取变换并转为 Eigen
        auto& t = transform_stamped.transform.translation;
        auto& q = transform_stamped.transform.rotation;
        
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        Eigen::Vector3d trans(t.x, t.y, t.z);
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.translate(trans);
        transform.rotate(quat);
        
        for (size_t i = 0; i < cloud_in->size(); ++i) {
            Eigen::Vector3d pt_in(cloud_in->points[i].x,
                                   cloud_in->points[i].y,
                                   cloud_in->points[i].z);
            Eigen::Vector3d pt_out = transform * pt_in;
            
            cloud_out->points[i].x = pt_out.x();
            cloud_out->points[i].y = pt_out.y();
            cloud_out->points[i].z = pt_out.z();
        }
        
        return true;
    } catch (const tf2::TransformException& ex) {
        return false;
    }
}

void StairDetector::publishPrefilterCloud() {
    if (!prefilter_cloud_pub_ || !last_prefilter_cloud_ || last_prefilter_cloud_->empty()) {
        return;
    }
    
    // 方法1: 发布为点云消息（保留原有功能）
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*last_prefilter_cloud_, cloud_msg);
    cloud_msg.header.stamp = now();
    cloud_msg.header.frame_id = cfg_.target_frame;
    prefilter_cloud_pub_->publish(cloud_msg);
    
    // 方法2: 用球形 Marker 可视化（添加到 markers）
    if (!marker_pub_) {
        return;
    }
    
    visualization_msgs::msg::Marker spheres;
    spheres.header.frame_id = cfg_.target_frame;
    spheres.header.stamp = now();
    spheres.ns = "prefilter_cloud_spheres";
    spheres.id = 9999;  // 使用特殊ID避免冲突
    spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    spheres.action = visualization_msgs::msg::Marker::ADD;
    
    // 球形大小
    spheres.scale.x = 0.04;  // 球形直径 4cm
    spheres.scale.y = 0.04;
    spheres.scale.z = 0.04;
    
    // 颜色：明亮的黄色
    spheres.color.r = 1.0f;
    spheres.color.g = 1.0f;
    spheres.color.b = 0.0f;
    spheres.color.a = 0.8f;  // 半透明
    
    spheres.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    // 添加所有预筛选点
    spheres.points.reserve(last_prefilter_cloud_->size());
    for (const auto& pt : last_prefilter_cloud_->points) {
        geometry_msgs::msg::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        spheres.points.push_back(p);
    }
    
    // 发布独立的 MarkerArray
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(spheres);
    marker_pub_->publish(marker_array);
}

rcl_interfaces::msg::SetParametersResult StairDetector::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : parameters) {
        const std::string& name = param.get_name();
        
        // === ROI 参数 ===
        if (name == "roi_x_min") cfg_.roi_x_min = param.as_double();
        else if (name == "roi_x_max") cfg_.roi_x_max = param.as_double();
        else if (name == "roi_y_min") cfg_.roi_y_min = param.as_double();
        else if (name == "roi_y_max") cfg_.roi_y_max = param.as_double();
        else if (name == "roi_z_min") cfg_.roi_z_min = param.as_double();
        else if (name == "roi_z_max") cfg_.roi_z_max = param.as_double();
        
        // === 聚类参数 ===
        else if (name == "cluster_tolerance") cfg_.cluster_tolerance = param.as_double();
        else if (name == "min_cluster_size") cfg_.min_cluster_size = param.as_int();
        else if (name == "max_cluster_size") cfg_.max_cluster_size = param.as_int();
        
        // === 几何约束参数 ===
        else if (name == "single_stair_height") cfg_.single_stair_height = param.as_double();
        else if (name == "double_stair_height") cfg_.double_stair_height = param.as_double();
        else if (name == "height_tolerance") cfg_.height_tolerance = param.as_double();
        else if (name == "min_stair_width") cfg_.min_stair_width = param.as_double();
        else if (name == "min_stair_depth") cfg_.min_stair_depth = param.as_double();
        else if (name == "max_z_thickness") cfg_.max_z_thickness = param.as_double();
        
        // === 法向量估计参数 ===
        else if (name == "enable_normal_estimation") cfg_.enable_normal_estimation = param.as_bool();
        else if (name == "min_planarity") cfg_.min_planarity = param.as_double();
        else if (name == "horizontal_normal_z_min") cfg_.horizontal_normal_z_min = param.as_double();
        else if (name == "horizontal_points_ratio_min") cfg_.horizontal_points_ratio_min = param.as_double();
        else if (name == "normal_min_points") cfg_.normal_min_points = param.as_int();
        
        // === 平面分割参数 ===
        else if (name == "enable_plane_segmentation") cfg_.enable_plane_segmentation = param.as_bool();
        else if (name == "ransac_distance_threshold") cfg_.ransac_distance_threshold = param.as_double();
        else if (name == "ransac_max_iterations") cfg_.ransac_max_iterations = param.as_int();
        else if (name == "min_plane_points") cfg_.min_plane_points = param.as_int();
        else if (name == "ground_plane_z_tolerance") cfg_.ground_plane_z_tolerance = param.as_double();
        else if (name == "max_planes") cfg_.max_planes = param.as_int();
        
        // === 预筛选参数 ===
        else if (name == "cell_size_xy") cfg_.cell_size_xy = param.as_double();
        else if (name == "min_cell_points") cfg_.min_cell_points = param.as_int();
        else if (name == "min_cell_height") cfg_.min_cell_height = param.as_double();
        else if (name == "max_cell_height") cfg_.max_cell_height = param.as_double();
        else if (name == "max_cell_top_z") cfg_.max_cell_top_z = param.as_double();
        
        // === 跟踪参数 ===
        else if (name == "min_detection_frames") cfg_.min_detection_frames = param.as_int();
        else if (name == "lowpass_alpha") cfg_.lowpass_alpha = param.as_double();
        else if (name == "blind_zone_distance") cfg_.blind_zone_distance = param.as_double();
        
        // 参数更新日志
        RCLCPP_INFO(get_logger(), "Parameter updated: %s = %s", 
                    name.c_str(), param.value_to_string().c_str());
    }
    
    return result;
}

} // namespace stair_detector

