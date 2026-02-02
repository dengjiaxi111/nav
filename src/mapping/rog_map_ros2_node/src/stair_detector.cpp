/**
 * @file stair_detector.cpp
 * @brief 台阶检测节点实现
 */

#include "rog_map_ros2_node/stair_detector.hpp"
#include <pcl/kdtree/kdtree.h>
#include <limits>
#include <array>
#include <unordered_map>
#include <cmath>
#include <numeric>
#include <sstream>
#include <iomanip>

namespace stair_detector {

StairDetector::StairDetector(const rclcpp::NodeOptions& options)
    : Node("stair_detector", options),
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
    
    // QoS 配置 - 与 ROG-Map 一致
    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1));
    
    // 订阅 ROG-Map 占据点云
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg_.input_cloud_topic, qos,
        std::bind(&StairDetector::cloudCallback, this, std::placeholders::_1));
    
    // 发布台阶目标
    target_pub_ = create_publisher<rog_map_ros2_node::msg::StairTarget>(
        cfg_.output_target_topic, 10);
    
    // 发布可视化 Marker
    if (cfg_.enable_visualization) {
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            cfg_.output_marker_topic, 10);
    }
    
    RCLCPP_INFO(get_logger(), "=== Stair Detector Initialized ===");
    RCLCPP_INFO(get_logger(), "  Input: %s", cfg_.input_cloud_topic.c_str());
    RCLCPP_INFO(get_logger(), "  Output: %s", cfg_.output_target_topic.c_str());
    RCLCPP_INFO(get_logger(), "  ROI: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]",
                cfg_.roi_x_min, cfg_.roi_x_max,
                cfg_.roi_y_min, cfg_.roi_y_max,
                cfg_.roi_z_min, cfg_.roi_z_max);
    RCLCPP_INFO(get_logger(), "  Stair Heights: Single=%.2fm, Double=%.2fm (±%.2fm)",
                cfg_.single_stair_height, cfg_.double_stair_height, cfg_.height_tolerance);
    RCLCPP_INFO(get_logger(), "  Cluster: tolerance=%.3fm, size=[%d, %d]",
                cfg_.cluster_tolerance, cfg_.min_cluster_size, cfg_.max_cluster_size);
    
    last_process_time_ = now();
}

void StairDetector::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto t_start = std::chrono::high_resolution_clock::now();
    
    // 转换为 PCL 点云
    PointCloud::Ptr cloud_odom(new PointCloud);
    pcl::fromROSMsg(*msg, *cloud_odom);
    
    if (cloud_odom->empty()) {
        return;
    }
    
    // 转换到 base_link 坐标系
    PointCloud::Ptr cloud_base(new PointCloud);
    if (!transformPointCloud(cloud_odom, cloud_base, cfg_.target_frame)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Failed to transform point cloud to %s", cfg_.target_frame.c_str());
        return;
    }
    
    // 执行检测
    processPointCloud(cloud_base);
    
    // 发布结果
    publishStairTarget();
    
    if (cfg_.enable_visualization) {
        publishVisualization();
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
        if (current_state_ != DetectionState::IDLE) {
            RCLCPP_WARN(get_logger(), "No stair-like cells found, resetting");
            current_state_ = DetectionState::IDLE;
            consecutive_detection_count_ = 0;
        }
        return;
    }

    // Step 2: 欧式聚类
    auto clusters = euclideanClustering(cloud_prefilter);
    last_clusters_ = clusters;
    if (clusters.empty()) {
        last_candidates_.clear();
        last_rejected_.clear();
        if (current_state_ != DetectionState::IDLE) {
            RCLCPP_WARN(get_logger(), "No clusters found, resetting");
            current_state_ = DetectionState::IDLE;
            consecutive_detection_count_ = 0;
        }
        return;
    }
    
    // Step 3: 硬约束筛选
    auto candidates = filterStairCandidates(clusters);

    // Step 3.5: 重叠候选筛选（保留综合表现最佳）
    auto filtered_candidates = resolveOverlappingCandidates(candidates);
    last_candidates_ = filtered_candidates;
    
    // Step 4 & 5: 精炼候选并更新跟踪
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
    
    // Z 轴过滤（切掉地面）
    pcl::PassThrough<PointT> pass_z;
    pass_z.setInputCloud(cloud_temp);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(cfg_.roi_z_min, cfg_.roi_z_max);
    pass_z.filter(*cloud_filtered);
    
    return cloud_filtered;
}

PointCloud::Ptr StairDetector::stairLikeFilter(const PointCloud::Ptr& cloud_in) {
    struct CellStats {
        int count = 0;
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::lowest();
    };

    const float cell = std::max(0.01f, cfg_.cell_size_xy);
    std::unordered_map<long long, CellStats> cells;
    cells.reserve(cloud_in->size());

    auto key_from = [cell](float x, float y) {
        const int ix = static_cast<int>(std::floor(x / cell));
        const int iy = static_cast<int>(std::floor(y / cell));
        return (static_cast<long long>(ix) << 32) ^ (static_cast<unsigned int>(iy));
    };

    for (const auto& pt : cloud_in->points) {
        const long long key = key_from(pt.x, pt.y);
        auto& stat = cells[key];
        stat.count += 1;
        stat.min_z = std::min(stat.min_z, pt.z);
        stat.max_z = std::max(stat.max_z, pt.z);
    }

    PointCloud::Ptr filtered(new PointCloud);
    filtered->reserve(cloud_in->size());

    for (const auto& pt : cloud_in->points) {
        const long long key = key_from(pt.x, pt.y);
        const auto& stat = cells[key];
        const float height = stat.max_z - stat.min_z;
        if (stat.count < cfg_.min_cell_points) {
            continue;
        }
        if (stat.max_z > cfg_.max_cell_top_z) {
            continue;
        }
        if (height < cfg_.min_cell_height || height > cfg_.max_cell_height) {
            continue;
        }
        filtered->push_back(pt);
    }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = true;
    return filtered;
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
        
        // 1. 高度校验：匹配一级或二级台阶
        bool is_single = std::abs(candidate.height - cfg_.single_stair_height) < cfg_.height_tolerance;
        bool is_double = std::abs(candidate.height - cfg_.double_stair_height) < cfg_.height_tolerance;
        
        if (!is_single && !is_double) {
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
        
        candidate.type = is_single ? StairType::SINGLE : StairType::DOUBLE;
        
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

} // namespace stair_detector

// 主函数
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stair_detector::StairDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
