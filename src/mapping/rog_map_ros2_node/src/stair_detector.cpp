/**
 * @file stair_detector.cpp
 * @brief 台阶检测节点实现
 */

#include "rog_map_ros2_node/stair_detector.hpp"
#include <pcl/kdtree/kdtree.h>
#include <limits>

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
    if (cloud_roi->empty()) {
        // 未检测到任何点云，重置状态
        if (current_state_ != DetectionState::IDLE) {
            RCLCPP_INFO(get_logger(), "Lost sight of stair, resetting to IDLE");
            current_state_ = DetectionState::IDLE;
            consecutive_detection_count_ = 0;
        }
        return;
    }
    
    // Step 2: 欧式聚类
    auto clusters = euclideanClustering(cloud_roi);
    if (clusters.empty()) {
        if (current_state_ != DetectionState::IDLE) {
            RCLCPP_WARN(get_logger(), "No clusters found, resetting");
            current_state_ = DetectionState::IDLE;
            consecutive_detection_count_ = 0;
        }
        return;
    }
    
    // Step 3: 硬约束筛选
    auto candidates = filterStairCandidates(clusters);
    
    // Step 4 & 5: 精炼候选并更新跟踪
    updateTracking(candidates);
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
    
    return clusters;
}

std::vector<StairCandidate> StairDetector::filterStairCandidates(
    const std::vector<PointCloud::Ptr>& clusters) {
    
    std::vector<StairCandidate> candidates;
    
    for (const auto& cluster : clusters) {
        StairCandidate candidate;
        candidate.cloud = cluster;
        
        // 计算包围盒
        PointT min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);
        
        candidate.bbox_min = Vec3f(min_pt.x, min_pt.y, min_pt.z);
        candidate.bbox_max = Vec3f(max_pt.x, max_pt.y, max_pt.z);
        
        // 几何特征
        candidate.height = max_pt.z;  // 顶面高度
        candidate.width = max_pt.y - min_pt.y;
        candidate.depth = max_pt.x - min_pt.x;
        
        float z_thickness = max_pt.z - min_pt.z;
        
        // === 硬约束判定 ===
        
        // 1. 高度校验：匹配一级或二级台阶
        bool is_single = std::abs(candidate.height - cfg_.single_stair_height) < cfg_.height_tolerance;
        bool is_double = std::abs(candidate.height - cfg_.double_stair_height) < cfg_.height_tolerance;
        
        if (!is_single && !is_double) {
            continue;  // 高度不匹配，丢弃
        }
        
        candidate.type = is_single ? StairType::SINGLE : StairType::DOUBLE;
        
        // 2. 形态校验：宽度、深度、厚度
        if (candidate.width < cfg_.min_stair_width) {
            continue;  // 太窄
        }
        
        if (candidate.depth < cfg_.min_stair_depth) {
            continue;  // 太浅
        }
        
        if (z_thickness > cfg_.max_z_thickness) {
            continue;  // 太厚，可能是墙壁
        }
        
        // 3. 精炼边缘和中心
        refineStairCandidate(candidate);
        
        candidates.push_back(candidate);
    }
    
    return candidates;
}

void StairDetector::refineStairCandidate(StairCandidate& candidate) {
    // 计算中心（使用 X/Y 的中点，Z 使用顶面高度）
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*candidate.cloud, centroid);
    
    candidate.center = Vec3f(centroid[0], centroid[1], candidate.height);
    
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
    
    // 如果有锁定的台阶，绘制
    if (current_state_ == DetectionState::LOCKED || current_state_ == DetectionState::DETECTED) {
        addCandidateMarker(markers, locked_stair_, 0,
                           current_state_ == DetectionState::LOCKED);
    }
    
    marker_pub_->publish(markers);
}

void StairDetector::addCandidateMarker(
    visualization_msgs::msg::MarkerArray& markers,
    const StairCandidate& candidate,
    int id,
    bool is_locked) {
    
    // 包围盒
    visualization_msgs::msg::Marker bbox;
    bbox.header.frame_id = cfg_.target_frame;
    bbox.header.stamp = now();
    bbox.ns = "stair_bbox";
    bbox.id = id;
    bbox.type = visualization_msgs::msg::Marker::CUBE;
    bbox.action = visualization_msgs::msg::Marker::ADD;
    
    bbox.pose.position.x = (candidate.bbox_min.x() + candidate.bbox_max.x()) / 2.0;
    bbox.pose.position.y = (candidate.bbox_min.y() + candidate.bbox_max.y()) / 2.0;
    bbox.pose.position.z = (candidate.bbox_min.z() + candidate.bbox_max.z()) / 2.0;
    bbox.pose.orientation.w = 1.0;
    
    bbox.scale.x = candidate.bbox_max.x() - candidate.bbox_min.x();
    bbox.scale.y = candidate.bbox_max.y() - candidate.bbox_min.y();
    bbox.scale.z = candidate.bbox_max.z() - candidate.bbox_min.z();
    
    if (is_locked) {
        bbox.color.r = 0.0f;
        bbox.color.g = 1.0f;  // 绿色：已锁定
        bbox.color.b = 0.0f;
    } else {
        bbox.color.r = 1.0f;  // 黄色：检测中
        bbox.color.g = 1.0f;
        bbox.color.b = 0.0f;
    }
    bbox.color.a = 0.3f;
    bbox.lifetime = rclcpp::Duration::from_seconds(0.2);
    
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
    
    center.scale.x = center.scale.y = center.scale.z = 0.1;
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
    p1.z = candidate.height;
    
    p2.x = candidate.edge_x;
    p2.y = candidate.bbox_max.y();
    p2.z = candidate.height;
    
    edge_line.points.push_back(p1);
    edge_line.points.push_back(p2);
    
    edge_line.scale.x = 0.02;  // 线宽
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
    text.pose.position.z = candidate.height + 0.2;
    text.pose.orientation.w = 1.0;
    
    text.text = (candidate.type == StairType::SINGLE) ? "Single (15cm)" : "Double (35cm)";
    text.scale.z = 0.15;
    text.color.r = text.color.g = text.color.b = 1.0f;
    text.color.a = 1.0f;
    text.lifetime = bbox.lifetime;
    
    markers.markers.push_back(text);
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
