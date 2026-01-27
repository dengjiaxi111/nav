/**
 * @file map_2d_projector.cpp
 * @brief 3D→2D地图投影实现
 */

#include "rog_map_ros2_node/map_2d_projector.hpp"
#include <algorithm>
#include <cmath>

namespace map_2d_projector {

Map2DProjector::Map2DProjector(rclcpp::Node::SharedPtr node, const ProjectorConfig& cfg)
    : node_(node), cfg_(cfg) {
    
    // 初始化2D地图
    initializeMap();
    
    // 初始化TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // QoS配置 - 与ROG-Map一致使用best_effort
    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1));
    
    // 订阅ROG-Map的膨胀占据点云
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rog_map/inf_occ", qos,
        std::bind(&Map2DProjector::cloudCallback, this, std::placeholders::_1));
    
    // 发布2D地图
    map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        cfg_.topic_name, qos);
    
    // 定时发布器
    int publish_period_ms = static_cast<int>(1000.0 / cfg_.publish_rate);
    publish_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(publish_period_ms),
        std::bind(&Map2DProjector::publishTimerCallback, this));
    
    RCLCPP_INFO(node_->get_logger(), "Map2DProjector initialized");
    RCLCPP_INFO(node_->get_logger(), "  Resolution: %.3f m", cfg_.resolution);
    RCLCPP_INFO(node_->get_logger(), "  Map range: %.1fx%.1f m", cfg_.map_range_x, cfg_.map_range_y);
    RCLCPP_INFO(node_->get_logger(), "  Height thresholds: slope<%.2f, step[%.2f-%.2f], obs>%.2f",
                cfg_.slope_height_max, cfg_.step_height_min, cfg_.step_height_max, cfg_.obstacle_height_min);
    RCLCPP_INFO(node_->get_logger(), "  High occupancy threshold: %.2f", cfg_.high_occupancy_thresh);
}

void Map2DProjector::initializeMap() {
    map_2d_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    
    // 计算地图尺寸（栅格数）
    int width = static_cast<int>(std::ceil(cfg_.map_range_x / cfg_.resolution));
    int height = static_cast<int>(std::ceil(cfg_.map_range_y / cfg_.resolution));
    
    map_2d_->header.frame_id = cfg_.frame_id;
    map_2d_->info.resolution = cfg_.resolution;
    map_2d_->info.width = width;
    map_2d_->info.height = height;
    
    // 初始化为unknown
    map_2d_->data.resize(width * height, cfg_.unknown_value);
    
    RCLCPP_INFO(node_->get_logger(), "  Map size: %dx%d cells", width, height);
}

void Map2DProjector::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    latest_cloud_ = msg;
}

bool Map2DProjector::getRobotPose(geometry_msgs::msg::Pose& robot_pose) {
    try {
        // 查询 base_link -> odom 的TF变换
        geometry_msgs::msg::TransformStamped transform_stamped = 
            tf_buffer_->lookupTransform(
                cfg_.frame_id,        // target frame (odom)
                "base_link",          // source frame
                tf2::TimePointZero);  // 获取最新的变换
        
        // 提取位姿
        robot_pose.position.x = transform_stamped.transform.translation.x;
        robot_pose.position.y = transform_stamped.transform.translation.y;
        robot_pose.position.z = transform_stamped.transform.translation.z;
        robot_pose.orientation = transform_stamped.transform.rotation;
        
        // 更新缓存
        robot_position_ = Vec3f(
            robot_pose.position.x,
            robot_pose.position.y,
            robot_pose.position.z);
        
        return true;
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,  // 每5秒警告一次
            "Could not get transform from base_link to %s: %s",
            cfg_.frame_id.c_str(), ex.what());
        return false;
    }
}

void Map2DProjector::publishTimerCallback() {
    if (!latest_cloud_) {
        return;
    }
    
    // 获取机器人位姿
    geometry_msgs::msg::Pose robot_pose;
    if (!getRobotPose(robot_pose)) {
        return;  // TF查询失败，跳过本次更新
    }
    
    // 转换点云
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*latest_cloud_, pcl_cloud);
    
    if (pcl_cloud.empty()) {
        return;
    }
    
    // Step 1: 高程分析
    std::unordered_map<int64_t, ColumnMetrics> columns;
    elevationAnalysis(pcl_cloud, robot_position_.z(), columns);
    
    // Step 2: 更新2D地图
    update2DMap(columns, robot_position_);
    
    // Step 3: 发布地图
    publishMap();
}

void Map2DProjector::elevationAnalysis(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    float robot_z,
    std::unordered_map<int64_t, ColumnMetrics>& columns) {
    
    // 计算实际的高度范围（相对于机器人当前z）
    float z_min = robot_z + cfg_.z_min_relative;
    float z_max = robot_z + cfg_.z_max_relative;
    
    // 临时存储每个柱的z值列表
    std::unordered_map<int64_t, std::vector<float>> column_z_values;
    
    // 收集每个柱的所有z值
    for (const auto& pt : cloud.points) {
        // 过滤超出高度范围的点
        if (pt.z < z_min || pt.z > z_max) {
            continue;
        }
        
        // 计算grid key
        int64_t key = xyToGridKey(pt.x, pt.y);
        column_z_values[key].push_back(pt.z);
    }
    
    // 计算每列的高程统计量（中科大方案：高度差H + 占据率）
    for (auto& [key, z_values] : column_z_values) {
        auto& col = columns[key];
        col.point_count = static_cast<int>(z_values.size());
        
        if (col.point_count == 0) continue;
        
        // 计算 min_z, max_z, height_diff
        col.min_z = *std::min_element(z_values.begin(), z_values.end());
        col.max_z = *std::max_element(z_values.begin(), z_values.end());
        col.height_diff = col.max_z - col.min_z;
        
        // 计算占据率：((n+1)*res)/H
        if (col.height_diff > 1e-3f) {
            float numerator = (col.point_count + 1) * cfg_.resolution;
            col.occupancy_rate = numerator / col.height_diff;
        } else {
            col.occupancy_rate = 1.0f;  // 高度差极小，视为满占据
        }
    }
}

int8_t Map2DProjector::classifyColumn(const ColumnMetrics& metrics, float robot_z) {
    /*
     * 基于中科大方案 + RM场景分析
     * 
     * 核心特征：高度差H + 占据率
     * 
     * 场景分类：
     * 1. 坡道/平面（可通行）：H很小（<0.1m），占据率高
     * 2. 台阶（可通行）：0.1m < H < 0.2m，两侧等高（TODO：需要邻域分析）
     * 3. 机器人/人（不可通行）：H > 0.3m，占据率高
     * 4. 矮墙/悬崖（不可通行）：0.1m < H < 0.2m，但一侧不等高（非台阶）
     * 5. 限高杆/悬空（可通行）：完全高于机器人
     */
    
    const float H = metrics.height_diff;
    const float occ_rate = metrics.occupancy_rate;
    
    // 计算关键高度（修正：基于base_link向下测量）
    // robot_z 是 base_link 在odom系下的z坐标
    // cfg_.base_to_ground: base_link到机器人底部（轮子底面）的距离
    // 
    // 物理意义：
    //   robot_bottom = 机器人底部（轮子底面）的绝对高度
    //   robot_top = 机器人顶部的绝对高度
    //   ground_level = 地面参考高度（略高于robot_bottom，用于过滤贴地噪点）
    float robot_bottom = robot_z - cfg_.base_to_ground;    // base_link下方，即轮子底面
    float robot_top = robot_bottom + cfg_.robot_height;     // 从底部向上测量总高度
    float ground_level = robot_bottom + cfg_.ground_tolerance;  // 地面=底部+小容差
    
    // 调试日志（每1000个栅格采样一次，避免刷屏）
    static int debug_counter = 0;
    bool should_log = cfg_.enable_debug_log && (debug_counter++ % 1000 == 0);
    
    if (should_log) {
        RCLCPP_INFO(node_->get_logger(),
            "[Classify Debug] H=%.3f, occ=%.2f, min_z=%.3f, max_z=%.3f | "
            "robot_z=%.3f, robot_bottom=%.3f, robot_top=%.3f, ground_level=%.3f",
            H, occ_rate, metrics.min_z, metrics.max_z,
            robot_z, robot_bottom, robot_top, ground_level);
    }
    
    // === Case 1: 噪声过滤 ===
    if (metrics.point_count < 2) {
        if (should_log) RCLCPP_INFO(node_->get_logger(), "  → Case1: Noise (count=%d)", metrics.point_count);
        return cfg_.free_value;
    }
    
    // === Case 2: 完全悬空（限高杆、悬空物体）===
    if (metrics.min_z > robot_top) {
        if (should_log) RCLCPP_INFO(node_->get_logger(), 
            "  → Case2: Overhead (min_z=%.3f > robot_top=%.3f)", metrics.min_z, robot_top);
        return cfg_.free_value;  // 机器人可从下方通过
    }
    
    // === Case 3: 坡道/平面（H很小）===
    if (H < cfg_.slope_height_max) {
        // 高度差小于0.1m，判断为平面或缓坡
        
        // 3.1 完全低于机器人底部 → 地面
        if (metrics.max_z < ground_level) {
            if (should_log) RCLCPP_INFO(node_->get_logger(), 
                "  → Case3.1: Ground (max_z=%.3f < ground_level=%.3f)", metrics.max_z, ground_level);
            return cfg_.free_value;
        }
        
        // 3.2 与机器人高度重叠 → 检查是否会阻挡
        if (metrics.max_z < robot_top) {
            // 最高点低于机器人顶部，不会完全阻挡
            // 检查是否为小障碍（如地面凸起）
            float height_above_ground = metrics.max_z - robot_bottom;
            if (height_above_ground < cfg_.slope_height_max) {
                if (should_log) RCLCPP_INFO(node_->get_logger(), 
                    "  → Case3.2: Small bump (height_above_ground=%.3f < %.3f)", 
                    height_above_ground, cfg_.slope_height_max);
                return cfg_.free_value;  // 小凸起，可通行
            }
        }
        
        // 3.3 完全高于机器人 → 悬空平台
        if (metrics.min_z > robot_top) {
            if (should_log) RCLCPP_INFO(node_->get_logger(), 
                "  → Case3.3: Overhead platform (already checked in Case2)");
            return cfg_.free_value;
        }
        
        // 默认：H小的都视为可通行
        if (should_log) RCLCPP_INFO(node_->get_logger(), "  → Case3.default: Slope/Flat (H=%.3f)", H);
        return cfg_.free_value;
    }
    
    // === Case 4: 台阶范围（0.1m < H < 0.2m）===
    if (H >= cfg_.step_height_min && H < cfg_.step_height_max) {
        // 台阶高度区间，需要更细致判断
        
        // 4.1 检查是否在可通行高度范围
        bool blocks_passage = (metrics.min_z < robot_top) && 
                              (metrics.max_z > ground_level);
        
        if (!blocks_passage) {
            if (should_log) RCLCPP_INFO(node_->get_logger(), 
                "  → Case4.1: Step not blocking (blocks_passage=false)");
            return cfg_.free_value;  // 不阻挡通行高度
        }
        
        // 4.2 当前简化版本：暂时保守标记为obstacle
        // TODO: 需要邻域分析判断是台阶（两侧等高）还是矮墙（一侧悬空）
        
        // 当前简化：如果占据率高（密集点云），可能是墙
        if (occ_rate > cfg_.high_occupancy_thresh) {
            if (should_log) RCLCPP_INFO(node_->get_logger(), 
                "  → Case4.2: Dense step/wall (occ=%.2f > %.2f) → OBSTACLE", 
                occ_rate, cfg_.high_occupancy_thresh);
            return cfg_.obstacle_value;  // 密集垂直结构，可能是墙
        }
        
        // 占据率低，可能是稀疏障碍或台阶，保守处理
        if (should_log) RCLCPP_INFO(node_->get_logger(), 
            "  → Case4.3: Sparse step (occ=%.2f) → OBSTACLE", occ_rate);
        return cfg_.obstacle_value;
    }
    
    // === Case 5: 高障碍物（H > 0.3m）===
    if (H >= cfg_.obstacle_height_min) {
        // 机器人、人、墙等高大障碍物
        
        // 5.1 完全高于机器人 → 悬空（已在Case 2处理）
        // 这里到达说明min_z <= robot_top
        
        // 5.2 与机器人高度重叠 → 检查占据率
        bool blocks_passage = (metrics.min_z < robot_top) && 
                              (metrics.max_z > ground_level);
        
        if (blocks_passage) {
            // 高大且阻挡通行 → 障碍物
            if (should_log) RCLCPP_INFO(node_->get_logger(), 
                "  → Case5: High obstacle (H=%.3f, blocks_passage=true) → OBSTACLE", H);
            return cfg_.obstacle_value;
        }
        
        // 不阻挡通行高度（如地面深坑？）
        if (should_log) RCLCPP_INFO(node_->get_logger(), 
            "  → Case5: High but not blocking → FREE");
        return cfg_.free_value;
    }
    
    // === Case 6: 中间范围（0.2m < H < 0.3m）===
    // 介于台阶和高障碍物之间，保守处理
    bool blocks_passage = (metrics.min_z < robot_top) && 
                          (metrics.max_z > ground_level);
    
    if (blocks_passage) {
        if (should_log) RCLCPP_INFO(node_->get_logger(), 
            "  → Case6: Medium obstacle (H=%.3f) → OBSTACLE", H);
        return cfg_.obstacle_value;
    }
    
    if (should_log) RCLCPP_INFO(node_->get_logger(), 
        "  → Case6.default: Medium but not blocking → FREE");
    return cfg_.free_value;
}

void Map2DProjector::update2DMap(
    const std::unordered_map<int64_t, ColumnMetrics>& columns,
    const Vec3f& robot_pos) {
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 固定地图原点到栅格对齐的位置（避免抖动）
    // 计算机器人所在栅格的中心作为参考点
    float center_x = std::floor(robot_pos.x() / cfg_.resolution) * cfg_.resolution;
    float center_y = std::floor(robot_pos.y() / cfg_.resolution) * cfg_.resolution;
    
    // 地图原点固定在栅格对齐的位置
    float half_range_x = cfg_.map_range_x / 2.0f;
    float half_range_y = cfg_.map_range_y / 2.0f;
    
    // 对齐到栅格边界
    float origin_x = std::floor((center_x - half_range_x) / cfg_.resolution) * cfg_.resolution;
    float origin_y = std::floor((center_y - half_range_y) / cfg_.resolution) * cfg_.resolution;
    
    map_2d_->info.origin.position.x = origin_x;
    map_2d_->info.origin.position.y = origin_y;
    map_2d_->info.origin.position.z = 0.0;
    map_2d_->info.origin.orientation.w = 1.0;
    
    // 重置地图为unknown
    std::fill(map_2d_->data.begin(), map_2d_->data.end(), cfg_.unknown_value);
    
    // 填充分析结果
    int obstacle_count = 0;
    int free_count = 0;
    int unknown_count = 0;
    
    for (const auto& [key, col] : columns) {
        Vec2f pos = gridKeyToPos(key);
        
        // 计算在地图中的索引（相对于地图原点）
        float local_x = pos.x() - origin_x;
        float local_y = pos.y() - origin_y;
        
        int ix = static_cast<int>(std::floor(local_x / cfg_.resolution));
        int iy = static_cast<int>(std::floor(local_y / cfg_.resolution));
        
        // 边界检查
        if (ix < 0 || ix >= static_cast<int>(map_2d_->info.width) ||
            iy < 0 || iy >= static_cast<int>(map_2d_->info.height)) {
            continue;
        }
        
        // 计算1D索引 (row-major)
        int index = iy * map_2d_->info.width + ix;
        
        // 分类并设置占据值
        int8_t value = classifyColumn(col, robot_pos.z());
        map_2d_->data[index] = value;
        
        // 统计
        if (value == cfg_.obstacle_value) obstacle_count++;
        else if (value == cfg_.free_value) free_count++;
        else unknown_count++;
    }
    
    // 每100帧输出一次统计信息
    static int update_count = 0;
    update_count++;
    if (update_count % 100 == 0) {
        RCLCPP_INFO(node_->get_logger(),
            "[Map2D] Processed %zu columns: %d obstacles, %d free, %d unknown",
            columns.size(), obstacle_count, free_count, unknown_count);
    }
}

void Map2DProjector::publishMap() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_2d_->header.stamp = node_->get_clock()->now();
    map_pub_->publish(*map_2d_);
}

nav_msgs::msg::OccupancyGrid::SharedPtr Map2DProjector::getLatestMap() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return map_2d_;
}

void Map2DProjector::processPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud,
    const geometry_msgs::msg::Pose& robot_pose) {
    
    // 转换点云
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*cloud, pcl_cloud);
    
    Vec3f robot_pos(
        robot_pose.position.x,
        robot_pose.position.y,
        robot_pose.position.z);
    
    // 高程分析
    std::unordered_map<int64_t, ColumnMetrics> columns;
    elevationAnalysis(pcl_cloud, robot_pos.z(), columns);
    
    // 更新2D地图
    update2DMap(columns, robot_pos);
}

} // namespace map_2d_projector
