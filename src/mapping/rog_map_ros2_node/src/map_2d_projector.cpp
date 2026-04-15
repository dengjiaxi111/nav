/**
 * @file map_2d_projector.cpp
 * @brief 3D→2D地图投影实现（基于中科大高程分析方案）
 * 
 * 核心算法：对每个(x,y)柱进行高度差H + 占据率分析
 * 新增功能：动态腿长支持、台阶邻域检测
 * 
 * **组件模式 + 直接调用**: 继承 rclcpp::Node，通过 rog_map_ptr_ 直接访问 
 *                         ROGMapROS boxSearchInflate()，避免 topic 序列化开销
 */

#include "rog_map_ros2_node/map_2d_projector.hpp"
#include <rog_map_ros/rog_map_ros2.hpp>  // 包含 ROGMapROS 定义
#include <algorithm>
#include <cmath>

namespace map_2d_projector {

using namespace super_utils;  // 访问 vec_E 和其他工具

Map2DProjector::Map2DProjector(const rclcpp::NodeOptions& options,
                               rog_map::ROGMapROS* rog_map_ptr)
    : Node("map_2d_projector", options), 
      rog_map_ptr_(rog_map_ptr),
      current_leg_length_(0.10f),
      current_base_to_ground_(0.10f) {
    
    // 从参数服务器加载配置
    cfg_.robot_height = this->declare_parameter<double>("robot_height", 0.5);
    cfg_.robot_width = this->declare_parameter<double>("robot_width", 0.4);
    cfg_.base_to_ground_default = this->declare_parameter<double>("base_to_ground_default", 0.10);

    cfg_.ground_tolerance = this->declare_parameter<double>("ground_tolerance", 0.08);  // 增加地面容差
    
    cfg_.enable_dynamic_leg_length = this->declare_parameter<bool>("enable_dynamic_leg_length", false);
    cfg_.wheel_frame = this->declare_parameter<std::string>("wheel_frame", "wheel_link");
    cfg_.leg_length_min = this->declare_parameter<double>("leg_length_min", 0.08);
    cfg_.leg_length_max = this->declare_parameter<double>("leg_length_max", 0.25);
    
    cfg_.slope_height_max = this->declare_parameter<double>("slope_height_max", 0.08);
    cfg_.step_height_min = this->declare_parameter<double>("step_height_min", 0.12);
    cfg_.step_height_max = this->declare_parameter<double>("step_height_max", 0.23);
    cfg_.obstacle_height_min = this->declare_parameter<double>("obstacle_height_min", 0.25);
    cfg_.high_occupancy_thresh = this->declare_parameter<double>("high_occupancy_thresh", 0.5);
    
    cfg_.normal_z_slope_thresh = this->declare_parameter<double>("normal_z_slope_thresh", 0.866);
    cfg_.normal_z_wall_thresh = this->declare_parameter<double>("normal_z_wall_thresh", 0.5);
    cfg_.normal_min_points = this->declare_parameter<int>("normal_min_points", 5);
    cfg_.planarity_thresh = this->declare_parameter<double>("planarity_thresh", 0.7);

    cfg_.map_range_x = this->declare_parameter<double>("map_range_x", 10.0);
    cfg_.map_range_y = this->declare_parameter<double>("map_range_y", 10.0);
    cfg_.resolution = this->declare_parameter<double>("resolution", 0.05);
    cfg_.z_min_relative = this->declare_parameter<double>("z_min_relative", -0.5);
    cfg_.z_max_relative = this->declare_parameter<double>("z_max_relative", 2.0);
    
    cfg_.publish_rate = this->declare_parameter<double>("publish_rate", 10.0);
    cfg_.frame_id = this->declare_parameter<std::string>("frame_id", "odom");
    cfg_.topic_name = this->declare_parameter<std::string>("topic_name", "rog_map/map_2d");

    current_leg_length_ = cfg_.base_to_ground_default;
    current_base_to_ground_ = cfg_.base_to_ground_default;
    
    // 参数验证（允许延迟注入）
    if (rog_map_ptr_ == nullptr) {
        RCLCPP_WARN(this->get_logger(), "ROG-Map pointer not set, will need setRogMapPtr() call");
    }
    
    // 初始化2D地图
    initializeMap();
    
    // 初始化TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // QoS配置
    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1));
    
    // 发布2D地图
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        cfg_.topic_name, qos);

    // 可选发布台阶调试可视化
    if (cfg_.enable_step_debug_viz) {
        step_debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            cfg_.step_debug_topic, qos);
    }
    
    // 定时更新器（直接调用 boxSearchInflate）
    int update_period_ms = static_cast<int>(1000.0 / cfg_.publish_rate);
    update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_period_ms),
        std::bind(&Map2DProjector::updateTimerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Map2DProjector initialized (component + direct-call mode)");
    RCLCPP_INFO(this->get_logger(), "  Resolution: %.3f m", cfg_.resolution);
    RCLCPP_INFO(this->get_logger(), "  Map range: %.1fx%.1f m", cfg_.map_range_x, cfg_.map_range_y);
    RCLCPP_INFO(this->get_logger(), "  Height thresholds: slope<%.2f, step[%.2f-%.2f], obs>%.2f",
                cfg_.slope_height_max, cfg_.step_height_min, cfg_.step_height_max, cfg_.obstacle_height_min);
    RCLCPP_INFO(this->get_logger(), "  Dynamic leg length: %s (frame: %s)",
                cfg_.enable_dynamic_leg_length ? "ENABLED" : "DISABLED", cfg_.wheel_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "  Direct ROG-Map access: %s", rog_map_ptr_ ? "ENABLED" : "PENDING");
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
    
    RCLCPP_INFO(this->get_logger(), "  Map size: %dx%d cells", width, height);
}

void Map2DProjector::updateTimerCallback() {
    // 检查 ROG-Map 指针
    if (rog_map_ptr_ == nullptr) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "ROG-Map pointer not set, skipping update");
        return;
    }
    
    // 获取机器人位姿
    geometry_msgs::msg::Pose robot_pose;
    if (!getRobotPose(robot_pose)) {
        return;  // TF查询失败，跳过本次更新
    }
    
    // 更新动态腿长（如果启用）
    updateDynamicLegLength();
    
    // === 直接调用 ROG-Map boxSearchInflate() ===
    Eigen::Vector3d box_min(
        robot_position_.x() - cfg_.map_range_x / 2.0,
        robot_position_.y() - cfg_.map_range_y / 2.0,
        robot_position_.z() + cfg_.z_min_relative
    );
    Eigen::Vector3d box_max(
        robot_position_.x() + cfg_.map_range_x / 2.0,
        robot_position_.y() + cfg_.map_range_y / 2.0,
        robot_position_.z() + cfg_.z_max_relative
    );
    
    rog_map::vec_E<Eigen::Vector3d> inf_occ_points;
    rog_map_ptr_->boxSearchInflate(box_min, box_max, rog_map::OCCUPIED, inf_occ_points);
    
    // 检查数据有效性
    if (inf_occ_points.empty()) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "No occupied points in ROG-Map query range");
        return;
    }
    
    // 转换为 PCL 点云格式（直接使用 Eigen::Vector3f）
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.reserve(inf_occ_points.size());
    for (const auto& pt : inf_occ_points) {
        pcl_cloud.push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Direct query: %zu occupied points", pcl_cloud.size());
    
    // === 改进后的处理流程 ===
    std::unordered_map<int64_t, ColumnMetrics> columns;
    
    // Step 1: 高程分析（含点云缓存）
    elevationAnalysis(pcl_cloud, robot_position_.z(), columns);
    
    // Step 2: 法向量估计（仅低占据率柱体，跳过高占据率障碍物）
    normalEstimation(columns);
    
    // Step 6: 写入2D地图
    update2DMap(columns, robot_position_);
    
    // 发布地图
    publishMap();

    // 发布台阶调试可视化
   
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
            this->get_logger(),
            *this->get_clock(),
            5000,  // 每5秒警告一次
            "Could not get transform from base_link to %s: %s",
            cfg_.frame_id.c_str(), ex.what());
        return false;
    }
}

void Map2DProjector::updateDynamicLegLength() {
    if (!cfg_.enable_dynamic_leg_length) {
        current_base_to_ground_ = cfg_.base_to_ground_default;
        return;
    }
    
    try {
        // 查询 base_link -> wheel_link 的TF变换
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform("base_link", cfg_.wheel_frame, tf2::TimePointZero);
        
        // 腿长 = base_link到wheel_link的z方向距离（取绝对值）
        float leg_length = std::abs(transform.transform.translation.z);
        
        // 限制在合理范围内
        leg_length = std::clamp(leg_length, cfg_.leg_length_min, cfg_.leg_length_max);
        
        // 平滑更新（低通滤波，避免抖动）
        const float alpha = 0.3f;
        current_leg_length_ = alpha * leg_length + (1.0f - alpha) * current_leg_length_;
        current_base_to_ground_ = current_leg_length_;
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Leg length TF not available: %s, using default %.3f", 
            ex.what(), cfg_.base_to_ground_default);
        current_base_to_ground_ = cfg_.base_to_ground_default;
    }
}

std::vector<int64_t> Map2DProjector::getNeighborKeys(int64_t key) const {
    int ix, iy;
    gridKeyToXY(key, ix, iy);
    
    std::vector<int64_t> neighbors;
    neighbors.reserve(8);
    
    // 8邻域
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            int64_t nkey = (static_cast<int64_t>(ix + dx) << 32) | 
                           (static_cast<int64_t>(iy + dy) & 0xFFFFFFFF);
            neighbors.push_back(nkey);
        }
    }
    return neighbors;
}

void Map2DProjector::elevationAnalysis(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    float robot_z,
    std::unordered_map<int64_t, ColumnMetrics>& columns) {
    
    // 计算实际的高度范围（相对于机器人当前z）
    float z_min = robot_z + cfg_.z_min_relative;
    float z_max = robot_z + cfg_.z_max_relative;
    
    // 清空并重建点云缓存（用于法向量计算）
    column_points_cache_.clear();
    
    // 收集每个柱的所有点（含xyz，用于法向量）
    for (const auto& pt : cloud.points) {
        // 过滤超出高度范围的点
        if (pt.z < z_min || pt.z > z_max) {
            continue;
        }
        
        // 计算grid key
        int64_t key = xyToGridKey(pt.x, pt.y);
        column_points_cache_[key].emplace_back(pt.x, pt.y, pt.z);
    }
    
    // 计算每列的高程统计量
    for (auto& [key, points] : column_points_cache_) {
        auto& col = columns[key];
        col.point_count = static_cast<int>(points.size());
        
        if (col.point_count == 0) continue;
        
        // 提取z值用于统计
        std::vector<float> z_values;
        z_values.reserve(points.size());
        for (const auto& p : points) {
            z_values.push_back(p.z());
        }
        
        // 计算 min_z, max_z, height_diff
        col.min_z = *std::min_element(z_values.begin(), z_values.end());
        col.max_z = *std::max_element(z_values.begin(), z_values.end());
        col.height_diff = col.max_z - col.min_z;
        
        // 计算 median_z（用于坡面检测，更鲁棒）
        std::sort(z_values.begin(), z_values.end());
        size_t mid = z_values.size() / 2;
        if (z_values.size() % 2 == 0) {
            col.median_z = (z_values[mid - 1] + z_values[mid]) / 2.0f;
        } else {
            col.median_z = z_values[mid];
        }
        
        // 计算占据率：((n+1)*res)/H
        if (col.height_diff > 1e-3f) {
            float numerator = (col.point_count + 1) * cfg_.resolution;
            col.occupancy_rate = numerator / col.height_diff;
        } else {
            col.occupancy_rate = 1.0f;  // 高度差极小，视为满占据（平面）
        }
        
        col.cell_type = CellType::UNKNOWN;
        col.normal_valid = false;
        col.is_cliff_edge = false;
        col.lower_z_valid = false;
    }
}

void Map2DProjector::normalEstimation(std::unordered_map<int64_t, ColumnMetrics>& columns) {
    /**
     * 法向量估计（PCA方法）
     * 
     * 对所有点数充足的柱体计算法向量，不再按占据率跳过。
     * 原因：坡面上的LiDAR点云密度很高，占据率也高，但仍需法向量区分坡面和墙壁。
     * 
     * PCA原理：点云协方差矩阵的最小特征值对应法向量
     */
    
    for (auto& [key, col] : columns) {
        // 跳过点数不足的柱体
        if (col.point_count < cfg_.normal_min_points) {
            col.normal_valid = false;
            continue;
        }
        
        // 获取该柱的点云
        auto it = column_points_cache_.find(key);
        if (it == column_points_cache_.end()) {
            col.normal_valid = false;
            continue;
        }
        
        const auto& points = it->second;
        
        // 计算质心
        Vec3f centroid = Vec3f::Zero();
        for (const auto& p : points) {
            centroid += p;
        }
        centroid /= static_cast<float>(points.size());
        
        // 计算协方差矩阵
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (const auto& p : points) {
            Vec3f d = p - centroid;
            cov += d * d.transpose();
        }
        cov /= static_cast<float>(points.size());
        
        // 特征值分解
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        if (solver.info() != Eigen::Success) {
            col.normal_valid = false;
            continue;
        }
        
        // 特征值从小到大排列：λ0 < λ1 < λ2
        // 最小特征值对应法向量
        Vec3f eigenvalues = solver.eigenvalues();
        Eigen::Matrix3f eigenvectors = solver.eigenvectors();
        
        // 法向量 = 最小特征值对应的特征向量
        col.normal = eigenvectors.col(0);
        
        // 确保法向量朝上（z分量为正）
        if (col.normal.z() < 0) {
            col.normal = -col.normal;
        }
        
        col.normal_z_abs = std::abs(col.normal.z());
        
        // 平面性 = 1 - (λ0/λ1)，越大越平坦
        if (eigenvalues(1) > 1e-6f) {
            col.planarity = 1.0f - eigenvalues(0) / eigenvalues(1);
        } else {
            col.planarity = 0.0f;
        }
        
        col.normal_valid = true;
    }
}

int8_t Map2DProjector::classifyColumn(const ColumnMetrics& metrics, float robot_z) {
    /**
     * 综合分类（结合高程分析、法向量、台阶检测）
     * 
     * 分类优先级：
     * 1. 已标记为STEP或FREE → 直接返回
     * 2. 法向量有效 + |nz|大 → 可通行坡面（新增：优先判断坡面）
     * 3. 高占据率 + H大 → 障碍物（墙壁/人/车）
     * 4. 法向量有效 + |nz|小 → 垂直障碍物
     * 5. H很小 → 可通行地面/坡面
     * 6. 悬崖边缘 → 台阶
     * 7. 默认按高度判断
     */
    
    // 调试日志辅助宏
    #define DEBUG_CLASSIFY(reason, result) \
        if (cfg_.enable_debug_log) { \
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, \
                "[Classify] %s: H=%.3f occ=%.2f nz=%.2f pts=%d → %s", \
                reason, H, occ_rate, metrics.normal_z_abs, metrics.point_count, result); \
        }
    
    // 已分类的直接返回
    if (metrics.cell_type == CellType::STEP) {
        return cfg_.step_value;
    }
    if (metrics.cell_type == CellType::FREE) {
        return cfg_.free_value;
    }
    
    const float H = metrics.height_diff;
    const float occ_rate = metrics.occupancy_rate;
    
    // 计算关键高度
    float robot_bottom = robot_z - current_base_to_ground_;
    float robot_top = robot_bottom + cfg_.robot_height;
    float ground_level = robot_bottom + cfg_.ground_tolerance;
    
    // === Case 1: 噪声过滤 ===
    if (metrics.point_count < 2) {
        return cfg_.free_value;
    }
    
    // === Case 2: 完全悬空（限高杆）===
    if (metrics.min_z > robot_top) {
        DEBUG_CLASSIFY("Suspended", "FREE");
        return cfg_.free_value;
    }
    
    // === Case 3: 法向量判定的坡面（优先级高于高占据率判断）===
    // 关键修复：如果法向量有效且指向上方（|nz|大），无论H多大都判为可通行
    // 这解决了长坡面H很大但仍可通行的问题
    if (metrics.normal_valid && metrics.normal_z_abs >= cfg_.normal_z_slope_thresh) {
        // 法向量接近垂直向上 → 可通行坡面/平台
        // 但需确保不是悬空结构（高层平台下方需有支撑或超出机器人高度）
        if (metrics.min_z <= ground_level + cfg_.slope_height_max) {
            // 地面附近的坡面 → 直接可通行
            DEBUG_CLASSIFY("Slope(nz>=thresh,ground)", "FREE");
            return cfg_.free_value;
        }
        // 高层平台：min_z高于地面，但法向量朝上，说明是可以上去的平台
        // 如果min_z在机器人可达范围内（台阶或矮平台），可以考虑标记为台阶
        if (metrics.min_z <= ground_level + cfg_.step_height_max) {
            DEBUG_CLASSIFY("Slope(nz>=thresh,step)", "STEP");
            return cfg_.step_value;  // 作为台阶处理
        }
        // 过高的平台，但不阻挡通行（机器人从下方穿过）
        if (metrics.min_z > robot_top) {
            DEBUG_CLASSIFY("Slope(nz>=thresh,high)", "FREE");
            return cfg_.free_value;
        }
    }
    
    // === Case 4: 高占据率障碍物（快速路径）===
    // 高占据率 + H大于坡面阈值 + 法向量确认不是坡面 → 判障碍物
    // 关键：如果法向量有效且 nz >= wall_thresh（可能是坡面），不走此快速路径
    if (occ_rate > cfg_.high_occupancy_thresh && H > cfg_.slope_height_max) {
        bool likely_slope = metrics.normal_valid && 
                            metrics.normal_z_abs >= cfg_.normal_z_wall_thresh;
        if (!likely_slope) {
            bool blocks_passage = (metrics.min_z < robot_top) && (metrics.max_z > ground_level);
            if (blocks_passage) {
                DEBUG_CLASSIFY("HighOccupancy+LargeH+NotSlope", "OBS");
                return cfg_.obstacle_value;
            }
        }
    }
    
    // === Case 5: 法向量判定的垂直障碍物 ===
    if (metrics.normal_valid && metrics.normal_z_abs < cfg_.normal_z_wall_thresh) {
        // 法向量接近水平 → 垂直结构
        bool blocks_passage = (metrics.min_z < robot_top) && (metrics.max_z > ground_level);
        if (blocks_passage) {
            DEBUG_CLASSIFY("Wall(nz<wall_thresh)", "OBS");
            return cfg_.obstacle_value;
        }
    }
    
    // === Case 6: 坡道/平面（H很小）===
    if (H < cfg_.slope_height_max) {
        DEBUG_CLASSIFY("SmallH", "FREE");
        return cfg_.free_value;
    }
    
    // === Case 7: 悬崖边缘（下行台阶）===
    if (metrics.is_cliff_edge && metrics.lower_z_valid) {
        DEBUG_CLASSIFY("CliffEdge", "STEP");
        return cfg_.step_value;
    }
    
    // === Case 8: 法向量在中间范围（坡面阈值~墙面阈值之间）===
    // 这种情况可能是：中等坡度坡面、倾斜墙面、复杂结构
    // 策略：结合占据率和高度综合判断
    if (metrics.normal_valid && 
        metrics.normal_z_abs >= cfg_.normal_z_wall_thresh && 
        metrics.normal_z_abs < cfg_.normal_z_slope_thresh) {
        // 低占据率 + 中等法向量 → 倾向于判断为坡面
        if (occ_rate < cfg_.high_occupancy_thresh * 0.8f) {
            DEBUG_CLASSIFY("MidNormal+LowOcc", "FREE");
            return cfg_.free_value;
        }
        // 否则按高度判断
    }
    
    // === Case 9: 台阶范围但未被标记 → 矮墙 ===
    if (H >= cfg_.step_height_min && H <= cfg_.step_height_max) {
        bool blocks_passage = (metrics.min_z < robot_top) && (metrics.max_z > ground_level);
        if (blocks_passage) {
            DEBUG_CLASSIFY("StepRange→Wall", "OBS");
            return cfg_.obstacle_value;
        }
        return cfg_.free_value;
    }
    
    // === Case 10: 高障碍物 ===
    if (H >= cfg_.obstacle_height_min) {
        bool blocks_passage = (metrics.min_z < robot_top) && (metrics.max_z > ground_level);
        if (blocks_passage) {
            DEBUG_CLASSIFY("HighObstacle", "OBS");
            return cfg_.obstacle_value;
        }
        return cfg_.free_value;
    }
    
    // === Default: 按通行判断 ===
    bool blocks_passage = (metrics.min_z < robot_top) && (metrics.max_z > ground_level);
    if (blocks_passage) {
        DEBUG_CLASSIFY("Default→Blocks", "OBS");
        return cfg_.obstacle_value;
    }
    
    DEBUG_CLASSIFY("Default→Pass", "FREE");
    return cfg_.free_value;
    
    #undef DEBUG_CLASSIFY
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
    int step_count = 0;
    
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
        else if (value == cfg_.step_value) step_count++;
        else if (value == cfg_.free_value) free_count++;
    }
    
    // 每100帧输出一次统计信息
    static int update_count = 0;
    update_count++;
    if (update_count % 100 == 0) {
        RCLCPP_INFO(this->get_logger(),
            "[Map2D] %zu columns: %d obs, %d step, %d free | leg=%.3f",
            columns.size(), obstacle_count, step_count, free_count, current_leg_length_);
    }
}

void Map2DProjector::publishMap() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_2d_->header.stamp = this->get_clock()->now();
    map_pub_->publish(*map_2d_);
}

nav_msgs::msg::OccupancyGrid::SharedPtr Map2DProjector::getLatestMap() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return map_2d_;
}

void Map2DProjector::publishStepDebugMarkers() {

    if (step_cells_.empty()) {
        return;
    }
    
    visualization_msgs::msg::MarkerArray markers;
    auto now = this->get_clock()->now();
    
    // 删除旧的marker
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.stamp = now;
    delete_marker.header.frame_id = cfg_.frame_id;
    delete_marker.ns = "steps";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    // 台阶栅格可视化（使用CUBE_LIST提高效率）
    visualization_msgs::msg::Marker step_marker;
    step_marker.header.stamp = now;
    step_marker.header.frame_id = cfg_.frame_id;
    step_marker.ns = "steps";
    step_marker.id = 0;
    step_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    step_marker.action = visualization_msgs::msg::Marker::ADD;
    step_marker.scale.x = cfg_.resolution * 0.9;
    step_marker.scale.y = cfg_.resolution * 0.9;
    step_marker.scale.z = 0.05;
    step_marker.color.r = 0.0f;
    step_marker.color.g = 1.0f;
    step_marker.color.b = 0.5f;
    step_marker.color.a = 0.8f;
    step_marker.lifetime = rclcpp::Duration::from_seconds(0.15);
    
    for (int64_t key : step_cells_) {
        Vec2f pos = gridKeyToPos(key);
        geometry_msgs::msg::Point pt;
        pt.x = pos.x();
        pt.y = pos.y();
        pt.z = robot_position_.z();
        step_marker.points.push_back(pt);
    }
    markers.markers.push_back(step_marker);
    
    // 腿长状态文本（显示在机器人上方）
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.stamp = now;
    text_marker.header.frame_id = "base_link";
    text_marker.ns = "leg_status";
    text_marker.id = 1;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = 0.0;
    text_marker.pose.position.y = 0.0;
    text_marker.pose.position.z = cfg_.robot_height + 0.2;
    text_marker.scale.z = 0.12;
    text_marker.color.r = 1.0f;
    text_marker.color.g = 1.0f;
    text_marker.color.b = 1.0f;
    text_marker.color.a = 1.0f;
    
    char buf[64];
    snprintf(buf, sizeof(buf), "Leg:%.2fm Steps:%zu", 
             current_leg_length_, step_cells_.size());
    text_marker.text = buf;
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.15);
    markers.markers.push_back(text_marker);
    
    step_debug_pub_->publish(markers);
}

} // namespace map_2d_projector
