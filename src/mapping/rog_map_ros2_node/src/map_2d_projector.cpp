/**
 * @file map_2d_projector.cpp
 * @brief 3D→2D地图投影实现（基于中科大高程分析方案）
 * 
 * 核心算法：对每个(x,y)柱进行高度差H + 占据率分析
 * 新增功能：动态腿长支持、台阶邻域检测
 */

#include "rog_map_ros2_node/map_2d_projector.hpp"
#include <algorithm>
#include <cmath>

namespace map_2d_projector {

Map2DProjector::Map2DProjector(rclcpp::Node::SharedPtr node, const ProjectorConfig& cfg)
    : node_(node), cfg_(cfg),
      current_leg_length_(cfg.base_to_ground_default),
      current_base_to_ground_(cfg.base_to_ground_default) {
    
    // 初始化2D地图
    initializeMap();
    
    // 初始化TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // QoS配置 - 与ROG-Map一致使用best_effort
    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1));
    
    // 订阅ROG-Map的占据点云
    // 关键：使用原始占据点云 /rog_map/occ，而不是膨胀后的 /rog_map/inf_occ
    // 原因：膨胀点云会在坡面上产生Z轴堆叠，导致误判为障碍物
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg_.input_cloud_topic, qos,
        std::bind(&Map2DProjector::cloudCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), "Subscribing to cloud topic: %s", cfg_.input_cloud_topic.c_str());
    
    // 发布2D地图
    map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        cfg_.topic_name, qos);
    
    // 发布台阶调试可视化（功能已移至独立节点）
    // if (cfg_.enable_step_debug_viz) {
    //     step_debug_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    //         cfg_.step_debug_topic, qos);
    // }
    
    // 定时发布器
    int publish_period_ms = static_cast<int>(1000.0 / cfg_.publish_rate);
    publish_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(publish_period_ms),
        std::bind(&Map2DProjector::publishTimerCallback, this));
    
    RCLCPP_INFO(node_->get_logger(), "Map2DProjector initialized (wheel-leg optimized)");
    RCLCPP_INFO(node_->get_logger(), "  Resolution: %.3f m", cfg_.resolution);
    RCLCPP_INFO(node_->get_logger(), "  Map range: %.1fx%.1f m", cfg_.map_range_x, cfg_.map_range_y);
    RCLCPP_INFO(node_->get_logger(), "  Height thresholds: slope<%.2f, step[%.2f-%.2f], obs>%.2f",
                cfg_.slope_height_max, cfg_.step_height_min, cfg_.step_height_max, cfg_.obstacle_height_min);
    RCLCPP_INFO(node_->get_logger(), "  Dynamic leg length: %s (frame: %s)",
                cfg_.enable_dynamic_leg_length ? "ENABLED" : "DISABLED", cfg_.wheel_frame.c_str());
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

void Map2DProjector::updateDynamicLegLength() {
    if (!cfg_.enable_dynamic_leg_length) {
        current_base_to_ground_ = cfg_.base_to_ground_default;
        return;
    }
    
    try {
        // 查询 base_link -> wheel_link 的TF变换
        // wheel_link 应该位于轮子中心，其z坐标相对于base_link表示腿长
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
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
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

void Map2DProjector::publishTimerCallback() {
    if (!latest_cloud_) {
        return;
    }
    
    // 获取机器人位姿
    geometry_msgs::msg::Pose robot_pose;
    if (!getRobotPose(robot_pose)) {
        return;  // TF查询失败，跳过本次更新
    }
    
    // 更新动态腿长（如果启用）
    updateDynamicLegLength();
    
    // 转换点云
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*latest_cloud_, pcl_cloud);
    
    if (pcl_cloud.empty()) {
        return;
    }
    
    // === 改进后的处理流程 ===
    std::unordered_map<int64_t, ColumnMetrics> columns;
    
    // Step 1: 高程分析（含点云缓存）
    elevationAnalysis(pcl_cloud, robot_position_.z(), columns);
    
    // Step 2: 法向量估计（仅低占据率柱体，跳过高占据率障碍物）
    normalEstimation(columns);
    
    // Step 3: 悬崖边缘检测（下行台阶识别）
    cliffEdgeDetection(columns, robot_position_.z());
    
    // Step 4: 坡面检测（结合法向量）
    slopeDetection(columns);
    
    // Step 5: 台阶检测（已移至独立 stair_detector_node，此处注释）
    // stepDetection(columns, robot_position_.z());
    
    // Step 6: 更新2D地图
    update2DMap(columns, robot_position_);
    
    // Step 7: 发布地图
    publishMap();
    
    // Step 8: 发布台阶调试可视化
    if (cfg_.enable_step_debug_viz && step_debug_pub_) {
        publishStepDebugMarkers();
    }
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
     * 策略：仅对低占据率柱体计算，高占据率直接跳过
     * - 高占据率（>0.5）：极大概率是墙壁/人/车等障碍物，无需法向量
     * - 低占据率：可能是坡面、地面或薄障碍物，需要法向量区分
     * 
     * PCA原理：点云协方差矩阵的最小特征值对应法向量
     */
    
    for (auto& [key, col] : columns) {
        // 跳过高占据率柱体（障碍物快速路径）
        if (col.occupancy_rate > cfg_.high_occupancy_thresh) {
            col.normal_valid = false;
            continue;
        }
        
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

void Map2DProjector::cliffEdgeDetection(
    std::unordered_map<int64_t, ColumnMetrics>& columns,
    float robot_z) {
    
    /**
     * 悬崖边缘检测（下行台阶识别）
     * 
     * 问题：激光雷达无法扫到台阶正下方，只能看到远处的地面
     * 方案：检测边缘栅格，向远处搜索低位平面，推断台阶高度
     * 
     * 流程：
     * 1. 找到可能的边缘栅格（有地面点，但邻域出现unknown或高度突降）
     * 2. 沿8方向搜索，寻找低位平面
     * 3. 验证高度差是否在台阶范围内（15cm或20cm）
     * 4. 标记为可通行台阶边缘
     */
    
    float ground_level = robot_z - current_base_to_ground_ + cfg_.ground_tolerance;
    
    for (auto& [key, col] : columns) {
        // 只对地面附近的栅格进行边缘检测
        if (col.point_count < 2) continue;
        if (col.height_diff > cfg_.slope_height_max) continue;  // 非平面
        if (col.min_z > ground_level + 0.1f) continue;  // 太高，不是地面
        
        auto neighbors = getNeighborKeys(key);
        
        int unknown_neighbor_count = 0;
        int lower_neighbor_count = 0;
        float best_lower_z = col.min_z;
        
        // 扩展搜索：不仅看8邻域，还要向远处搜索
        int ix, iy;
        gridKeyToXY(key, ix, iy);
        
        // 8方向搜索
        const int dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
        const int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};
        
        for (int dir = 0; dir < 8; ++dir) {
            float dir_lower_z = col.min_z;
            bool found_lower_plane = false;
            
            // 沿该方向搜索
            for (int step = 1; step <= cfg_.cliff_search_radius; ++step) {
                int nx = ix + dx[dir] * step;
                int ny = iy + dy[dir] * step;
                int64_t nkey = (static_cast<int64_t>(nx) << 32) | 
                               (static_cast<int64_t>(ny) & 0xFFFFFFFF);
                
                auto it = columns.find(nkey);
                if (it == columns.end()) {
                    // 远处是unknown，记录
                    if (step == 1) unknown_neighbor_count++;
                    continue;
                }
                
                const auto& ncol = it->second;
                
                // 检查是否是平坦的低位平面
                if (ncol.height_diff < cfg_.slope_height_max && 
                    ncol.point_count >= 3) {
                    
                    float z_drop = col.min_z - ncol.min_z;
                    
                    // 高度跌落在台阶范围内
                    if (z_drop >= cfg_.step_height_min && z_drop <= cfg_.step_height_max) {
                        found_lower_plane = true;
                        if (ncol.min_z < dir_lower_z) {
                            dir_lower_z = ncol.min_z;
                        }
                        lower_neighbor_count++;
                        break;  // 找到就停止该方向搜索
                    }
                }
            }
            
            if (found_lower_plane && dir_lower_z < best_lower_z) {
                best_lower_z = dir_lower_z;
            }
        }
        
        // 判定条件：有unknown邻域 + 找到足够的低位平面
        bool has_cliff_feature = (unknown_neighbor_count >= 1) || 
                                  (lower_neighbor_count >= cfg_.cliff_min_lower_points);
        
        if (has_cliff_feature && lower_neighbor_count >= cfg_.cliff_min_lower_points) {
            float step_height = col.min_z - best_lower_z;
            
            // 验证台阶高度是否符合15cm或20cm规格
            bool is_15cm_step = (step_height >= cfg_.step_15cm_min && step_height <= cfg_.step_15cm_max);
            bool is_20cm_step = (step_height >= cfg_.step_20cm_min && step_height <= cfg_.step_20cm_max);
            
            if (is_15cm_step || is_20cm_step) {
                col.is_cliff_edge = true;
                col.inferred_lower_z = best_lower_z;
                col.lower_z_valid = true;
            }
        }
    }
}

void Map2DProjector::slopeDetection(std::unordered_map<int64_t, ColumnMetrics>& columns) {
    /**
     * 坡面检测（结合法向量和梯度连续性）
     * 
     * 优化后方案：
     * 1. 法向量有效且|nz| >= slope_thresh → 直接判定为坡面
     * 2. 法向量有效且|nz|在中间范围 + 低占据率 → 倾向于判断为坡面
     * 3. 法向量无效时，回退到梯度连续性分析
     * 4. 占据率高的已在normalEstimation中跳过，不会误判
     */
    
    for (auto& [key, col] : columns) {
        // 跳过已经分类的或点数太少的
        if (col.cell_type != CellType::UNKNOWN || col.point_count < 2) {
            continue;
        }
        
        // H很小，直接标记为FREE（平坦地面）
        if (col.height_diff < cfg_.slope_height_max) {
            col.cell_type = CellType::FREE;
            continue;
        }
        
        // 方法1：法向量判定（优先）
        if (col.normal_valid) {
            // |nz| >= slope_thresh → 坡面（放宽平面性要求，因为真实坡面可能不够平坦）
            if (col.normal_z_abs >= cfg_.normal_z_slope_thresh) {
                col.cell_type = CellType::FREE;
                continue;
            }
            
            // |nz| 在中间范围 [wall_thresh, slope_thresh) + 低占据率 → 更可能是坡面
            if (col.normal_z_abs >= cfg_.normal_z_wall_thresh && 
                col.occupancy_rate < cfg_.high_occupancy_thresh * 0.7f) {
                // 进一步检查平面性
                if (col.planarity > cfg_.planarity_thresh * 0.8f) {
                    col.cell_type = CellType::FREE;
                    continue;
                }
            }
            
            // |nz| < wall_thresh → 垂直障碍物（但不在这里标记，留给classify）
            if (col.normal_z_abs < cfg_.normal_z_wall_thresh) {
                continue;  // 跳过，让classify处理
            }
        }
        
        // 方法2：梯度连续性分析（回退方案，用于法向量无效的情况）
        if (isSlopeCell(key, columns)) {
            col.cell_type = CellType::FREE;
        }
    }
}

bool Map2DProjector::isSlopeCell(
    int64_t key,
    const std::unordered_map<int64_t, ColumnMetrics>& columns) {
    
    /**
     * 坡面判定：基于 median_z 梯度连续性
     * 
     * 为什么用 median_z 而不是 min_z 或 max_z？
     * - min_z：易受地面噪声/草皮影响，可能偏低
     * - max_z：易受障碍物顶部影响，可能偏高  
     * - median_z：代表点云的"主体高度"，对噪声鲁棒
     * 
     * 坡面特征：median_z 沿某方向平滑单调变化
     * 墙壁特征：median_z 突变或梯度不连续
     */
    
    const auto& col = columns.at(key);
    float this_median_z = col.median_z;
    
    // 占据率过高 → 更像墙壁而非坡面
    if (col.occupancy_rate > cfg_.high_occupancy_thresh) {
        return false;
    }
    
    auto neighbors = getNeighborKeys(key);
    
    // 收集邻域梯度信息
    std::vector<float> gradients;
    int valid_neighbor_count = 0;
    int continuous_gradient_count = 0;
    
    for (int64_t nkey : neighbors) {
        auto it = columns.find(nkey);
        if (it == columns.end()) continue;
        
        const auto& ncol = it->second;
        if (ncol.point_count < 2) continue;
        
        valid_neighbor_count++;
        
        // 计算梯度：(邻域median_z - 当前median_z) / 距离
        int nix, niy, ix, iy;
        gridKeyToXY(nkey, nix, niy);
        gridKeyToXY(key, ix, iy);
        
        float dist = cfg_.resolution;
        if (std::abs(nix - ix) + std::abs(niy - iy) == 2) {
            dist = cfg_.resolution * 1.414f;  // 对角线
        }
        
        float z_diff = ncol.median_z - this_median_z;
        float gradient = z_diff / dist;
        
        gradients.push_back(gradient);
        
        // 检查梯度是否在合理坡度范围内
        if (std::abs(gradient) <= cfg_.slope_gradient_thresh) {
            continuous_gradient_count++;
        }
    }
    
    // 条件1：必须有足够的有效邻域
    if (valid_neighbor_count < 3) {
        return false;
    }
    
    // 条件2：大部分邻域的梯度都在合理范围内
    float continuous_ratio = static_cast<float>(continuous_gradient_count) / valid_neighbor_count;
    if (continuous_ratio < 0.6f) {
        return false;  // 超过40%的邻域梯度过大，不是平滑坡面
    }
    
    // 条件3：检查梯度的一致性（方差不能太大）
    if (gradients.size() >= 3) {
        float mean = 0.0f;
        for (float g : gradients) {
            mean += g;
        }
        mean /= gradients.size();
        
        float variance = 0.0f;
        for (float g : gradients) {
            variance += (g - mean) * (g - mean);
        }
        variance /= gradients.size();
        
        // 梯度方差过大说明高度变化不规则，不是平滑坡面
        float stddev = std::sqrt(variance);
        if (stddev > cfg_.slope_continuity_thresh * 2.0f) {
            return false;
        }
    }
    
    // 条件4：至少有一定数量的连续邻域
    if (continuous_gradient_count < cfg_.slope_min_continuous_count) {
        return false;
    }
    
    return true;
}

void Map2DProjector::stepDetection(
    std::unordered_map<int64_t, ColumnMetrics>& columns,
    float robot_z) {
    
    step_cells_.clear();
    
    // 计算机器人通行高度范围（使用动态腿长）
    float robot_bottom = robot_z - current_base_to_ground_;
    float robot_top = robot_bottom + cfg_.robot_height;
    float ground_level = robot_bottom + cfg_.ground_tolerance;
    
    // 统计变量
    int candidates = 0;
    int cliff_edge_steps = 0;  // 下行台阶（悬崖边缘）
    int normal_steps = 0;      // 常规台阶（上行）
    int skipped_by_slope = 0;
    
    for (auto& [key, col] : columns) {
        // 跳过已经被坡面检测标记为FREE的
        if (col.cell_type == CellType::FREE) {
            skipped_by_slope++;
            continue;
        }
        
        // === 情况1：悬崖边缘（下行台阶）===
        if (col.is_cliff_edge && col.lower_z_valid) {
            // 已在cliffEdgeDetection中验证过高度差
            col.cell_type = CellType::STEP;
            step_cells_.insert(key);
            cliff_edge_steps++;
            continue;
        }
        
        // === 情况2：常规台阶（上行）===
        const float H = col.height_diff;
        
        // 只对台阶高度范围内的栅格进行邻域分析
        if (H < cfg_.step_height_min || H > cfg_.step_height_max) {
            continue;
        }
        
        candidates++;
        
        // 检查是否阻挡通行
        bool blocks_passage = (col.min_z < robot_top) && (col.max_z > ground_level);
        if (!blocks_passage) {
            continue;
        }
        
        // 邻域分析判断是否为台阶
        if (isStepCell(key, columns, robot_z)) {
            col.cell_type = CellType::STEP;
            step_cells_.insert(key);
            normal_steps++;
        }
    }
    
    // 定期日志
    static int log_counter = 0;
    if (++log_counter % 100 == 0) {
        RCLCPP_INFO(node_->get_logger(), 
            "[StepDetect] cliff_edge=%d, normal=%d, skipped_slope=%d | total=%zu",
            cliff_edge_steps, normal_steps, skipped_by_slope, step_cells_.size());
    }
}

bool Map2DProjector::isStepCell(
    int64_t key,
    const std::unordered_map<int64_t, ColumnMetrics>& columns,
    float /*robot_z*/) {
    
    /**
     * 台阶检测核心逻辑
     * 
     * 台阶特征：
     * 1. 当前栅格高度差在台阶范围 [step_height_min, step_height_max]
     * 2. 邻域存在两个不同高度的平面（上下台阶面）
     * 3. 占据率不能太高（太高说明是密集墙壁）
     */
    
    const auto& col = columns.at(key);
    
    // 占据率过高 → 更像墙壁而非台阶边缘
    if (col.occupancy_rate > cfg_.high_occupancy_thresh) {
        return false;
    }
    
    auto neighbors = getNeighborKeys(key);
    
    // 统计邻域特征
    int higher_plane_count = 0;   // 比当前高的平面邻域数
    int lower_plane_count = 0;    // 比当前低的平面邻域数
    float higher_z_sum = 0.0f;
    float lower_z_sum = 0.0f;
    
    float this_min_z = col.min_z;
    
    for (int64_t nkey : neighbors) {
        auto it = columns.find(nkey);
        if (it == columns.end()) continue;
        
        const auto& ncol = it->second;
        
        // 噪声过滤：点数太少不可靠
        if (ncol.point_count < 2) continue;
        
        // 邻域必须是相对平坦的（高度差小）才能算作平面
        // 如果邻域本身也是大高度差，说明是复杂结构，不作为参考
        if (ncol.height_diff > cfg_.step_height_min) {
            continue;
        }
        
        float z_diff = ncol.min_z - this_min_z;
        
        if (z_diff > cfg_.step_continuity_thresh) {
            // 邻域更高 → 当前可能是台阶下层
            higher_plane_count++;
            higher_z_sum += ncol.min_z;
        } else if (z_diff < -cfg_.step_continuity_thresh) {
            // 邻域更低 → 当前可能是台阶上层
            lower_plane_count++;
            lower_z_sum += ncol.min_z;
        }
        // 否则：同一高度平面，不计入
    }
    
    // === 台阶判定条件 ===
    // 必须两侧都有足够的平面邻域（表示这是两个平面的交界）
    if (higher_plane_count < cfg_.step_neighbor_min_count || 
        lower_plane_count < cfg_.step_neighbor_min_count) {
        return false;  // 一侧缺少平面 → 可能是矮墙边缘
    }
    
    // 验证高低平面的高度差是否合理
    float avg_higher_z = higher_z_sum / higher_plane_count;
    float avg_lower_z = lower_z_sum / lower_plane_count;
    float plane_diff = avg_higher_z - avg_lower_z;
    
    // 平面高度差应该在合理的台阶范围内
    if (plane_diff < cfg_.step_height_min || plane_diff > cfg_.step_height_max * 1.5f) {
        return false;
    }
    
    // 通过所有检查 → 判定为台阶
    return true;
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
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, \
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
    // 高占据率 + H大于坡面阈值 + 法向量不是坡面 → 判障碍物
    if (occ_rate > cfg_.high_occupancy_thresh && H > cfg_.slope_height_max) {
        bool blocks_passage = (metrics.min_z < robot_top) && (metrics.max_z > ground_level);
        if (blocks_passage) {
            DEBUG_CLASSIFY("HighOccupancy+LargeH", "OBS");
            return cfg_.obstacle_value;
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
        RCLCPP_INFO(node_->get_logger(),
            "[Map2D] %zu columns: %d obs, %d step, %d free | leg=%.3f",
            columns.size(), obstacle_count, step_count, free_count, current_leg_length_);
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
    
    // 台阶检测
    stepDetection(columns, robot_pos.z());
    
    // 更新2D地图
    update2DMap(columns, robot_pos);
}

void Map2DProjector::publishStepDebugMarkers() {
    if (step_cells_.empty()) {
        return;
    }
    
    visualization_msgs::msg::MarkerArray markers;
    auto now = node_->get_clock()->now();
    
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
