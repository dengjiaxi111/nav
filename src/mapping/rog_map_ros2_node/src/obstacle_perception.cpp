/**
 * @file obstacle_perception.cpp
 * @brief 障碍物感知模块实现
 * 
 * 核心算法：
 * 1. 高程分析 - 基于中科大方案，对每个(x,y)柱进行高度差+占据率分析
 * 2. 时空聚类 - 利用occupancy map尾迹特性进行动态障碍物跟踪
 * 3. 卡尔曼滤波 - 匀速模型估计障碍物速度
 */

#include "rog_map_ros2_node/obstacle_perception.hpp"
#include <algorithm>
#include <cmath>

namespace obstacle_perception {

ObstaclePerceptionNode::ObstaclePerceptionNode(const rclcpp::NodeOptions& options)
    : Node("obstacle_perception", options) {
    
    // 声明参数
    cfg_.robot_height = declare_parameter("robot_height", 0.6f);
    cfg_.height_diff_thresh = declare_parameter("height_diff_thresh", 0.15f);
    cfg_.occupancy_rate_thresh = declare_parameter("occupancy_rate_thresh", 0.3f);
    cfg_.ground_height = declare_parameter("ground_height", 0.0f);
    cfg_.ceiling_height = declare_parameter("ceiling_height", 2.0f);
    cfg_.cluster_distance = declare_parameter("cluster_distance", 0.3f);
    cfg_.min_cluster_size = declare_parameter("min_cluster_size", 3);
    cfg_.max_frame_diff = declare_parameter("max_frame_diff", 5);
    cfg_.process_noise = declare_parameter("process_noise", 0.1f);
    cfg_.measurement_noise = declare_parameter("measurement_noise", 0.05f);
    cfg_.velocity_thresh = declare_parameter("velocity_thresh", 0.1f);
    cfg_.resolution = declare_parameter("resolution", 0.05f);
    cfg_.local_range = declare_parameter("local_range", 5.0f);
    cfg_.max_association_dist = declare_parameter("max_association_dist", 0.5f);
    cfg_.max_lost_frames = declare_parameter("max_lost_frames", 10);
    
    // QoS配置
    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1));
    
    // 订阅ROG-Map输出的occupied点云
    occ_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "rog_map/occ", qos,
        std::bind(&ObstaclePerceptionNode::occCallback, this, std::placeholders::_1));
    
    // 订阅里程计
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", qos,
        std::bind(&ObstaclePerceptionNode::odomCallback, this, std::placeholders::_1));
    
    // 发布处理结果
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("perception/obstacles", qos);
    traversable_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("perception/traversable", qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("perception/markers", qos);
    
    // 处理定时器 (20Hz)
    process_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ObstaclePerceptionNode::processCallback, this));
    
    latest_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    
    RCLCPP_INFO(get_logger(), "Obstacle perception node initialized");
    RCLCPP_INFO(get_logger(), "  - Robot height: %.2f m", cfg_.robot_height);
    RCLCPP_INFO(get_logger(), "  - Height diff thresh: %.2f m", cfg_.height_diff_thresh);
    RCLCPP_INFO(get_logger(), "  - Resolution: %.3f m", cfg_.resolution);
}

void ObstaclePerceptionNode::occCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    pcl::fromROSMsg(*msg, *latest_cloud_);
}

void ObstaclePerceptionNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    robot_position_ = Vec3f(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
}

void ObstaclePerceptionNode::processCallback() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    Vec3f robot_pos;
    
    {
        std::lock_guard<std::mutex> lock(data_mtx_);
        if (latest_cloud_->empty()) {
            return;
        }
        cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*latest_cloud_);
        robot_pos = robot_position_;
    }
    
    frame_counter_++;
    double current_time = get_clock()->now().seconds();
    
    // Step 1: 高程分析
    std::unordered_map<int64_t, ColumnAnalysis> columns;
    elevationAnalysis(*cloud, columns);
    
    // Step 2: 可通行性分割
    std::vector<Vec2f> obstacle_points, traversable_points;
    traversabilitySegmentation(columns, obstacle_points, traversable_points);
    
    // Step 3: 空间聚类
    std::vector<TimedCluster> current_clusters;
    spatialClustering(obstacle_points, current_clusters);
    
    // 为每个聚类设置时间戳
    for (auto& cluster : current_clusters) {
        cluster.timestamp = static_cast<double>(frame_counter_);
    }
    
    // Step 4: 数据关联与卡尔曼滤波
    dataAssociation(current_clusters);
    
    // Step 5: 维护历史聚类队列
    cluster_history_.push_back(current_clusters);
    while (cluster_history_.size() > static_cast<size_t>(cfg_.max_frame_diff)) {
        cluster_history_.pop_front();
    }
    
    // Step 6: 清理丢失的障碍物
    auto it = tracked_obstacles_.begin();
    while (it != tracked_obstacles_.end()) {
        double lost_frames = current_time - it->last_update_time;
        if (lost_frames > cfg_.max_lost_frames * 0.05) { // 0.05s per frame
            it = tracked_obstacles_.erase(it);
        } else {
            ++it;
        }
    }
    
    // 发布结果
    publishResults(obstacle_points, traversable_points);
    publishObstacleMarkers();
}

void ObstaclePerceptionNode::elevationAnalysis(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    std::unordered_map<int64_t, ColumnAnalysis>& columns) {
    
    // 以当前机器人z坐标为基准，计算实际的地面和天花板高度
    // 这样在坡道等地形上也能正确判断障碍物
    float robot_z = robot_position_.z();
    float actual_ground = robot_z + cfg_.ground_height;    // ground_height 现在是相对偏移
    float actual_ceiling = robot_z + cfg_.ceiling_height;  // ceiling_height 现在是相对偏移
    
    // 对每个点，按(x,y) grid索引归类
    for (const auto& pt : cloud.points) {
        // 过滤范围外的点
        Vec2f pos_2d(pt.x - robot_position_.x(), pt.y - robot_position_.y());
        if (pos_2d.norm() > cfg_.local_range) {
            continue;
        }
        
        // 过滤高度范围外的点（相对于当前机器人高度）
        if (pt.z < actual_ground || pt.z > actual_ceiling) {
            continue;
        }
        
        int64_t key = posToGridKey(Vec2f(pt.x, pt.y));
        
        auto it = columns.find(key);
        if (it == columns.end()) {
            ColumnAnalysis col;
            col.min_height = pt.z;
            col.max_height = pt.z;
            col.point_count = 1;
            columns[key] = col;
        } else {
            it->second.min_height = std::min(it->second.min_height, pt.z);
            it->second.max_height = std::max(it->second.max_height, pt.z);
            it->second.point_count++;
        }
    }
    
    // 计算每个柱的高度差和占据率
    for (auto& [key, col] : columns) {
        col.height_diff = col.max_height - col.min_height;
        
        // 占据率 = (点数 * 分辨率) / 高度差
        // 公式：占据率 = ((n + 1) * res) / H
        if (col.height_diff > cfg_.resolution) {
            col.occupancy_rate = (col.point_count * cfg_.resolution) / col.height_diff;
        } else {
            col.occupancy_rate = 1.0f; // 薄片视为完全占据
        }
    }
}

void ObstaclePerceptionNode::traversabilitySegmentation(
    const std::unordered_map<int64_t, ColumnAnalysis>& columns,
    std::vector<Vec2f>& obstacle_points,
    std::vector<Vec2f>& traversable_points) {
    
    obstacle_points.clear();
    traversable_points.clear();
    
    // 以当前机器人z坐标为基准计算相对地面高度
    float robot_z = robot_position_.z();
    float actual_ground = robot_z + cfg_.ground_height;
    
    for (const auto& [key, col] : columns) {
        Vec2f pos = gridKeyToPos(key);
        
        /**
         * 障碍物判定逻辑（中科大方案）：
         * 
         * 1. 高度差 > 阈值 且 占据率 > 阈值 => 墙壁/实体障碍物
         *    - 墙壁特征：垂直面，雷达扫到一条线，高度差大，占据率高
         * 
         * 2. 高度差 < 机器人高度 且 最高点 > 地面一定高度 => 可能是低矮障碍
         * 
         * 3. 占据率低 + 高度差大 => 可能是栏杆等稀疏结构
         */
        
        bool is_obstacle = false;
        
        // 规则1: 竖直墙壁 - 高占据率 + 足够高度差
        if (col.occupancy_rate > cfg_.occupancy_rate_thresh && 
            col.height_diff > cfg_.height_diff_thresh) {
            is_obstacle = true;
        }
        
        // 规则2: 低矮但密实的障碍物（如箱子）
        // 相对于当前机器人高度判断
        if (col.min_height > actual_ground + 0.05f && 
            col.min_height < robot_z + cfg_.robot_height * 0.8f) {
            is_obstacle = true;
        }
        
        // 规则3: 稀疏但高的结构（如栏杆）- 即使占据率低也是障碍
        if (col.height_diff > cfg_.robot_height && col.point_count >= 2) {
            is_obstacle = true;
        }
        
        if (is_obstacle) {
            obstacle_points.push_back(pos);
        } else {
            traversable_points.push_back(pos);
        }
    }
}

void ObstaclePerceptionNode::spatialClustering(
    const std::vector<Vec2f>& points,
    std::vector<TimedCluster>& clusters) {
    
    clusters.clear();
    if (points.empty()) return;
    
    // 简单的基于距离的聚类（类似DBSCAN但更简单）
    std::vector<bool> visited(points.size(), false);
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (visited[i]) continue;
        
        TimedCluster cluster;
        std::vector<size_t> queue;
        queue.push_back(i);
        visited[i] = true;
        
        size_t head = 0;
        while (head < queue.size()) {
            size_t idx = queue[head++];
            cluster.points.push_back(points[idx]);
            
            // 寻找邻近点
            for (size_t j = 0; j < points.size(); ++j) {
                if (visited[j]) continue;
                if ((points[idx] - points[j]).norm() < cfg_.cluster_distance) {
                    visited[j] = true;
                    queue.push_back(j);
                }
            }
        }
        
        // 过滤小聚类
        if (static_cast<int>(cluster.points.size()) >= cfg_.min_cluster_size) {
            // 计算聚类中心
            cluster.centroid = Vec2f::Zero();
            for (const auto& p : cluster.points) {
                cluster.centroid += p;
            }
            cluster.centroid /= static_cast<float>(cluster.points.size());
            clusters.push_back(cluster);
        }
    }
}

void ObstaclePerceptionNode::dataAssociation(const std::vector<TimedCluster>& clusters) {
    double current_time = get_clock()->now().seconds();
    
    // 预测所有现有障碍物
    for (auto& obs : tracked_obstacles_) {
        double dt = current_time - obs.last_update_time;
        if (dt > 0) {
            kalmanPredict(obs, dt);
        }
    }
    
    // 贪心数据关联
    std::vector<bool> cluster_matched(clusters.size(), false);
    std::vector<bool> obstacle_matched(tracked_obstacles_.size(), false);
    
    // 对每个聚类寻找最近的障碍物
    for (size_t i = 0; i < clusters.size(); ++i) {
        float min_dist = cfg_.max_association_dist;
        int best_match = -1;
        
        for (size_t j = 0; j < tracked_obstacles_.size(); ++j) {
            if (obstacle_matched[j]) continue;
            
            Vec2f predicted_pos(tracked_obstacles_[j].state(0), tracked_obstacles_[j].state(1));
            float dist = (clusters[i].centroid - predicted_pos).norm();
            
            // 卡方检验
            float chi2 = chiSquareTest(tracked_obstacles_[j], clusters[i].centroid);
            
            if (dist < min_dist && chi2 < cfg_.chi_square_thresh) {
                min_dist = dist;
                best_match = static_cast<int>(j);
            }
        }
        
        if (best_match >= 0) {
            // 更新匹配的障碍物
            double dt = current_time - tracked_obstacles_[best_match].last_update_time;
            kalmanUpdate(tracked_obstacles_[best_match], clusters[i].centroid, dt);
            tracked_obstacles_[best_match].last_update_time = current_time;
            tracked_obstacles_[best_match].observation_count++;
            
            // 判断是否为动态障碍物
            Vec2f vel(tracked_obstacles_[best_match].state(2), tracked_obstacles_[best_match].state(3));
            tracked_obstacles_[best_match].is_dynamic = vel.norm() > cfg_.velocity_thresh;
            tracked_obstacles_[best_match].velocity = vel;
            tracked_obstacles_[best_match].position = clusters[i].centroid;
            
            cluster_matched[i] = true;
            obstacle_matched[best_match] = true;
        }
    }
    
    // 为未匹配的聚类创建新障碍物
    for (size_t i = 0; i < clusters.size(); ++i) {
        if (!cluster_matched[i]) {
            Obstacle new_obs = initObstacle(clusters[i].centroid, current_time);
            tracked_obstacles_.push_back(new_obs);
        }
    }
}

Obstacle ObstaclePerceptionNode::initObstacle(const Vec2f& position, double timestamp) {
    Obstacle obs;
    obs.id = next_obstacle_id_++;
    obs.position = position;
    obs.velocity = Vec2f::Zero();
    obs.last_update_time = timestamp;
    obs.is_dynamic = false;
    obs.observation_count = 1;
    
    // 初始化卡尔曼状态
    obs.state = Eigen::Vector4f(position.x(), position.y(), 0.0f, 0.0f);
    obs.covariance = Eigen::Matrix4f::Identity() * cfg_.measurement_noise;
    
    return obs;
}

void ObstaclePerceptionNode::kalmanPredict(Obstacle& obs, double dt) {
    // 状态转移矩阵 (匀速模型)
    Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
    F(0, 2) = static_cast<float>(dt);
    F(1, 3) = static_cast<float>(dt);
    
    // 过程噪声
    Eigen::Matrix4f Q = Eigen::Matrix4f::Zero();
    float dt2 = static_cast<float>(dt * dt);
    float dt3 = dt2 * static_cast<float>(dt);
    float dt4 = dt2 * dt2;
    float q = cfg_.process_noise;
    Q(0, 0) = dt4 / 4 * q;
    Q(0, 2) = dt3 / 2 * q;
    Q(1, 1) = dt4 / 4 * q;
    Q(1, 3) = dt3 / 2 * q;
    Q(2, 0) = dt3 / 2 * q;
    Q(2, 2) = dt2 * q;
    Q(3, 1) = dt3 / 2 * q;
    Q(3, 3) = dt2 * q;
    
    // 预测
    obs.state = F * obs.state;
    obs.covariance = F * obs.covariance * F.transpose() + Q;
}

void ObstaclePerceptionNode::kalmanUpdate(Obstacle& obs, const Vec2f& measurement, double dt) {
    // 观测矩阵
    Eigen::Matrix<float, 2, 4> H;
    H << 1, 0, 0, 0,
         0, 1, 0, 0;
    
    // 观测噪声
    Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * cfg_.measurement_noise;
    
    // 卡尔曼增益
    Eigen::Matrix2f S = H * obs.covariance * H.transpose() + R;
    Eigen::Matrix<float, 4, 2> K = obs.covariance * H.transpose() * S.inverse();
    
    // 更新
    Eigen::Vector2f z(measurement.x(), measurement.y());
    Eigen::Vector2f y = z - H * obs.state;  // 残差
    obs.state = obs.state + K * y;
    obs.covariance = (Eigen::Matrix4f::Identity() - K * H) * obs.covariance;
}

float ObstaclePerceptionNode::chiSquareTest(const Obstacle& obs, const Vec2f& measurement) {
    // 观测矩阵
    Eigen::Matrix<float, 2, 4> H;
    H << 1, 0, 0, 0,
         0, 1, 0, 0;
    
    // 计算创新协方差
    Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * cfg_.measurement_noise;
    Eigen::Matrix2f S = H * obs.covariance * H.transpose() + R;
    
    // 计算残差
    Eigen::Vector2f z(measurement.x(), measurement.y());
    Eigen::Vector2f y = z - H * obs.state;
    
    // 卡方值
    return y.transpose() * S.inverse() * y;
}

void ObstaclePerceptionNode::publishResults(
    const std::vector<Vec2f>& obstacle_points,
    const std::vector<Vec2f>& traversable_points) {
    
    auto now = get_clock()->now();
    
    // 发布障碍物点云 - 使用 UniquePtr 实现零拷贝
    if (obstacle_pub_->get_subscription_count() > 0) {
        // 使用 loan_message 或 make_unique 发布，支持进程内零拷贝
        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        
        pcl::PointCloud<pcl::PointXYZ> obs_cloud;
        obs_cloud.reserve(obstacle_points.size());
        for (const auto& p : obstacle_points) {
            obs_cloud.push_back(pcl::PointXYZ(p.x(), p.y(), 0.0f));
        }
        pcl::toROSMsg(obs_cloud, *msg);
        msg->header.stamp = now;
        msg->header.frame_id = "odom";
        obstacle_pub_->publish(std::move(msg));  // 移动语义，避免拷贝
    }
    
    // 发布可通行区域点云 - 使用 UniquePtr 实现零拷贝
    if (traversable_pub_->get_subscription_count() > 0) {
        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        
        pcl::PointCloud<pcl::PointXYZ> trav_cloud;
        trav_cloud.reserve(traversable_points.size());
        for (const auto& p : traversable_points) {
            trav_cloud.push_back(pcl::PointXYZ(p.x(), p.y(), 0.0f));
        }
        pcl::toROSMsg(trav_cloud, *msg);
        msg->header.stamp = now;
        msg->header.frame_id = "odom";
        traversable_pub_->publish(std::move(msg));  // 移动语义，避免拷贝
    }
}

void ObstaclePerceptionNode::publishObstacleMarkers() {
    if (marker_pub_->get_subscription_count() == 0) return;
    
    visualization_msgs::msg::MarkerArray markers;
    auto now = get_clock()->now();
    
    int id = 0;
    for (const auto& obs : tracked_obstacles_) {
        // 障碍物位置标记
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = now;
        marker.header.frame_id = "odom";
        marker.ns = "obstacles";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = obs.position.x();
        marker.pose.position.y = obs.position.y();
        marker.pose.position.z = 0.3;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.6;
        
        // 动态障碍物用红色，静态用蓝色
        if (obs.is_dynamic) {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        marker.color.a = 0.7;
        marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        markers.markers.push_back(marker);
        
        // 速度箭头（仅动态障碍物）
        if (obs.is_dynamic && obs.velocity.norm() > cfg_.velocity_thresh) {
            visualization_msgs::msg::Marker arrow;
            arrow.header.stamp = now;
            arrow.header.frame_id = "odom";
            arrow.ns = "velocities";
            arrow.id = id++;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            
            geometry_msgs::msg::Point start, end;
            start.x = obs.position.x();
            start.y = obs.position.y();
            start.z = 0.3;
            end.x = obs.position.x() + obs.velocity.x();
            end.y = obs.position.y() + obs.velocity.y();
            end.z = 0.3;
            arrow.points.push_back(start);
            arrow.points.push_back(end);
            
            arrow.scale.x = 0.05;  // shaft diameter
            arrow.scale.y = 0.1;   // head diameter
            arrow.scale.z = 0.1;   // head length
            arrow.color.r = 1.0;
            arrow.color.g = 1.0;
            arrow.color.b = 0.0;
            arrow.color.a = 1.0;
            arrow.lifetime = rclcpp::Duration::from_seconds(0.2);
            markers.markers.push_back(arrow);
        }
        
        // ID文本
        visualization_msgs::msg::Marker text;
        text.header.stamp = now;
        text.header.frame_id = "odom";
        text.ns = "ids";
        text.id = id++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = obs.position.x();
        text.pose.position.y = obs.position.y();
        text.pose.position.z = 0.8;
        text.scale.z = 0.2;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;
        
        std::string label = "ID:" + std::to_string(obs.id);
        if (obs.is_dynamic) {
            label += " v=" + std::to_string(obs.velocity.norm()).substr(0, 4) + "m/s";
        }
        text.text = label;
        text.lifetime = rclcpp::Duration::from_seconds(0.2);
        markers.markers.push_back(text);
    }
    
    marker_pub_->publish(markers);
}

} // namespace obstacle_perception

// 注册组件
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_perception::ObstaclePerceptionNode)
