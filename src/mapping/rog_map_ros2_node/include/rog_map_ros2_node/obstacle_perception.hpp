/**
 * @file obstacle_perception.hpp
 * @brief 障碍物感知模块 - 基于ROG-Map的动态障碍物分割
 * 
 * 灵感来源：中科大哨兵2025技术报告
 * 核心思路：
 * 1. 高程分析：基于结构化点云对每个(x,y)柱进行高度差+占据率分析
 * 2. 时空聚类：对障碍物点按时间戳和位置聚类（利用occupancy map的尾迹特性）
 * 3. 卡尔曼滤波：匀速模型估计障碍物运动速度
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <unordered_map>
#include <vector>
#include <deque>
#include <mutex>

namespace obstacle_perception {

using Vec2f = Eigen::Vector2f;
using Vec2i = Eigen::Vector2i;
using Vec3f = Eigen::Vector3f;

/**
 * @brief 障碍物信息结构体
 */
struct Obstacle {
    int id;                          // 障碍物唯一ID
    Vec2f position;                  // 2D位置 (x, y)
    Vec2f velocity;                  // 2D速度估计
    double last_update_time;         // 最后更新时间
    bool is_dynamic;                 // 是否为动态障碍物
    
    // 卡尔曼滤波器状态
    Eigen::Vector4f state;           // [x, y, vx, vy]
    Eigen::Matrix4f covariance;      // 状态协方差
    int observation_count;           // 观测次数
};

/**
 * @brief 柱状高程分析结果
 */
struct ColumnAnalysis {
    float min_height;                // 最低点高度
    float max_height;                // 最高点高度
    float height_diff;               // 高度差
    float occupancy_rate;            // 占据率
    int point_count;                 // 点数
    bool is_obstacle;                // 是否为障碍物
};

/**
 * @brief 时间聚类结构
 */
struct TimedCluster {
    std::vector<Vec2f> points;       // 聚类中的点
    double timestamp;                // 时间戳（帧号）
    Vec2f centroid;                  // 聚类中心
};

/**
 * @brief 障碍物感知配置
 */
struct PerceptionConfig {
    // 高程分析参数
    float robot_height = 0.6f;       // 机器人高度（用于判断可通行性）
    float height_diff_thresh = 0.15f; // 高度差阈值（认为是障碍物）
    float occupancy_rate_thresh = 0.3f; // 占据率阈值
    float ground_height = 0.0f;      // 地面高度
    float ceiling_height = 2.0f;     // 天花板高度
    
    // 聚类参数
    float cluster_distance = 0.3f;   // 聚类距离阈值
    int min_cluster_size = 3;        // 最小聚类点数
    int max_frame_diff = 5;          // 时间聚类最大帧差
    
    // 卡尔曼滤波参数
    float process_noise = 0.1f;      // 过程噪声
    float measurement_noise = 0.05f; // 测量噪声
    float velocity_thresh = 0.1f;    // 动态障碍物速度阈值
    float chi_square_thresh = 5.991f; // 卡方检验阈值 (95% for 2 DOF)
    
    // 跟踪参数
    float max_association_dist = 0.5f; // 最大关联距离
    int max_lost_frames = 10;        // 最大丢失帧数
    
    // Grid参数（与ROG-Map一致）
    float resolution = 0.05f;        // 栅格分辨率
    float local_range = 5.0f;        // 局部感知范围
};

/**
 * @brief 障碍物感知节点
 */
class ObstaclePerceptionNode : public rclcpp::Node {
public:
    ObstaclePerceptionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
private:
    // ROS2 通信
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr occ_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversable_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr process_timer_;
    
    // 配置
    PerceptionConfig cfg_;
    
    // 状态
    std::mutex data_mtx_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
    Vec3f robot_position_{0, 0, 0};
    uint32_t frame_counter_{0};
    
    // 障碍物跟踪
    std::vector<Obstacle> tracked_obstacles_;
    int next_obstacle_id_{0};
    
    // 历史聚类（用于时空聚类）
    std::deque<std::vector<TimedCluster>> cluster_history_;
    
    // 回调函数
    void occCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void processCallback();
    
    // 核心算法
    /**
     * @brief 高程分析：对结构化点云进行柱状分析
     * @param cloud 输入点云（来自ROG-Map的occupied点）
     * @param columns 输出的柱状分析结果 (key: grid index)
     */
    void elevationAnalysis(
        const pcl::PointCloud<pcl::PointXYZ>& cloud,
        std::unordered_map<int64_t, ColumnAnalysis>& columns);
    
    /**
     * @brief 可通行区域分割：基于高程分析判断哪些区域可通行
     * @param columns 柱状分析结果
     * @param obstacle_points 输出的障碍物点
     * @param traversable_points 输出的可通行点
     */
    void traversabilitySegmentation(
        const std::unordered_map<int64_t, ColumnAnalysis>& columns,
        std::vector<Vec2f>& obstacle_points,
        std::vector<Vec2f>& traversable_points);
    
    /**
     * @brief 空间聚类：对障碍物点进行DBSCAN聚类
     * @param points 障碍物点
     * @param clusters 输出的聚类结果
     */
    void spatialClustering(
        const std::vector<Vec2f>& points,
        std::vector<TimedCluster>& clusters);
    
    /**
     * @brief 时空聚类匹配：将当前帧聚类与历史聚类关联
     * @param current_clusters 当前帧聚类
     */
    void temporalAssociation(std::vector<TimedCluster>& current_clusters);
    
    /**
     * @brief 卡尔曼滤波更新
     * @param obs 障碍物对象
     * @param measurement 测量值 (x, y)
     * @param dt 时间间隔
     */
    void kalmanUpdate(Obstacle& obs, const Vec2f& measurement, double dt);
    
    /**
     * @brief 卡尔曼预测
     * @param obs 障碍物对象
     * @param dt 时间间隔
     */
    void kalmanPredict(Obstacle& obs, double dt);
    
    /**
     * @brief 卡方检验：判断观测是否匹配
     * @param obs 障碍物对象
     * @param measurement 测量值
     * @return 卡方值
     */
    float chiSquareTest(const Obstacle& obs, const Vec2f& measurement);
    
    /**
     * @brief 初始化新障碍物
     * @param position 位置
     * @param timestamp 时间戳
     * @return 新障碍物对象
     */
    Obstacle initObstacle(const Vec2f& position, double timestamp);
    
    /**
     * @brief 数据关联：匈牙利算法或贪心关联
     * @param clusters 当前帧聚类
     */
    void dataAssociation(const std::vector<TimedCluster>& clusters);
    
    // 可视化
    void publishResults(
        const std::vector<Vec2f>& obstacle_points,
        const std::vector<Vec2f>& traversable_points);
    void publishObstacleMarkers();
    
    // 工具函数
    inline int64_t posToGridKey(const Vec2f& pos) const {
        int ix = static_cast<int>(std::floor(pos.x() / cfg_.resolution));
        int iy = static_cast<int>(std::floor(pos.y() / cfg_.resolution));
        return (static_cast<int64_t>(ix) << 32) | (static_cast<int64_t>(iy) & 0xFFFFFFFF);
    }
    
    inline Vec2f gridKeyToPos(int64_t key) const {
        int ix = static_cast<int>(key >> 32);
        int iy = static_cast<int>(key & 0xFFFFFFFF);
        return Vec2f(
            (ix + 0.5f) * cfg_.resolution,
            (iy + 0.5f) * cfg_.resolution
        );
    }
};

} // namespace obstacle_perception
