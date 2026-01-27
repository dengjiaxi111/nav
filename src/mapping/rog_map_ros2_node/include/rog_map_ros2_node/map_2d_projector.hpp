/**
 * @file map_2d_projector.hpp
 * @brief 3D占据地图降维为2D导航地图
 * 
 * 核心策略（基于RM场景优化）：
 * 1. 柱状高程分析：对每个(x,y)坐标分析z轴点云分布
 * 2. 高度差判定：H = max_z - min_z，判断是否为垂直障碍物
 * 3. 占据率判定：((n+1)*res)/H，判断柱体内点云密度
 * 4. 可通行性分类：根据高程特征判断障碍/可通行/未知
 * 
 * 适用场景：
 * - RoboMaster竖直墙体（激光雷达扫描产生平面点云）
 * - 结构化点云输入（降低面噪声影响）
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <unordered_map>
#include <vector>
#include <mutex>

namespace map_2d_projector {

using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;

/**
 * @brief 柱状高程分析结果（基于中科大的方案）
 */
struct ColumnMetrics {
    float min_z;                     // 最低点高度
    float max_z;                     // 最高点高度
    float height_diff;               // 高度差 H = max_z - min_z
    float occupancy_rate;            // 占据率 = ((n+1)*res)/H
    int point_count;                 // 点数 n
};

/**
 * @brief 2D地图投影配置（基于中科大的高程分析方案）
 */
struct ProjectorConfig {
    // 机器人参数
    float robot_height = 0.5f;       // 机器人高度（从底部到顶部）
    float base_to_ground = 0.05f;    // base_link到地面的距离（底盘中心到轮子底部）
    float ground_tolerance = 0.03f;  // 地面起伏容差（用于过滤地面噪点）
    
    // 高程分析阈值（核心参数）
    float slope_height_max = 0.10f;   // 坡道/平面最大高度差（H < 0.1m → 可通行）
    float step_height_min = 0.10f;    // 台阶最小高度差（0.1m < H < 0.2m）
    float step_height_max = 0.20f;    // 台阶最大高度差
    float obstacle_height_min = 0.30f; // 障碍物最小高度差（H > 0.3m）
    float high_occupancy_thresh = 0.5f; // 高占据率阈值（密集点云）
    
    // 地图范围参数（相对于机器人）
    float map_range_x = 10.0f;       // X方向范围 ±5m
    float map_range_y = 10.0f;       // Y方向范围 ±5m
    float resolution = 0.05f;        // 栅格分辨率
    
    // 高度范围（相对于机器人当前z）
    float z_min_relative = -0.3f;    // 相对最低高度（避免地面噪声）
    float z_max_relative = 2.0f;     // 相对最高高度（天花板限制）
    
    // 占据值映射
    int8_t obstacle_value = 100;     // 障碍物占据值
    int8_t free_value = 0;           // 自由空间占据值
    int8_t unknown_value = -1;       // 未知区域占据值
    
    // 调试开关
    bool enable_debug_log = false;   // 启用详细分类日志
    
    // 发布参数
    float publish_rate = 10.0f;      // 发布频率 (Hz)
    std::string frame_id = "odom";   // 坐标系
    std::string topic_name = "rog_map/map_2d"; // 发布话题
};

/**
 * @brief 2D地图投影节点 - 将ROG-Map的3D占据转换为Nav2兼容的2D地图
 */
class Map2DProjector {
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     * @param cfg 配置参数
     */
    Map2DProjector(rclcpp::Node::SharedPtr node, const ProjectorConfig& cfg);
    
    /**
     * @brief 处理3D点云并生成2D地图
     * @param cloud 输入的3D占据点云（来自ROG-Map的inf_occ）
     * @param robot_pose 机器人当前位姿
     */
    void processPointCloud(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud,
        const geometry_msgs::msg::Pose& robot_pose);
    
    /**
     * @brief 获取最新的2D地图
     * @return 2D占据栅格地图
     */
    nav_msgs::msg::OccupancyGrid::SharedPtr getLatestMap();

private:
    rclcpp::Node::SharedPtr node_;
    ProjectorConfig cfg_;
    
    // 地图数据
    nav_msgs::msg::OccupancyGrid::SharedPtr map_2d_;
    std::mutex map_mutex_;
    
    // 机器人状态
    Vec3f robot_position_{0, 0, 0};
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 发布器和订阅器
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // 缓存
    sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;
    
    /**
     * @brief 获取机器人位姿（通过TF2）
     * @param robot_pose 输出机器人位姿
     * @return 是否成功获取
     */
    bool getRobotPose(geometry_msgs::msg::Pose& robot_pose);
    
    /**
     * @brief 高程分析：对每个(x,y)柱进行z轴分析
     * @param cloud PCL点云
     * @param robot_z 机器人当前z坐标
     * @param columns 输出的柱状分析结果
     */
    void elevationAnalysis(
        const pcl::PointCloud<pcl::PointXYZ>& cloud,
        float robot_z,
        std::unordered_map<int64_t, ColumnMetrics>& columns);
    
    /**
     * @brief 根据高程特征判定可通行性
     * @param metrics 柱状分析结果
     * @param robot_z 机器人当前z坐标
     * @return 占据值 (0=free, 100=obstacle, -1=unknown)
     */
    int8_t classifyColumn(const ColumnMetrics& metrics, float robot_z);
    
    /**
     * @brief 更新2D地图
     * @param columns 柱状分析结果
     * @param robot_pos 机器人位置
     */
    void update2DMap(
        const std::unordered_map<int64_t, ColumnMetrics>& columns,
        const Vec3f& robot_pos);
    
    /**
     * @brief 初始化地图结构
     */
    void initializeMap();
    
    /**
     * @brief 发布2D地图
     */
    void publishMap();
    
    /**
     * @brief 点云回调
     */
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    /**
     * @brief 定时发布回调
     */
    void publishTimerCallback();
    
    // 工具函数
    inline int64_t xyToGridKey(float x, float y) const {
        int ix = static_cast<int>(std::floor(x / cfg_.resolution));
        int iy = static_cast<int>(std::floor(y / cfg_.resolution));
        return (static_cast<int64_t>(ix) << 32) | (static_cast<int64_t>(iy) & 0xFFFFFFFF);
    }
    
    inline void gridKeyToXY(int64_t key, int& ix, int& iy) const {
        ix = static_cast<int>(key >> 32);
        iy = static_cast<int>(key & 0xFFFFFFFF);
    }
    
    inline Vec2f gridKeyToPos(int64_t key) const {
        int ix, iy;
        gridKeyToXY(key, ix, iy);
        return Vec2f((ix + 0.5f) * cfg_.resolution, (iy + 0.5f) * cfg_.resolution);
    }
};

} // namespace map_2d_projector
