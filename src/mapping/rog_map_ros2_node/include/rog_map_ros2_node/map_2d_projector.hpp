/**
 * @file map_2d_projector.hpp
 * @brief 3D占据地图降维为2D导航地图（轮腿机器人优化版）
 * 
 * 核心策略（基于RM场景优化）：
 * 1. 柱状高程分析：对每个(x,y)坐标分析z轴点云分布
 * 2. 高度差判定：H = max_z - min_z，判断是否为垂直障碍物
 * 3. 占据率判定：((n+1)*res)/H，判断柱体内点云密度
 * 4. 动态腿长：支持轮腿机器人站高/站低时的可通行性变化
 * 
 * 适用场景：
 * - RoboMaster场地：矮墙、高墙、坡道、台阶、多层平面
 * - 轮腿机器人：支持动态高度变化
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <robots_msgs/msg/leg_length.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <mutex>

// ROG-Map forward declaration
namespace rog_map {
    class ROGMapROS;
}

namespace map_2d_projector {

using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;

/**
 * @brief 栅格分类枚举
 */
enum class CellType : int8_t {
    UNKNOWN = -1,        // 未知区域
    FREE = 0,            // 自由空间
    STEP = 50,           // 台阶（轮腿可跨越）
    OBSTACLE = 100       // 障碍物（不可通行）
};

/**
 * @brief 柱状高程分析结果（含法向量特征）
 */
struct ColumnMetrics {
    // === 基础高程特征 ===
    float min_z = 0.0f;              // 最低点高度
    float max_z = 0.0f;              // 最高点高度
    float median_z = 0.0f;           // 中位数高度（更鲁棒）
    float height_diff = 0.0f;        // 高度差 H = max_z - min_z
    float occupancy_rate = 0.0f;     // 占据率 = ((n+1)*res)/H
    int point_count = 0;             // 点数 n
    
    // === 法向量特征（仅低占据率柱体计算）===
    Vec3f normal{0, 0, 1};           // 局部平面法向量（默认朝上）
    float normal_z_abs = 1.0f;       // |nz|，快速判断坡面 (>0.866 ≈ <30°)
    float planarity = 0.0f;          // 平面性：1-(λ2/λ1)，越大越平坦
    bool normal_valid = false;       // 法向量是否有效计算
    
    // === 分类结果 ===
    CellType cell_type = CellType::UNKNOWN;
};

/**
 * @brief 2D地图投影配置
 */
struct ProjectorConfig {
    // === 机器人几何参数 ===
    float robot_height = 0.5f;           // 机器人高度（底部到顶部）
    float robot_width = 0.4f;            // 机器人宽度
    float base_to_ground_default = 0.10f; // base_link到地面的默认距离
    float ground_tolerance = 0.03f;      // 地面起伏容差
    
    // === 动态腿长配置 ===
    bool enable_dynamic_leg_length = false;  // 是否启用动态腿长
    std::string wheel_frame = "wheel_link";  // 轮子坐标系名称
    float leg_length_min = 0.08f;            // 腿长最小值（收腿）
    float leg_length_max = 0.25f;            // 腿长最大值（伸腿）
    bool enable_leg_length_topic = false;    // 是否直接订阅腿长更新离地高度
    std::string leg_length_topic = "LegLength";
    float leg_length_offset = 0.0f;          // 结构偏置: base_to_ground = leg_length + offset
    
    // === 高程分析阈值 ===
    float slope_height_max = 0.08f;      // 坡道最大高度差（H < 8cm → 可通行）
    float step_height_min = 0.12f;       // 台阶最小高度差 (支持15cm台阶，留3cm容差)
    float step_height_max = 0.23f;       // 台阶最大高度差 (支持20cm台阶，留3cm容差)
    float obstacle_height_min = 0.25f;   // 障碍物最小高度差
    float high_occupancy_thresh = 0.5f;  // 高占据率阈值（跳过法向量分析）
    bool keep_step_cells = false;        // 是否将台阶保留为STEP，否则按FREE处理

    // === 障碍邻域支持过滤 ===
    bool enable_obstacle_support_filter = true;  // 抑制孤立稀疏障碍候选
    int obstacle_support_radius_cells = 1;       // 邻域半径，1 表示 3x3
    int obstacle_min_support_count = 2;          // 包含自身在内的最小候选数量
    float obstacle_support_max_height = 0.25f;   // 仅过滤低矮/薄层障碍候选
    
    // === 法向量分析参数 ===
    float normal_z_slope_thresh = 0.866f;   // cos(30°)，|nz|大于此值为可通行坡面
    float normal_z_wall_thresh = 0.5f;      // cos(60°)，|nz|小于此值为垂直障碍
    int normal_min_points = 5;              // 法向量估计最小点数
    float planarity_thresh = 0.7f;          // 平面性阈值，大于此值认为是平面

    // === 邻域/连通坡面识别参数 ===
    bool enable_slope_region_filter = true; // 是否启用邻域平面拟合坡面过滤
    int slope_region_neighbor_radius = 2;   // 邻域PCA半径，2 表示 5x5
    int slope_region_min_points = 12;       // 邻域PCA最少点数
    int slope_region_min_cells = 12;        // 连通坡面最少栅格数
    float slope_region_max_angle_deg = 40.0f;       // 最大可通行坡角
    float slope_region_planarity_thresh = 0.65f;    // 邻域平面性阈值
    float slope_region_neighbor_dz_margin = 0.03f;  // 相邻栅格高度连续余量
    
    // === 地图范围参数 ===
    float map_range_x = 10.0f;           // X方向范围
    float map_range_y = 10.0f;           // Y方向范围
    float resolution = 0.05f;            // 栅格分辨率
    
    // === 高度范围（相对于机器人） ===
    float z_min_relative = -0.5f;        // 相对最低高度
    float z_max_relative = 2.0f;         // 相对最高高度
    
    // === 占据值映射 ===
    int8_t obstacle_value = 100;
    int8_t step_value = 50;              // 台阶占据值（供规划器参考）
    int8_t free_value = 0;
    int8_t unknown_value = -1;
    
    // === 调试开关 ===
    bool enable_debug_log = false;
    bool enable_step_debug_viz = true;   // 发布台阶可视化
    
    // === 发布参数 ===
    float publish_rate = 20.0f;          // 20Hz帧率
    std::string frame_id = "odom";
    std::string topic_name = "rog_map/map_2d";
    std::string step_debug_topic = "rog_map/step_debug";  // 台阶调试话题
    
    // === 输入话题 ===
    std::string input_cloud_topic = "/rog_map/inf_occ";  // 使用膨胀后的占据点云
};

/**
 * @brief 2D地图投影组件 - 将ROG-Map的3D占据转换为Nav2兼容的2D地图
 * 
 * **直接调用模式**: 通过指针直接访问 ROGMapROS 实例,无需 topic 通信
 * **组件模式**: 继承 rclcpp::Node，支持进程内通信和动态加载
 */
class Map2DProjector : public rclcpp::Node {
public:
    /**
     * @brief 构造函数（组件模式）
     * @param options 节点选项
     * @param rog_map_ptr ROG-Map 实例指针（用于直接查询）
     */
    explicit Map2DProjector(const rclcpp::NodeOptions& options,
                            rog_map::ROGMapROS* rog_map_ptr = nullptr);
    
    /**
     * @brief 设置 ROG-Map 实例指针（延迟注入）
     */
    void setRogMapPtr(rog_map::ROGMapROS* ptr) { rog_map_ptr_ = ptr; }
    
    /**
     * @brief 获取最新的2D地图
     */
    nav_msgs::msg::OccupancyGrid::SharedPtr getLatestMap();
    
    /**
     * @brief 获取当前动态腿长（如启用）
     */
    float getCurrentLegLength() const { return current_leg_length_; }

private:
    ProjectorConfig cfg_;
    
    // ROG-Map 实例指针（直接查询）
    rog_map::ROGMapROS* rog_map_ptr_;
    
    // 地图数据
    nav_msgs::msg::OccupancyGrid::SharedPtr map_2d_;
    std::mutex map_mutex_;
    
    // 机器人状态
    Vec3f robot_position_{0, 0, 0};
    float current_leg_length_;          // 当前腿长（动态更新）
    float current_base_to_ground_;      // 当前base_link到地面距离
    float latest_leg_length_msg_{0.0f};
    bool has_leg_length_msg_{false};
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 发布器和定时器
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr step_debug_pub_;
    rclcpp::Subscription<robots_msgs::msg::LegLength>::SharedPtr leg_length_sub_;
    rclcpp::TimerBase::SharedPtr update_timer_;  // 改名: publish_timer_ -> update_timer_
    
    // 缓存
    std::unordered_set<int64_t> step_cells_;  // 被判定为台阶的栅格
    
    // 点云z值缓存（用于法向量计算）
    std::unordered_map<int64_t, std::vector<Vec3f>> column_points_cache_;
    
    // === 核心方法 ===
    
    /**
     * @brief 获取机器人位姿（通过TF2）
     */
    bool getRobotPose(geometry_msgs::msg::Pose& robot_pose);
    
    /**
     * @brief 更新动态腿长（通过TF2查询wheel_link）
     */
    void updateDynamicLegLength();
    void legLengthCallback(const robots_msgs::msg::LegLength::SharedPtr msg);
    
    /**
     * @brief 高程分析：对每个(x,y)柱进行z轴分析（同时缓存点云用于法向量）
     */
    void elevationAnalysis(
        const pcl::PointCloud<pcl::PointXYZ>& cloud,
        float robot_z,
        std::unordered_map<int64_t, ColumnMetrics>& columns);
    
    /**
     * @brief 法向量估计：仅对低占据率柱体计算PCA法向量
     * 高占据率柱体直接跳过（避免人/车等动态物体误判）
     */
    void normalEstimation(std::unordered_map<int64_t, ColumnMetrics>& columns);

    /**
     * @brief 邻域PCA + 连通域坡面识别，将连续斜平面预标记为FREE
     */
    void slopeRegionEstimation(std::unordered_map<int64_t, ColumnMetrics>& columns);
    
    /**
     * @brief 根据高程特征判定可通行性
     */
    int8_t classifyColumn(const ColumnMetrics& metrics, float robot_z);
    
    /**
     * @brief 更新2D地图
     */
    void update2DMap(
        const std::unordered_map<int64_t, ColumnMetrics>& columns,
        const Vec3f& robot_pos);

    bool needsObstacleSupport(const ColumnMetrics& metrics) const;
    bool hasObstacleSupport(int64_t key, const std::unordered_set<int64_t>& obstacle_candidates) const;
    
    void initializeMap();
    void publishMap();
    void publishStepDebugMarkers();
    void updateTimerCallback();  // 改名: publishTimerCallback() -> updateTimerCallback()
    
    inline int64_t xyToGridKey(float x, float y) const {
        int ix = static_cast<int>(std::floor(x / cfg_.resolution));
        int iy = static_cast<int>(std::floor(y / cfg_.resolution));
        return (static_cast<int64_t>(ix) << 32) | (static_cast<int64_t>(iy) & 0xFFFFFFFF);
    }

    inline int64_t gridIndexToKey(int ix, int iy) const {
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
