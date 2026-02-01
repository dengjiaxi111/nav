/**
 * @file map_2d_projector.hpp
 * @brief 3D占据地图降维为2D导航地图（轮腿机器人优化版）
 * 
 * 核心策略（基于RM场景优化）：
 * 1. 柱状高程分析：对每个(x,y)坐标分析z轴点云分布
 * 2. 高度差判定：H = max_z - min_z，判断是否为垂直障碍物
 * 3. 占据率判定：((n+1)*res)/H，判断柱体内点云密度
 * 4. 邻域分析：区分台阶（可跨越）和矮墙（不可跨越）
 * 5. 动态腿长：支持轮腿机器人站高/站低时的可通行性变化
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
 * @brief 柱状高程分析结果（含法向量特征和悬崖边缘检测）
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
    
    // === 悬崖边缘检测（下行台阶）===
    bool is_cliff_edge = false;      // 是否为悬崖/台阶边缘
    float inferred_lower_z = 0.0f;   // 推断的下方地面高度
    bool lower_z_valid = false;      // 下方高度是否可靠
    
    // === 分类结果 ===
    CellType cell_type = CellType::UNKNOWN;
};

/**
 * @brief 2D地图投影配置
 */
struct ProjectorConfig {
    // === 机器人几何参数 ===
    float robot_height = 0.5f;           // 机器人高度（底部到顶部）
    float robot_width = 0.4f;            // 机器人宽度（用于台阶邻域分析）
    float base_to_ground_default = 0.10f; // base_link到地面的默认距离
    float ground_tolerance = 0.03f;      // 地面起伏容差
    
    // === 动态腿长配置 ===
    bool enable_dynamic_leg_length = false;  // 是否启用动态腿长
    std::string wheel_frame = "wheel_link";  // 轮子坐标系名称
    float leg_length_min = 0.08f;            // 腿长最小值（收腿）
    float leg_length_max = 0.25f;            // 腿长最大值（伸腿）
    
    // === 高程分析阈值 ===
    float slope_height_max = 0.08f;      // 坡道最大高度差（H < 8cm → 可通行）
    float step_height_min = 0.12f;       // 台阶最小高度差 (支持15cm台阶，留3cm容差)
    float step_height_max = 0.23f;       // 台阶最大高度差 (支持20cm台阶，留3cm容差)
    float obstacle_height_min = 0.25f;   // 障碍物最小高度差
    float high_occupancy_thresh = 0.5f;  // 高占据率阈值（跳过法向量分析）
    
    // === 法向量分析参数 ===
    float normal_z_slope_thresh = 0.866f;   // cos(30°)，|nz|大于此值为可通行坡面
    float normal_z_wall_thresh = 0.5f;      // cos(60°)，|nz|小于此值为垂直障碍
    int normal_min_points = 5;              // 法向量估计最小点数
    float planarity_thresh = 0.7f;          // 平面性阈值，大于此值认为是平面
    
    // === 台阶检测参数 ===
    float step_15cm_min = 0.12f;            // 15cm台阶识别下限
    float step_15cm_max = 0.18f;            // 15cm台阶识别上限
    float step_20cm_min = 0.17f;            // 20cm台阶识别下限
    float step_20cm_max = 0.23f;            // 20cm台阶识别上限
    float step_continuity_thresh = 0.03f;   // 台阶连续性阈值（同一平面容差）
    int step_neighbor_min_count = 2;        // 台阶两侧最小邻域数
    float step_edge_ratio_thresh = 0.3f;    // 台阶边缘占比阈值
    
    // === 悬崖/下行台阶检测参数 ===
    int cliff_search_radius = 5;            // 悬崖边缘搜索半径（栅格数）
    float cliff_min_drop = 0.10f;           // 最小高度跌落（识别为悬崖边缘）
    int cliff_min_lower_points = 3;         // 下方平面最小有效栅格数
    
    // === 坡面检测参数（梯度连续性分析）===
    float slope_gradient_thresh = 0.577f;   // tan(30°)，最大允许坡度
    float slope_continuity_thresh = 0.03f;  // 梯度连续性阈值
    int slope_min_continuous_count = 3;     // 判定为坡面的最小连续邻域数
    
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
};

/**
 * @brief 2D地图投影节点 - 将ROG-Map的3D占据转换为Nav2兼容的2D地图
 */
class Map2DProjector {
public:
    Map2DProjector(rclcpp::Node::SharedPtr node, const ProjectorConfig& cfg);
    
    /**
     * @brief 处理3D点云并生成2D地图
     */
    void processPointCloud(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud,
        const geometry_msgs::msg::Pose& robot_pose);
    
    /**
     * @brief 获取最新的2D地图
     */
    nav_msgs::msg::OccupancyGrid::SharedPtr getLatestMap();
    
    /**
     * @brief 获取当前动态腿长（如启用）
     */
    float getCurrentLegLength() const { return current_leg_length_; }

private:
    rclcpp::Node::SharedPtr node_;
    ProjectorConfig cfg_;
    
    // 地图数据
    nav_msgs::msg::OccupancyGrid::SharedPtr map_2d_;
    std::mutex map_mutex_;
    
    // 机器人状态
    Vec3f robot_position_{0, 0, 0};
    float current_leg_length_;          // 当前腿长（动态更新）
    float current_base_to_ground_;      // 当前base_link到地面距离
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 发布器和订阅器
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr step_debug_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // 缓存
    sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;
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
     * @brief 悬崖边缘检测：识别下行台阶，推断下方高度
     */
    void cliffEdgeDetection(
        std::unordered_map<int64_t, ColumnMetrics>& columns,
        float robot_z);
    
    /**
     * @brief 坡面检测：结合法向量和梯度连续性
     */
    void slopeDetection(std::unordered_map<int64_t, ColumnMetrics>& columns);
    
    /**
     * @brief 检查单个栅格是否为坡面（结合法向量）
     */
    bool isSlopeCell(
        int64_t key,
        const std::unordered_map<int64_t, ColumnMetrics>& columns);
    
    /**
     * @brief 台阶检测：基于邻域分析区分台阶和矮墙
     * 
     * @param columns 高程分析结果（会更新cell_type字段）
     * @param robot_z 机器人当前z坐标
     */
    void stepDetection(
        std::unordered_map<int64_t, ColumnMetrics>& columns,
        float robot_z);
    
    /**
     * @brief 检查单个栅格是否为台阶（含下行台阶检测）
     */
    bool isStepCell(
        int64_t key,
        const std::unordered_map<int64_t, ColumnMetrics>& columns,
        float robot_z);
    
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
    
    void initializeMap();
    void publishMap();
    void publishStepDebugMarkers();
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishTimerCallback();
    
    // === 工具函数 ===
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
    
    /**
     * @brief 获取8邻域的grid key
     */
    std::vector<int64_t> getNeighborKeys(int64_t key) const;
};

} // namespace map_2d_projector
