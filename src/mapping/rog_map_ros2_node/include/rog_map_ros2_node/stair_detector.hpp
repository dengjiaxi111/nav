/**
 * @file stair_detector.hpp
 * @brief 基于 ROG-Map 的台阶检测节点（几何约束方案）
 * 
 * 算法流程：
 * 1. ROI 裁剪：直通滤波器限制 X/Y/Z 范围
 * 2. 欧式聚类：分割独立物体
 * 3. 硬约束筛选：高度 + 形态验证
 * 4. 边缘提取：精确计算台阶前沿和中心
 * 5. 时间跟踪：状态机 + 低通滤波
 * 
 * 场景：RoboMaster 轮腿机器人助跑起跳上台阶
 * 台阶规格：一级 (0.15m) 或 二级 (0.20m + 0.15m = 0.35m)
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <Eigen/Dense>
#include <memory>
#include <deque>

// 使用自定义消息
#include "rog_map_ros2_node/msg/stair_target.hpp"

namespace stair_detector {

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using Vec3f = Eigen::Vector3f;

/**
 * @brief 台阶检测状态枚举
 */
enum class DetectionState : uint8_t {
    IDLE = 0,           // 未检测到台阶
    DETECTED = 1,       // 检测到台阶，但置信度不足
    LOCKED = 2,         // 台阶已锁定，可以执行上台阶
    BLIND_ZONE = 3      // 盲区模式，依赖里程计推算
};

/**
 * @brief 台阶类型枚举
 */
enum class StairType : uint8_t {
    UNKNOWN = 0,
    SINGLE = 1,         // 一级台阶 (0.15m)
    DOUBLE = 2          // 二级台阶 (0.35m)
};

/**
 * @brief 台阶检测配置参数
 */
struct StairDetectorConfig {
    // === ROI 裁剪参数 ===
    float roi_x_min = 0.2f;          // 前方最小距离 (m)
    float roi_x_max = 2.0f;          // 前方最大距离 (m)
    float roi_y_min = -0.6f;         // 左侧范围 (m)
    float roi_y_max = 0.6f;          // 右侧范围 (m)
    float roi_z_min = 0.05f;         // 高度最小值 (m, 切掉地面)
    float roi_z_max = 0.5f;          // 高度最大值 (m)
    
    // === 聚类参数 ===
    float cluster_tolerance = 0.05f;  // 聚类搜索半径 (m)
    int min_cluster_size = 200;       // 最小聚类点数
    int max_cluster_size = 10000;     // 最大聚类点数
    
    // === 台阶几何约束 ===
    float single_stair_height = 0.20f;      // 一级台阶标准高度 (m)
    float double_stair_height = 0.35f;      // 二级台阶标准高度 (m)
    float height_tolerance = 0.04f;         // 高度匹配容差 (m)
    
    float min_stair_width = 0.35f;          // 最小台阶宽度 (m)
    float min_stair_depth = 0.15f;          // 最小台阶深度 (m)
    float max_z_thickness = 0.08f;          // Z 方向最大厚度 (m, 防止误识别墙壁)
    
    float normal_z_threshold = 0.9f;        // 法向量 Z 分量阈值 (防止斜面)
    
    // === 时间跟踪参数 ===
    int min_detection_frames = 3;           // 连续检测帧数要求
    float lowpass_alpha = 0.7f;             // 低通滤波系数
    
    // === 盲区策略 ===
    float blind_zone_distance = 0.3f;       // 进入盲区的距离阈值 (m)
    
    // === ROS 参数 ===
    std::string input_cloud_topic = "/rog_map/inf_occ";  // 默认膨胀占据点云
    std::string output_target_topic = "/stair_detector/target";
    std::string output_marker_topic = "/stair_detector/markers";
    std::string target_frame = "base_link";  // 从配置加载
    std::string map_frame = "odom";          // 从配置加载
    
    bool enable_visualization = true;
    double update_rate = 20.0;              // 检测频率 (Hz)
};

/**
 * @brief 台阶候选结构体
 */
struct StairCandidate {
    Vec3f center;           // 中心点 (odom 系)
    Vec3f bbox_min;         // 包围盒最小点
    Vec3f bbox_max;         // 包围盒最大点
    float height;           // 台阶高度
    float width;            // 台阶宽度
    float depth;            // 台阶深度
    float edge_x;           // 前沿 X 坐标
    StairType type;         // 台阶类型
    PointCloud::Ptr cloud;  // 原始点云
};

/**
 * @brief 台阶检测器主类
 */
class StairDetector : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    explicit StairDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~StairDetector() override = default;

private:
    // === 核心算法流程 ===
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void processPointCloud(const PointCloud::Ptr& cloud_in);
    
    // Step 1: ROI 裁剪
    PointCloud::Ptr roiFilter(const PointCloud::Ptr& cloud_in);
    
    // Step 2: 欧式聚类
    std::vector<PointCloud::Ptr> euclideanClustering(const PointCloud::Ptr& cloud_filtered);
    
    // Step 3: 硬约束筛选
    std::vector<StairCandidate> filterStairCandidates(
        const std::vector<PointCloud::Ptr>& clusters);
    
    // Step 4: 边缘提取与参数化
    void refineStairCandidate(StairCandidate& candidate);
    
    // Step 5: 时间一致性跟踪
    void updateTracking(const std::vector<StairCandidate>& candidates);
    
    // === 辅助函数 ===
    bool validateStairGeometry(const StairCandidate& candidate);
    float computeDistanceToRobot(const Vec3f& point);
    
    // === 可视化 ===
    void publishVisualization();
    void addCandidateMarker(
        visualization_msgs::msg::MarkerArray& markers,
        const StairCandidate& candidate,
        int id,
        bool is_locked);
    
    // === 发布结果 ===
    void publishStairTarget();
    
    // === TF2 辅助 ===
    bool transformPointCloud(
        const PointCloud::Ptr& cloud_in,
        PointCloud::Ptr& cloud_out,
        const std::string& target_frame);

private:
    // 配置参数
    StairDetectorConfig cfg_;
    
    // ROS 接口
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<rog_map_ros2_node::msg::StairTarget>::SharedPtr target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 状态跟踪
    DetectionState current_state_;
    StairCandidate locked_stair_;
    int consecutive_detection_count_;
    std::deque<StairCandidate> history_buffer_;  // 用于平滑
    
    // 盲区模式
    bool in_blind_zone_;
    Vec3f last_known_position_;
    rclcpp::Time last_detection_time_;
    
    // 性能监控
    rclcpp::Time last_process_time_;
};

} // namespace stair_detector
