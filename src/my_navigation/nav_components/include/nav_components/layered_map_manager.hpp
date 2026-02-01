// nav_components/include/nav_components/layered_map_manager.hpp
// 分层地图管理器 - 融合静态地图与动态局部障碍物层

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_core/map_interface.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav_components/static_map_loader.hpp"
#include "nav_components/costmap_inflater.hpp"
#include "nav_components/esdf_map.hpp"

#include <mutex>
#include <atomic>
#include <vector>

namespace nav_components {

/**
 * @brief 分层地图管理器
 * 
 * 支持两层地图融合:
 * 1. 静态层 (static_layer): 从yaml/pgm加载的全局地图
 * 2. 动态层 (dynamic_layer): 来自rog_map的滑动窗口局部障碍物
 * 
 * 融合策略:
 * - 占据取max: fused[i] = max(static[i], dynamic_projected[i])
 * - 动态层通过TF从odom投影到map坐标系
 * - 支持增量ESDF更新以优化性能
 */
class LayeredMapManager : public nav_core::MapInterface {
public:
    using Ptr = std::shared_ptr<LayeredMapManager>;

    LayeredMapManager();
    ~LayeredMapManager() = default;

    // ============ 初始化 ============
    
    /**
     * @brief 初始化管理器
     * @param node ROS2节点指针
     * @param tf_buffer TF缓冲区（用于odom->map变换）
     */
    void initialize(rclcpp::Node* node, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

    /**
     * @brief 从yaml文件加载静态地图
     * @param yaml_path 地图yaml文件路径
     * @param params 膨胀参数
     * @return 是否成功
     */
    bool loadStaticMap(const std::string& yaml_path, const InflationParams& params);

    /**
     * @brief 设置静态地图（从外部订阅）
     */
    void setStaticMap(nav_msgs::msg::OccupancyGrid::SharedPtr map);

    /**
     * @brief 设置膨胀参数
     */
    void setInflationParams(const InflationParams& params);

    // ============ 动态层更新 ============

    /**
     * @brief 更新动态障碍物层
     * @param local_map 局部障碍物地图（odom坐标系）
     * 
     * 内部流程:
     * 1. 查询odom->map变换
     * 2. 将local_map投影到全局地图坐标
     * 3. 融合到fused_map
     * 4. 增量更新ESDF
     */
    void updateDynamicLayer(const nav_msgs::msg::OccupancyGrid::SharedPtr& local_map);

    // ============ 获取地图 ============

    nav_msgs::msg::OccupancyGrid::SharedPtr getStaticMap() const;
    nav_msgs::msg::OccupancyGrid::SharedPtr getFusedMap() const;
    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmap() const;
    nav_msgs::msg::OccupancyGrid::SharedPtr getEsdfVis() const;
    EsdfMap::Ptr getEsdf() const;

    bool hasMap() const { return static_map_ != nullptr; }
    bool hasEsdf() const { return esdf_ != nullptr; }

    // ============ ESDF配置 ============

    /**
     * @brief 启用/禁用ESDF计算
     */
    void setEsdfEnabled(bool enabled) { esdf_enabled_ = enabled; }
    
    /**
     * @brief 设置ESDF可视化最大距离
     */
    void setEsdfVisMaxDist(double dist) { esdf_vis_max_dist_ = dist; }
    
    /**
     * @brief 启用/禁用性能日志输出
     */
    void setEnablePerformanceLogging(bool enabled) { enable_performance_logging_ = enabled; }

    // ============ MapInterface实现 ============

    nav_core::MapType type() const override { return nav_core::MapType::COSTMAP; }
    double resolution() const override { return resolution_; }
    bool hasDistance() const override { return esdf_ != nullptr; }

    void getBounds(double& min_x, double& min_y,
                   double& max_x, double& max_y) const override;

    nav_core::MapQuery query(double x, double y) const override;

    /**
     * @brief 获取ESDF距离和梯度（用于轨迹优化）
     */
    double getEsdfDistanceWithGradient(double x, double y, double* gx, double* gy) const;

    /**
     * @brief 设置障碍物阈值
     */
    void setObstacleThreshold(int8_t thresh) { obstacle_threshold_ = thresh; }

private:
    // ============ 内部方法 ============

    /**
     * @brief 重建融合地图
     * 将静态层与动态层投影融合
     */
    void rebuildFusedMap();

    /**
     * @brief 重建costmap（从融合地图，独立EDT计算）
     */
    void rebuildCostmap();
    
    /**
     * @brief 从ESDF复用距离场构建costmap（性能优化，共享EDT）
     */
    void rebuildCostmapFromEsdf();
    
    /**
     * @brief 重建ESDF（仅用于初始化）
     */
    void rebuildEsdf();

    /**
     * @brief 坐标变换：odom → map
     */
    bool transformOdomToMap(double ox, double oy, double& mx, double& my) const;

    // ============ 成员变量 ============

    rclcpp::Node* node_ = nullptr;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Logger logger_;

    // 地图数据
    nav_msgs::msg::OccupancyGrid::SharedPtr static_map_;    // 静态层
    nav_msgs::msg::OccupancyGrid::SharedPtr dynamic_map_;   // 动态层（已投影到map坐标系）
    nav_msgs::msg::OccupancyGrid::SharedPtr fused_map_;     // 融合地图
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;       // 膨胀后的costmap
    nav_msgs::msg::OccupancyGrid::SharedPtr esdf_vis_;      // ESDF可视化
    EsdfMap::Ptr esdf_;

    // 参数
    InflationParams inflation_params_;
    double resolution_ = 0.05;
    double origin_x_ = 0.0, origin_y_ = 0.0;
    int width_ = 0, height_ = 0;
    int8_t obstacle_threshold_ = 50;

    // ESDF配置
    std::atomic<bool> esdf_enabled_{false};
    double esdf_vis_max_dist_ = 2.0;

    // 性能日志开关
    bool enable_performance_logging_ = false;

    // 坐标系
    std::string map_frame_ = "map";
    std::string odom_frame_ = "odom";

    // 线程安全
    mutable std::mutex map_mutex_;

    // 性能优化：记录上次动态层的边界，用于增量更新
    struct DynamicLayerBounds {
        int min_x = 0, max_x = 0;
        int min_y = 0, max_y = 0;
        bool valid = false;
    } last_dynamic_bounds_;
};

}  // namespace nav_components
