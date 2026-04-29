// nav_components/include/nav_components/layered_map_manager.hpp
// 分层地图管理器 - 融合静态地图与动态局部障碍物层

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_core/map_interface.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>

#include "nav_components/static_map_loader.hpp"
#include "nav_components/costmap_inflater.hpp"
#include "nav_components/esdf_map.hpp"

#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <unordered_set>
#include <cstdint>
#include <array>

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

    /**
     * @brief 创建空白静态地图（用于纯动态SLAM模式）
     * @param width_m 地图宽度（米）
     * @param height_m 地图高度（米）
     * @param resolution 分辨率（米/格）
     * @param params 膨胀参数
     */
    void createBlankStaticMap(double width_m, double height_m, 
                              double resolution, const InflationParams& params);

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
    
    /**
     * @brief 启用/禁用静态层
     * @param enabled true=启用静态层, false=禁用（仅使用动态层）
     */
    void setStaticLayerEnabled(bool enabled) { static_layer_enabled_ = enabled; }

    struct StairLayerConfig {
        bool enable{false};
        std::string mask_yaml_path{};
        double clear_perp_dist_m{0.4};
        // 兼容旧参数：<0 时分别回退到 clear_perp_dist_m
        double clear_perp_high_dist_m{-1.0};
        double clear_perp_low_dist_m{-1.0};
        int black_min{0};
        int black_max{40};
        int gray_min{90};
        int gray_max{170};
        double stair_level2_clear_perp_dist_m{-1.0};
        double stair_level2_clear_perp_high_dist_m{-1.0};
        double stair_level2_clear_perp_low_dist_m{-1.0};
        int stair_level2_black_min{-1};
        int stair_level2_black_max{-1};
        int stair_level2_gray_min{-1};
        int stair_level2_gray_max{-1};
        int pair_search_radius_cells{4};
        bool enable_oneway_stair_down{false};
        int oneway_black_min{41};
        int oneway_black_max{80};
        int oneway_gray_min{171};
        int oneway_gray_max{230};
    };

    struct StairCrossingBand {
        // 沿台阶法向的低侧(negative)/高侧(positive)作用距离
        double low_side_dist_m{0.0};
        double high_side_dist_m{0.0};
        // 沿台阶切向的半宽（通常可与 half_length 一致）
        double tangent_half_width_m{0.0};
    };

    struct StairPrimitive {
        int stair_id{-1};
        Eigen::Vector2d center = Eigen::Vector2d::Zero();
        Eigen::Vector2d normal = Eigen::Vector2d::Zero();
        Eigen::Vector2d tangent = Eigen::Vector2d::Zero();
        double half_length{0.0};
        StairCrossingBand crossing_band{};
        bool is_oneway_down{false};
        bool is_level2{false};
    };

    struct FlySlopeLayerConfig {
        bool enable{false};
        std::string mask_yaml_path{};
        double clear_perp_dist_m{0.4};
        // 兼容旧参数：<0 时分别回退到 clear_perp_dist_m
        double clear_perp_high_dist_m{-1.0};
        double clear_perp_low_dist_m{-1.0};
        // 低侧/高侧双线像素范围
        int low_min{231};
        int low_max{240};
        int high_min{241};
        int high_max{250};
        int pair_search_radius_cells{4};
        // 单向约束：仅允许 low -> high，禁止 high -> low
        bool enable_oneway_low_to_high{true};
    };

    struct FlySlopePrimitive {
        int fly_slope_id{-1};
        Eigen::Vector2d center = Eigen::Vector2d::Zero();
        Eigen::Vector2d normal = Eigen::Vector2d::Zero(); // low -> high
        Eigen::Vector2d tangent = Eigen::Vector2d::Zero();
        double half_length{0.0};
        StairCrossingBand crossing_band{};
        bool is_oneway_low_to_high{true};
    };

    void setStairLayerConfig(const StairLayerConfig& cfg);
    void setFlySlopeLayerConfig(const FlySlopeLayerConfig& cfg);
    void setRuntimeBlockedStairUphillIds(const std::unordered_set<int>& stair_ids);
    void clearRuntimeBlockedStairUphillIds();

    bool isTransitionAllowed(int from_x, int from_y, int to_x, int to_y) const;
    void getForbiddenTransitionSegments(std::vector<std::array<double, 4>>& segments) const;
    bool getStairTraverseNormal(double wx, double wy, double& nx, double& ny) const;
    bool getStairPrimitiveAt(double wx, double wy, StairPrimitive& primitive) const;
    bool getStairPrimitiveById(int stair_id, StairPrimitive& primitive) const;
    std::vector<StairPrimitive> getStairPrimitives() const;
    bool getFlySlopeTraverseNormal(double wx, double wy, double& nx, double& ny) const;
    bool getFlySlopePrimitiveAt(double wx, double wy, FlySlopePrimitive& primitive) const;
    bool getFlySlopePrimitiveById(int fly_slope_id, FlySlopePrimitive& primitive) const;
    std::vector<FlySlopePrimitive> getFlySlopePrimitives() const;

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

    bool loadStairMaskFromYaml(const std::string& yaml_path);
    bool loadFlySlopeMaskFromYaml(const std::string& yaml_path);
    void rebuildStairLayerCache();
    void rebuildFlySlopeLayerCache();
    void applyStairLayerPolicy();
    bool worldToGlobalIndex(double wx, double wy, int& idx) const;
    static uint64_t encodeDirectedTransition(int from_idx, int to_idx);
    void addForbiddenDirectedTransitions(int from_idx, int to_idx,
                                         std::unordered_set<uint64_t>& out);

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

    // 分层地图开关
    bool static_layer_enabled_ = true;   // 静态层开关（默认启用）

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

    // 特殊地形层（stair layer）配置
    StairLayerConfig stair_layer_cfg_{};
    bool stair_mask_loaded_{false};
    std::string loaded_stair_mask_yaml_{};

    int stair_mask_width_{0};
    int stair_mask_height_{0};
    int stair_mask_max_val_{255};
    double stair_mask_resolution_{0.05};
    double stair_mask_origin_x_{0.0};
    double stair_mask_origin_y_{0.0};
    std::vector<uint8_t> stair_mask_pixels_{};

    std::vector<int> stair_clear_indices_{};
    std::unordered_set<uint64_t> stair_forbidden_transitions_{};
    std::vector<float> stair_normal_x_{};
    std::vector<float> stair_normal_y_{};
    std::vector<uint8_t> stair_normal_valid_{};
    std::vector<StairPrimitive> stair_primitives_{};
    std::vector<int> stair_primitive_id_map_{};  // global idx -> stair_id, -1 表示无
    std::unordered_set<int> runtime_blocked_stair_uphill_ids_{};

    // 飞坡语义层（fly slope layer）配置
    FlySlopeLayerConfig fly_slope_layer_cfg_{};
    bool fly_slope_mask_loaded_{false};
    std::string loaded_fly_slope_mask_yaml_{};

    int fly_slope_mask_width_{0};
    int fly_slope_mask_height_{0};
    int fly_slope_mask_max_val_{255};
    double fly_slope_mask_resolution_{0.05};
    double fly_slope_mask_origin_x_{0.0};
    double fly_slope_mask_origin_y_{0.0};
    std::vector<uint8_t> fly_slope_mask_pixels_{};

    std::vector<int> fly_slope_clear_indices_{};
    std::unordered_set<uint64_t> fly_slope_forbidden_transitions_{};
    std::vector<float> fly_slope_normal_x_{};
    std::vector<float> fly_slope_normal_y_{};
    std::vector<uint8_t> fly_slope_normal_valid_{};
    std::vector<FlySlopePrimitive> fly_slope_primitives_{};
    std::vector<int> fly_slope_primitive_id_map_{};  // global idx -> fly_slope_id, -1 表示无
};

}  // namespace nav_components
