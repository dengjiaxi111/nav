// nav_components/include/nav_components/simple_planner.hpp
// A* 规划器 + B样条平滑 + B样条优化

#pragma once
#include <nav_core/planner_base.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <queue>
#include <unordered_set>
#include "nav_components/planner/path_smoother.hpp"

namespace nav_components {

// 前向声明
class LayeredMapManager;

class SimplePlanner : public nav_core::PlannerBase {
public:
    void initialize(rclcpp::Node* node) override;
    
    // 重写 setMap，接入 LayeredMapManager（统一从 costmap 取栅格）
    void setMap(nav_core::MapInterface::Ptr map) override;
    
    bool plan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        nav_msgs::msg::Path& path) override;
    
    // 公开接口：路径验证（供 nav_server 周期性检查使用）
    bool validatePath(const nav_msgs::msg::Path& path);
    
    // 公开接口：供外部检查位置和地图坐标转换
    bool worldToMap(double wx, double wy, int& mx, int& my);
    void mapToWorld(int mx, int my, double& wx, double& wy);
    bool isValid(int x, int y);
    void clearCache();  // 清空缓存路径（强制重规划时使用）

private:
    // A*节点
    struct Node {
        int x, y;
        double g, h;
        Node* parent = nullptr;
        double f() const { return g + h; }
    };
    
    // 坐标转换
    double getCost(int x, int y);
    double heuristic(int x1, int y1, int x2, int y2);
    
    // 内部功能：路径检查和缓存
    bool goalChanged(const geometry_msgs::msg::PoseStamped& new_goal);  // 检查目标是否变化
    nav_msgs::msg::Path prunePath(const nav_msgs::msg::Path& path, 
                                   const geometry_msgs::msg::PoseStamped& current_pose);  // 剪枝已驶过部分
    
    // A*搜索（独立函数，支持自适应重规划）
    bool runAstar(int sx, int sy, int gx, int gy,
                  const std_msgs::msg::Header& header,
                  nav_msgs::msg::Path& path);

    // A* 台阶感知形态代价：入口附近抑制切向掠过、约束中垂线
    double computeStairShapeCost(int from_x, int from_y,
                                 int to_x, int to_y);
    
    // 脱困：BFS搜索最近的可通行格子
    bool findNearestFreeCell(int cx, int cy, int& fx, int& fy, int max_radius = 50);
    
    rclcpp::Node* node_ = nullptr;
    
    // 地图
    nav_msgs::msg::OccupancyGrid::SharedPtr grid_map_;
    std::shared_ptr<LayeredMapManager> map_manager_;  // 用于 ESDF 查询
    int8_t obstacle_threshold_ = 99;
    double cost_weight_ = 0.5;
    
    // 缓存的地图信息
    double resolution_ = 0.05;
    double origin_x_ = 0, origin_y_ = 0;
    int width_ = 0, height_ = 0;
    
    // 路径平滑
    PathSmoother smoother_;
    bool enable_smooth_ = true;
    
    // 新增：路径缓存和复用
    nav_msgs::msg::Path cached_path_;  // 缓存的路径
    geometry_msgs::msg::PoseStamped cached_goal_;  // 缓存的目标
    double goal_change_tolerance_ = 0.2;  // 目标变化阈值(米)
    double obstacle_check_threshold_ = 95;  // 障碍物检查阈值
    double esdf_warn_min_safe_dist_ = 0.12;  // 平滑后ESDF预警最小安全距离(米)
    double start_deviation_threshold_ = 1.0;  // 起点偏离缓存路径阈值(米)
    int astar_max_attempts_ = 2;  // A* 重试次数
    int astar_max_iterations_ = 0;  // A* 单次搜索最大迭代次数(<=0: 自动)
    int astar_threshold_step_ = 10;  // A* 每次重试降低的障碍物阈值步长
    bool enable_path_cache_ = true;  // 启用路径缓存
    bool enable_auto_prune_ = true;  // 启用自动剪枝
    double prune_distance_ = 0.5;  // 剪枝距离阈值(米)
    bool publish_astar_raw_path_ = true;  // 发布原始A*路径（平滑前）

    // A* 台阶感知形态参数（不含外切奖励）
    bool astar_stair_shape_enable_ = false;
    int astar_stair_search_radius_cells_ = 4;
    double astar_stair_trigger_dist_m_ = 1.2;
    double astar_stair_tangent_penalty_weight_ = 2.0;
    double astar_stair_centerline_penalty_weight_ = 1.0;
    bool astar_stair_lock_cluster_center_ = true;
    // A* 台阶段窗口长度（与 B-spline stair_align_*_pre/post 参数同源）
    double astar_stair_up_pre_dist_m_ = 0.6;
    double astar_stair_up_post_dist_m_ = 0.6;
    double astar_stair_down_pre_dist_m_ = 0.6;
    double astar_stair_down_post_dist_m_ = 0.6;

    // A* 单次规划内锁定的台阶簇中心与法向
    bool astar_stair_cluster_locked_ = false;
    Eigen::Vector2d astar_stair_locked_center_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d astar_stair_locked_normal_ = Eigen::Vector2d::Zero();

    // B-spline 锚点与 A* 台阶命中点对齐
    bool use_astar_stair_anchors_ = true;
    double astar_stair_anchor_match_max_dist_m_ = 0.6;
    
    // 调试可视化
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr astar_raw_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ctrl_pts_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stair_debug_pub_;
    void publishControlPoints(const std::vector<Eigen::Vector2d>& ctrl_pts, 
                              const std::string& frame_id);
    void publishStairDebugMarkers(const StairAlignDiagnostics& diag,
                                  const std::string& frame_id);
};

}  // namespace nav_components
