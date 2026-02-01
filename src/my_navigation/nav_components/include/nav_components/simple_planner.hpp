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
    
    // 重写 setMap，检查地图类型
    void setMap(nav_core::MapInterface::Ptr map) override;
    
    bool plan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        nav_msgs::msg::Path& path) override;
    
    // 公开接口：路径验证（供 nav_server 周期性检查使用）
    bool validatePath(const nav_msgs::msg::Path& path);

private:
    // A*节点
    struct Node {
        int x, y;
        double g, h;
        Node* parent = nullptr;
        double f() const { return g + h; }
    };
    
    // 坐标转换
    bool worldToMap(double wx, double wy, int& mx, int& my);
    void mapToWorld(int mx, int my, double& wx, double& wy);
    bool isValid(int x, int y);
    double getCost(int x, int y);
    double heuristic(int x1, int y1, int x2, int y2);
    
    // 内部功能：路径检查和缓存
    bool goalChanged(const geometry_msgs::msg::PoseStamped& new_goal);  // 检查目标是否变化
    nav_msgs::msg::Path prunePath(const nav_msgs::msg::Path& path, 
                                   const geometry_msgs::msg::PoseStamped& current_pose);  // 剪枝已驶过部分
    
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
    double goal_tolerance_ = 0.2;  // 目标变化阈值(米)
    double obstacle_check_threshold_ = 95;  // 障碍物检查阈值
    bool enable_path_cache_ = true;  // 启用路径缓存
    bool enable_auto_prune_ = true;  // 启用自动剪枝
    double prune_distance_ = 0.5;  // 剪枝距离阈值(米)
    
    // 调试可视化
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ctrl_pts_pub_;
    void publishControlPoints(const std::vector<Eigen::Vector2d>& ctrl_pts, 
                              const std::string& frame_id);
};

}  // namespace nav_components
