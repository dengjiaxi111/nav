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
class MapManager;

class SimplePlanner : public nav_core::PlannerBase {
public:
    void initialize(rclcpp::Node* node) override;
    
    // 重写 setMap，检查地图类型
    void setMap(nav_core::MapInterface::Ptr map) override;
    
    bool plan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        nav_msgs::msg::Path& path) override;

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
    
    rclcpp::Node* node_ = nullptr;
    
    // 地图
    nav_msgs::msg::OccupancyGrid::SharedPtr grid_map_;
    std::shared_ptr<MapManager> map_manager_;  // 用于 ESDF 查询
    int8_t obstacle_threshold_ = 99;
    double cost_weight_ = 0.5;
    
    // 缓存的地图信息
    double resolution_ = 0.05;
    double origin_x_ = 0, origin_y_ = 0;
    int width_ = 0, height_ = 0;
    
    // 路径平滑
    PathSmoother smoother_;
    bool enable_smooth_ = true;
    
    // 调试可视化
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ctrl_pts_pub_;
    void publishControlPoints(const std::vector<Eigen::Vector2d>& ctrl_pts, 
                              const std::string& frame_id);
};

}  // namespace nav_components
