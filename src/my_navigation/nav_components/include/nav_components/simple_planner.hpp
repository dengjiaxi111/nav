// nav_components/include/nav_components/simple_planner.hpp
// 简单A*规划器

#pragma once
#include <nav_core/planner_base.hpp>
#include <queue>
#include <unordered_set>

namespace nav_components {

class SimplePlanner : public nav_core::PlannerBase {
public:
    void initialize(rclcpp::Node* node) override;
    void setMap(nav_msgs::msg::OccupancyGrid::SharedPtr map) override;
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
    double heuristic(int x1, int y1, int x2, int y2);
    
    rclcpp::Node* node_ = nullptr;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    int width_ = 0, height_ = 0;
    double resolution_ = 0.05;
    double origin_x_ = 0, origin_y_ = 0;
    int8_t obstacle_threshold_ = 50;  // 障碍物阈值
};

}  // namespace nav_components
