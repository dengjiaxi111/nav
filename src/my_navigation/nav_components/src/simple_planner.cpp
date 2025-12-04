// nav_components/src/simple_planner.cpp
// A*路径规划实现 - 仅支持 OccupancyGrid

#include "nav_components/simple_planner.hpp"
#include "nav_components/grid_map_adapter.hpp"
#include <cmath>
#include <algorithm>

namespace nav_components {

void SimplePlanner::initialize(rclcpp::Node* node) {
    node_ = node;
    obstacle_threshold_ = node_->declare_parameter("planner.obstacle_threshold", 50);
}

void SimplePlanner::setMap(nav_core::MapInterface::Ptr map) {
    map_ = map;  // 保存基类指针
    
    if (!map) {
        grid_map_ = nullptr;
        return;
    }
    
    // 检查地图类型
    if (!map->isOccupancyGrid()) {
        RCLCPP_ERROR(node_->get_logger(), 
            "SimplePlanner 仅支持 OccupancyGrid 地图，当前地图类型不兼容");
        grid_map_ = nullptr;
        return;
    }
    
    // 转换为 GridMapAdapter 获取原始地图
    auto adapter = std::dynamic_pointer_cast<GridMapAdapter>(map);
    if (adapter) {
        grid_map_ = adapter->getOccupancyGrid();
        if (grid_map_) {
            width_ = grid_map_->info.width;
            height_ = grid_map_->info.height;
            resolution_ = grid_map_->info.resolution;
            origin_x_ = grid_map_->info.origin.position.x;
            origin_y_ = grid_map_->info.origin.position.y;
        }
    }
}

bool SimplePlanner::worldToMap(double wx, double wy, int& mx, int& my) {
    mx = static_cast<int>((wx - origin_x_) / resolution_);
    my = static_cast<int>((wy - origin_y_) / resolution_);
    return mx >= 0 && mx < width_ && my >= 0 && my < height_;
}

void SimplePlanner::mapToWorld(int mx, int my, double& wx, double& wy) {
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
}

bool SimplePlanner::isValid(int x, int y) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) return false;
    int8_t val = grid_map_->data[y * width_ + x];
    return val >= 0 && val < obstacle_threshold_;
}

double SimplePlanner::heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

bool SimplePlanner::plan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    nav_msgs::msg::Path& path)
{
    if (!grid_map_) {
        RCLCPP_ERROR(node_->get_logger(), "地图未设置或类型不兼容");
        return false;
    }
    
    int sx, sy, gx, gy;
    if (!worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
        !worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
        RCLCPP_ERROR(node_->get_logger(), "起点或终点超出地图范围");
        return false;
    }
    
    if (!isValid(sx, sy)) {
        RCLCPP_ERROR(node_->get_logger(), "起点在障碍物中");
        return false;
    }
    if (!isValid(gx, gy)) {
        RCLCPP_ERROR(node_->get_logger(), "终点在障碍物中");
        return false;
    }
    
    // A*搜索
    auto cmp = [](Node* a, Node* b) { return a->f() > b->f(); };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);
    std::vector<std::unique_ptr<Node>> all_nodes;
    std::vector<bool> closed(width_ * height_, false);
    
    auto start_node = std::make_unique<Node>();
    start_node->x = sx; start_node->y = sy;
    start_node->g = 0; start_node->h = heuristic(sx, sy, gx, gy);
    open.push(start_node.get());
    all_nodes.push_back(std::move(start_node));
    
    // 8方向
    const int dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};
    const double cost[] = {1, 1.414, 1, 1.414, 1, 1.414, 1, 1.414};
    
    Node* goal_node = nullptr;
    int iterations = 0;
    const int max_iter = width_ * height_;
    
    while (!open.empty() && iterations++ < max_iter) {
        Node* curr = open.top();
        open.pop();
        
        if (curr->x == gx && curr->y == gy) {
            goal_node = curr;
            break;
        }
        
        int idx = curr->y * width_ + curr->x;
        if (closed[idx]) continue;
        closed[idx] = true;
        
        for (int i = 0; i < 8; i++) {
            int nx = curr->x + dx[i];
            int ny = curr->y + dy[i];
            
            if (!isValid(nx, ny)) continue;
            int nidx = ny * width_ + nx;
            if (closed[nidx]) continue;
            
            auto new_node = std::make_unique<Node>();
            new_node->x = nx; new_node->y = ny;
            new_node->g = curr->g + cost[i];
            new_node->h = heuristic(nx, ny, gx, gy);
            new_node->parent = curr;
            
            open.push(new_node.get());
            all_nodes.push_back(std::move(new_node));
        }
    }
    
    if (!goal_node) {
        RCLCPP_WARN(node_->get_logger(), "A*未找到路径");
        return false;
    }
    
    // 回溯路径
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    for (Node* n = goal_node; n != nullptr; n = n->parent) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = goal.header;
        mapToWorld(n->x, n->y, pose.pose.position.x, pose.pose.position.y);
        pose.pose.orientation.w = 1.0;
        waypoints.push_back(pose);
    }
    
    std::reverse(waypoints.begin(), waypoints.end());
    path.header = goal.header;
    path.poses = waypoints;
    
    RCLCPP_INFO(node_->get_logger(), "规划成功，路径点数: %zu", waypoints.size());
    return true;
}

}  // namespace nav_components
