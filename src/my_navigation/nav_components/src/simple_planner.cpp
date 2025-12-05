// nav_components/src/simple_planner.cpp
// A* 路径规划 + B样条平滑 + B样条优化

#include "nav_components/simple_planner.hpp"
#include "nav_components/map_manager.hpp"
#include <cmath>
#include <algorithm>

namespace nav_components {

void SimplePlanner::initialize(rclcpp::Node* node) {
    node_ = node;
    obstacle_threshold_ = node_->declare_parameter("planner.obstacle_threshold", 99);
    cost_weight_ = node_->declare_parameter("planner.cost_weight", 0.5);
    enable_smooth_ = node_->declare_parameter("planner.enable_smooth", true);
    
    // 平滑参数
    SmoothParams smooth_params;
    smooth_params.resample_interval = node_->declare_parameter("planner.resample_interval", 0.3);
    smooth_params.output_resolution = node_->declare_parameter("planner.smooth_resolution", 0.05);
    
    // 优化参数
    smooth_params.enable_optimization = node_->declare_parameter("planner.enable_optimization", true);
    smooth_params.lambda_smooth = node_->declare_parameter("planner.opt_lambda_smooth", 10.0);
    smooth_params.lambda_collision = node_->declare_parameter("planner.opt_lambda_collision", 5.0);
    smooth_params.safe_dist = node_->declare_parameter("planner.opt_safe_dist", 0.3);
    smooth_params.opt_max_iter = node_->declare_parameter("planner.opt_max_iter", 50);
    
    // 曲率自适应参数
    smooth_params.curvature_adaptive = node_->declare_parameter("planner.opt_curvature_adaptive", true);
    smooth_params.curvature_weight = node_->declare_parameter("planner.opt_curvature_weight", 2.0);
    
    // 窄通道中心吸引参数
    smooth_params.narrow_passage_align = node_->declare_parameter("planner.narrow_passage_align", true);
    smooth_params.narrow_passage_thresh = node_->declare_parameter("planner.narrow_passage_thresh", 0.6);
    smooth_params.lambda_align = node_->declare_parameter("planner.opt_lambda_align", 3.0);
    smooth_params.align_entry_points = node_->declare_parameter("planner.align_entry_points", 2);
    smooth_params.narrow_min_consecutive = node_->declare_parameter("planner.narrow_min_consecutive", 3);
    
    smoother_.setParams(smooth_params);
    
    // 创建调试可视化发布器
    ctrl_pts_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "debug/control_points", 10);
}

void SimplePlanner::setMap(nav_core::MapInterface::Ptr map) {
    map_ = map;
    
    if (!map) {
        grid_map_ = nullptr;
        map_manager_ = nullptr;
        return;
    }
    
    // COSTMAP 和 OCCUPANCY_GRID 格式兼容
    auto map_type = map->type();
    if (map_type != nav_core::MapType::OCCUPANCY_GRID && 
        map_type != nav_core::MapType::COSTMAP) {
        RCLCPP_ERROR(node_->get_logger(), 
            "SimplePlanner 仅支持 OccupancyGrid/Costmap 地图");
        grid_map_ = nullptr;
        return;
    }
    
    // 从 MapManager 获取 costmap（膨胀后的地图）
    map_manager_ = std::dynamic_pointer_cast<MapManager>(map);
    if (map_manager_ && map_manager_->getCostmap()) {
        grid_map_ = map_manager_->getCostmap();
        width_ = grid_map_->info.width;
        height_ = grid_map_->info.height;
        resolution_ = grid_map_->info.resolution;
        origin_x_ = grid_map_->info.origin.position.x;
        origin_y_ = grid_map_->info.origin.position.y;
        
        // 设置 ESDF 回调用于 B样条优化
        if (map_manager_->hasEsdf()) {
            smoother_.setESDFCallback(
                [this](double x, double y, double* gx, double* gy) -> double {
                    return map_manager_->getEsdfDistanceWithGradient(x, y, gx, gy);
                });
            RCLCPP_INFO(node_->get_logger(), "B样条优化: ESDF 回调已设置");
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
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return false;
    }
    int8_t val = grid_map_->data[y * width_ + x];
    return val >= 0 && val < obstacle_threshold_;
}

// 获取代价值（归一化到 0~1）
double SimplePlanner::getCost(int x, int y) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return 1.0;
    }
    int8_t val = grid_map_->data[y * width_ + x];
    if (val < 0 || val >= obstacle_threshold_) {
        return 1.0;
    }
    return static_cast<double>(val) / 100.0;
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
    const double move_cost[] = {1, 1.414, 1, 1.414, 1, 1.414, 1, 1.414};
    
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
        if (closed[idx]) {
            continue;
        }
        closed[idx] = true;
        
        for (int i = 0; i < 8; i++) {
            int nx = curr->x + dx[i];
            int ny = curr->y + dy[i];
            
            if (!isValid(nx, ny)) {
                continue;
            }
            int nidx = ny * width_ + nx;
            if (closed[nidx]) {
                continue;
            }
            
            // 代价感知：移动成本 + 代价惩罚
            double cell_cost = getCost(nx, ny);
            double total_cost = move_cost[i] + cost_weight_ * cell_cost;
            
            auto new_node = std::make_unique<Node>();
            new_node->x = nx; new_node->y = ny;
            new_node->g = curr->g + total_cost;
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
    
    // 构造原始路径
    nav_msgs::msg::Path raw_path;
    raw_path.header = goal.header;
    raw_path.poses = waypoints;
    
    // B样条平滑
    if (enable_smooth_ && waypoints.size() >= 4) {
        if (smoother_.smooth(raw_path, path)) {
            RCLCPP_INFO(node_->get_logger(), "规划成功: A*=%zu点 -> 平滑=%zu点", 
                        waypoints.size(), path.poses.size());
            
            // 发布控制点用于调试
            publishControlPoints(smoother_.getLastControlPoints(), goal.header.frame_id);
        } else {
            path = raw_path;
            RCLCPP_INFO(node_->get_logger(), "规划成功 (平滑失败): %zu点", waypoints.size());
        }
    } else {
        path = raw_path;
        RCLCPP_INFO(node_->get_logger(), "规划成功: %zu点", waypoints.size());
    }
    
    return true;
}

void SimplePlanner::publishControlPoints(const std::vector<Eigen::Vector2d>& ctrl_pts,
                                          const std::string& frame_id) {
    if (!ctrl_pts_pub_ || ctrl_pts.empty()) return;
    
    visualization_msgs::msg::MarkerArray markers;
    
    // Marker 1: 控制点球体
    visualization_msgs::msg::Marker points_marker;
    points_marker.header.frame_id = frame_id;
    points_marker.header.stamp = node_->now();
    points_marker.ns = "control_points";
    points_marker.id = 0;
    points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points_marker.action = visualization_msgs::msg::Marker::ADD;
    points_marker.scale.x = 0.08;
    points_marker.scale.y = 0.08;
    points_marker.scale.z = 0.08;
    points_marker.color.r = 1.0;
    points_marker.color.g = 0.5;
    points_marker.color.b = 0.0;
    points_marker.color.a = 1.0;
    
    for (const auto& pt : ctrl_pts) {
        geometry_msgs::msg::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = 0.05;
        points_marker.points.push_back(p);
    }
    markers.markers.push_back(points_marker);
    
    // Marker 2: 控制多边形（连线）
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = frame_id;
    line_marker.header.stamp = node_->now();
    line_marker.ns = "control_polygon";
    line_marker.id = 1;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.02;
    line_marker.color.r = 0.0;
    line_marker.color.g = 0.8;
    line_marker.color.b = 0.2;
    line_marker.color.a = 0.8;
    
    for (const auto& pt : ctrl_pts) {
        geometry_msgs::msg::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = 0.05;
        line_marker.points.push_back(p);
    }
    markers.markers.push_back(line_marker);
    
    // Marker 3: 序号标签
    for (size_t i = 0; i < ctrl_pts.size(); ++i) {
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = frame_id;
        text_marker.header.stamp = node_->now();
        text_marker.ns = "control_point_ids";
        text_marker.id = static_cast<int>(i + 100);
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = ctrl_pts[i].x();
        text_marker.pose.position.y = ctrl_pts[i].y();
        text_marker.pose.position.z = 0.15;
        text_marker.scale.z = 0.1;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = std::to_string(i);
        markers.markers.push_back(text_marker);
    }
    
    ctrl_pts_pub_->publish(markers);
}

}  // namespace nav_components
