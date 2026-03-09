// nav_components/src/simple_planner.cpp
// A* 路径规划 + B样条平滑 + B样条优化

#include "nav_components/simple_planner.hpp"
#include "nav_components/layered_map_manager.hpp"
#include <cmath>
#include <algorithm>

namespace nav_components {

void SimplePlanner::initialize(rclcpp::Node* node) {
    node_ = node;
    obstacle_threshold_ = node_->declare_parameter("planner.obstacle_threshold", 99);
    cost_weight_ = node_->declare_parameter("planner.cost_weight", 0.5);
    enable_smooth_ = node_->declare_parameter("planner.enable_smooth", true);
    
    enable_path_cache_ = node_->declare_parameter("planner.enable_path_cache", true);
    enable_auto_prune_ = node_->declare_parameter("planner.enable_auto_prune", true);
    goal_tolerance_ = node_->declare_parameter("planner.goal_tolerance", 0.2);
    obstacle_check_threshold_ = node_->declare_parameter("planner.obstacle_check_threshold", 95);
    esdf_warn_min_safe_dist_ = node_->declare_parameter("planner.esdf_warn_min_safe_dist", 0.12);
    start_deviation_threshold_ = node_->declare_parameter("planner.start_deviation_threshold", 1.0);
    astar_max_attempts_ = node_->declare_parameter("planner.astar_max_attempts", 2);
    astar_threshold_step_ = node_->declare_parameter("planner.astar_threshold_step", 10);
    prune_distance_ = node_->declare_parameter("planner.prune_distance", 0.5);

    if (astar_max_attempts_ < 1) {
        RCLCPP_WARN(node_->get_logger(),
            "planner.astar_max_attempts=%d 非法，自动修正为 1", astar_max_attempts_);
        astar_max_attempts_ = 1;
    }
    if (astar_threshold_step_ < 0) {
        RCLCPP_WARN(node_->get_logger(),
            "planner.astar_threshold_step=%d 非法，自动修正为 0", astar_threshold_step_);
        astar_threshold_step_ = 0;
    }
    
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

    // 台阶段局部方向优化（通过台阶时尽量垂直边沿）
    // stair_align_expand_points: 以台阶命中控制点为中心，前后各扩展 n 个点参与该项优化
    smooth_params.stair_segment_align = node_->declare_parameter("planner.stair_segment_align", true);
    smooth_params.lambda_stair_align = node_->declare_parameter("planner.opt_lambda_stair_align", 6.0);
    smooth_params.stair_align_expand_points =
        node_->declare_parameter("planner.stair_align_expand_points", 2);

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
    
    // 统一使用 LayeredMapManager 提供的 costmap（膨胀后的地图）
    map_manager_ = std::dynamic_pointer_cast<LayeredMapManager>(map);
    if (map_manager_ && map_manager_->getCostmap()) {
        grid_map_ = map_manager_->getCostmap();
        width_ = grid_map_->info.width;
        height_ = grid_map_->info.height;
        resolution_ = grid_map_->info.resolution;
        origin_x_ = grid_map_->info.origin.position.x;
        origin_y_ = grid_map_->info.origin.position.y;
        
        if (map_manager_->hasEsdf()) {
            smoother_.setESDFCallback(
                [this](double x, double y, double* gx, double* gy) -> double {
                    return map_manager_->getEsdfDistanceWithGradient(x, y, gx, gy);
                });
            RCLCPP_INFO(node_->get_logger(), "B样条优化: ESDF 回调已设置");
        }

        smoother_.setStairNormalCallback(
            [this](double x, double y, double* nx, double* ny) -> bool {
                if (!map_manager_) {
                    return false;
                }
                double local_nx = 0.0;
                double local_ny = 0.0;
                if (!map_manager_->getStairTraverseNormal(x, y, local_nx, local_ny)) {
                    return false;
                }
                if (nx) {
                    *nx = local_nx;
                }
                if (ny) {
                    *ny = local_ny;
                }
                return true;
            });
        RCLCPP_INFO(node_->get_logger(), "B样条优化: 台阶法向回调已设置");
    } else {
        RCLCPP_ERROR(node_->get_logger(), 
            "SimplePlanner: 需要 LayeredMapManager 且 costmap 可用");
        grid_map_ = nullptr;
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

void SimplePlanner::clearCache() {
    cached_path_ = nav_msgs::msg::Path();
    cached_goal_ = geometry_msgs::msg::PoseStamped();
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

bool SimplePlanner::findNearestFreeCell(int cx, int cy, int& fx, int& fy, int max_radius) {
    // BFS 螺旋搜索：从 (cx, cy) 开始逐层扩展，找到最近的可通行格子
    // 返回 true 表示找到，结果存入 (fx, fy)
    for (int r = 1; r <= max_radius; ++r) {
        // 搜索正方形环上的所有格子
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                // 只搜索外环（跳过内部已搜过的区域）
                if (std::abs(dx) != r && std::abs(dy) != r) continue;
                
                int nx = cx + dx;
                int ny = cy + dy;
                if (isValid(nx, ny)) {
                    fx = nx;
                    fy = ny;
                    return true;
                }
            }
        }
    }
    return false;
}

bool SimplePlanner::plan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    nav_msgs::msg::Path& path)
{
    
    if (map_manager_) {
        auto latest_costmap = map_manager_->getCostmap();
        if (latest_costmap) {
            grid_map_ = latest_costmap;
            width_ = grid_map_->info.width;
            height_ = grid_map_->info.height;
            resolution_ = grid_map_->info.resolution;
            origin_x_ = grid_map_->info.origin.position.x;
            origin_y_ = grid_map_->info.origin.position.y;
        }
    }
    
    if (!grid_map_) {
        RCLCPP_ERROR(node_->get_logger(), "地图未设置或 costmap 不可用");
        return false;
    }
    
    if (enable_path_cache_ && !cached_path_.poses.empty()) {
        if (!goalChanged(goal)) {
            // 目标未变化，先检查起点是否偏离缓存路径过远
            double min_dist_to_path = std::numeric_limits<double>::max();
            for (const auto& pose : cached_path_.poses) {
                double dx = start.pose.position.x - pose.pose.position.x;
                double dy = start.pose.position.y - pose.pose.position.y;
                double dist = std::hypot(dx, dy);
                min_dist_to_path = std::min(min_dist_to_path, dist);
            }
            
            if (min_dist_to_path > start_deviation_threshold_) {
                RCLCPP_WARN(node_->get_logger(), 
                    "规划器: 起点偏离缓存路径过远 (%.2f m > %.2f m)，重新规划",
                    min_dist_to_path, start_deviation_threshold_);
                // 清空缓存，强制重新规划
                cached_path_ = nav_msgs::msg::Path();
            } 
            else if (validatePath(cached_path_)) {
                // 起点在路径附近且路径无障碍物，复用缓存
                
                if (enable_auto_prune_) {
                    path = prunePath(cached_path_, start);
                } else {
                    path = cached_path_;
                }
                return true;
            } else {
                // 清空缓存，强制重新规划
                cached_path_ = nav_msgs::msg::Path();
            }
        }
    }
    
    int sx, sy, gx, gy;
    if (!worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
        !worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
        RCLCPP_ERROR(node_->get_logger(), "起点或终点超出地图范围");
        return false;
    }
    
    // 起点终点有效性检查
    if (!isValid(sx, sy)) {
        RCLCPP_ERROR(node_->get_logger(), "起点(%d,%d)在障碍物中", sx, sy);
        return false;
    }
    
    if (!isValid(gx, gy)) {
        // 终点在障碍物中，尝试调整到附近可通行点
        int free_x, free_y;
        if (findNearestFreeCell(gx, gy, free_x, free_y)) {
            double wx, wy;
            mapToWorld(free_x, free_y, wx, wy);
            double escape_dist = std::hypot(free_x - gx, free_y - gy) * resolution_;
            RCLCPP_WARN(node_->get_logger(), 
                "终点(%d,%d)在障碍物中，调整到最近可通行点(%d,%d)，距离%.2fm",
                gx, gy, free_x, free_y, escape_dist);
            gx = free_x;
            gy = free_y;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "终点在障碍物中且无法找到附近可通行格子");
            return false;
        }
    }
    
    const int max_attempts = astar_max_attempts_;
    const int threshold_step = astar_threshold_step_;
    const int original_threshold = obstacle_threshold_;
    int attempts_done = 0;
    
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        attempts_done++;
        int current_threshold = std::clamp(original_threshold - attempt * threshold_step, 1, 100);
        
        obstacle_threshold_ = current_threshold;
        
        nav_msgs::msg::Path raw_path;
        bool astar_success = runAstar(sx, sy, gx, gy, goal.header, raw_path);
        
        obstacle_threshold_ = original_threshold;
        
        if (!astar_success) {
            RCLCPP_WARN(node_->get_logger(), 
                "A*规划失败 (阈值=%d)", current_threshold);
            continue;
        }
        
        nav_msgs::msg::Path smoothed_path;
        if (enable_smooth_ && raw_path.poses.size() >= 4) {
            if (!smoother_.smooth(raw_path, smoothed_path)) {
                smoothed_path = raw_path;
                RCLCPP_WARN(node_->get_logger(), "平滑失败，使用原始路径");
            } else {
                if (map_manager_ && map_manager_->hasEsdf()) {
                    double min_dist = 1e9;
                    geometry_msgs::msg::PoseStamped worst_pose;
                    int unsafe_count = 0;
                    const double min_safe_dist = esdf_warn_min_safe_dist_;
                    
                    for (const auto& pose : smoothed_path.poses) {
                        double grad_x = 0, grad_y = 0;
                        double dist = map_manager_->getEsdfDistanceWithGradient(
                            pose.pose.position.x, pose.pose.position.y, &grad_x, &grad_y);
                        
                        if (dist < min_dist) {
                            min_dist = dist;
                            worst_pose = pose;
                        }
                        if (dist < min_safe_dist) {
                            unsafe_count++;
                        }
                    }
                    
                    if (unsafe_count > 0) {
                        RCLCPP_WARN(node_->get_logger(), 
                            "平滑路径过于接近障碍物: %d/%zu 点 ESDF < %.2fm, 最小=%.3fm @ (%.2f, %.2f)",
                            unsafe_count, smoothed_path.poses.size(), min_safe_dist, min_dist,
                            worst_pose.pose.position.x, worst_pose.pose.position.y);
                    }
                }
                
                publishControlPoints(smoother_.getLastControlPoints(), goal.header.frame_id);
            }
        } else {
            smoothed_path = raw_path;
        }
        
        if (validatePath(smoothed_path)) {
            RCLCPP_DEBUG(node_->get_logger(), 
                "规划成功 (阈值=%d): A*=%zu点 -> 平滑=%zu点", 
                current_threshold, raw_path.poses.size(), smoothed_path.poses.size());
            path = smoothed_path;
            
            if (enable_path_cache_) {
                cached_path_ = path;
                cached_goal_ = goal;
            }
            return true;
        } else {
            RCLCPP_DEBUG(node_->get_logger(), 
                "路径验证失败 (阈值=%d)，尝试降低阈值", current_threshold);
        }

        if (threshold_step == 0) {
            break;
        }
    }
    
    RCLCPP_ERROR(node_->get_logger(),
        "规划失败: 尝试了 %d 次，均未成功 (step=%d)",
        attempts_done, threshold_step);
    return false;
}

bool SimplePlanner::runAstar(int sx, int sy, int gx, int gy,
                              const std_msgs::msg::Header& header,
                              nav_msgs::msg::Path& path) {
    auto cmp = [](Node* a, Node* b) { return a->f() > b->f(); };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);
    std::vector<std::unique_ptr<Node>> all_nodes;
    std::vector<bool> closed(width_ * height_, false);
    
    auto start_node = std::make_unique<Node>();
    start_node->x = sx; start_node->y = sy;
    start_node->g = 0; start_node->h = heuristic(sx, sy, gx, gy);
    open.push(start_node.get());
    all_nodes.push_back(std::move(start_node));
    
    const int dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};
    const double move_cost[] = {1, 1.414, 1, 1.414, 1, 1.414, 1, 1.414};
    
    Node* goal_node = nullptr;
    int iterations = 0;
    const int max_iter = std::min(width_ * height_ * 2, 1000000);
    
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

            if (map_manager_ && !map_manager_->isTransitionAllowed(curr->x, curr->y, nx, ny)) {
                continue;
            }
            
            if (!isValid(nx, ny)) {
                continue;
            }
            int nidx = ny * width_ + nx;
            if (closed[nidx]) {
                continue;
            }
            
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
        return false;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    for (Node* n = goal_node; n != nullptr; n = n->parent) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = header;
        mapToWorld(n->x, n->y, pose.pose.position.x, pose.pose.position.y);
        pose.pose.orientation.w = 1.0;
        waypoints.push_back(pose);
    }
    
    std::reverse(waypoints.begin(), waypoints.end());
    
    path.header = header;
    path.poses = waypoints;
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

bool SimplePlanner::validatePath(const nav_msgs::msg::Path& path) {
    if (path.poses.empty() || !map_manager_) {
        return false;
    }
    
    auto latest_costmap = map_manager_->getCostmap();
    if (!latest_costmap) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "无法获取最新Costmap进行路径验证");
        return false;
    }
    
    int cm_width = latest_costmap->info.width;
    int cm_height = latest_costmap->info.height;
    double cm_resolution = latest_costmap->info.resolution;
    double cm_origin_x = latest_costmap->info.origin.position.x;
    double cm_origin_y = latest_costmap->info.origin.position.y;
    
    int8_t max_cost_on_path = 0;
    
    for (const auto& pose : path.poses) {
        int mx = static_cast<int>((pose.pose.position.x - cm_origin_x) / cm_resolution);
        int my = static_cast<int>((pose.pose.position.y - cm_origin_y) / cm_resolution);
        
        if (mx < 0 || mx >= cm_width || my < 0 || my >= cm_height) {
            continue;
        }
        
        int idx = my * cm_width + mx;
        if (idx < 0 || idx >= static_cast<int>(latest_costmap->data.size())) {
            continue;
        }
        
        int8_t val = latest_costmap->data[idx];
        if (val > max_cost_on_path) {
            max_cost_on_path = val;
        }
        
        if (val >= obstacle_check_threshold_ || val < 0) {
            RCLCPP_WARN(node_->get_logger(), 
                "路径点 (%.2f, %.2f) 与障碍物重合: costmap=%d >= %d", 
                pose.pose.position.x, pose.pose.position.y, 
                static_cast<int>(val), static_cast<int>(obstacle_check_threshold_));
            return false;
        }
    }
    
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "路径验证通过: %zu点, 路径最大代价=%d, 阈值=%d",
        path.poses.size(), static_cast<int>(max_cost_on_path),
        static_cast<int>(obstacle_check_threshold_));
    
    return true;
}

bool SimplePlanner::goalChanged(const geometry_msgs::msg::PoseStamped& new_goal) {
    if (cached_goal_.header.frame_id.empty()) {
        return true;
    }
    
    double dx = new_goal.pose.position.x - cached_goal_.pose.position.x;
    double dy = new_goal.pose.position.y - cached_goal_.pose.position.y;
    double dist = std::hypot(dx, dy);
    
    return dist > goal_tolerance_;
}

nav_msgs::msg::Path SimplePlanner::prunePath(
    const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::PoseStamped& current_pose)
{
    if (path.poses.empty()) {
        return path;
    }
    
    double min_dist = std::numeric_limits<double>::max();
    size_t nearest_idx = 0;
    
    for (size_t i = 0; i < path.poses.size(); ++i) {
        double dx = path.poses[i].pose.position.x - current_pose.pose.position.x;
        double dy = path.poses[i].pose.position.y - current_pose.pose.position.y;
        double dist = std::hypot(dx, dy);
        
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    
    size_t prune_start_idx = 0;
    double accumulated_dist = 0.0;
    
    for (size_t i = 0; i < nearest_idx; ++i) {
        double dx = path.poses[nearest_idx].pose.position.x - path.poses[i].pose.position.x;
        double dy = path.poses[nearest_idx].pose.position.y - path.poses[i].pose.position.y;
        accumulated_dist = std::hypot(dx, dy);
        
        if (accumulated_dist <= prune_distance_) {
            prune_start_idx = i;
            break;
        }
    }
    
    nav_msgs::msg::Path pruned_path;
    pruned_path.header = path.header;
    pruned_path.poses.assign(path.poses.begin() + prune_start_idx, path.poses.end());
    
    return pruned_path;
}

}  // namespace nav_components
