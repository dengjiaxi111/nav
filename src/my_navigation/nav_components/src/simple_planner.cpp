// nav_components/src/simple_planner.cpp
// A* 路径规划 + B样条平滑 + B样条优化

#include "nav_components/simple_planner.hpp"
#include "nav_components/layered_map_manager.hpp"
#include <std_msgs/msg/color_rgba.hpp>
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstdio>
#include <cctype>

namespace nav_components {

void SimplePlanner::initialize(rclcpp::Node* node) {
    node_ = node;
    obstacle_threshold_ = node_->declare_parameter("planner.obstacle_threshold", 99);
    cost_weight_ = node_->declare_parameter("planner.cost_weight", 0.5);
    enable_smooth_ = node_->declare_parameter("planner.enable_smooth", true);
    
    enable_path_cache_ = node_->declare_parameter("planner.enable_path_cache", true);
    enable_auto_prune_ = node_->declare_parameter("planner.enable_auto_prune", true);
    goal_change_tolerance_ = node_->declare_parameter("planner.goal_change_tolerance", 0.2);
    obstacle_check_threshold_ = node_->declare_parameter("planner.obstacle_check_threshold", 95);
    esdf_warn_min_safe_dist_ = node_->declare_parameter("planner.esdf_warn_min_safe_dist", 0.12);
    start_deviation_threshold_ = node_->declare_parameter("planner.start_deviation_threshold", 1.0);
    astar_max_attempts_ = node_->declare_parameter("planner.astar_max_attempts", 2);
    astar_max_iterations_ = node_->declare_parameter("planner.astar_max_iterations", 0);
    astar_threshold_step_ = node_->declare_parameter("planner.astar_threshold_step", 10);
    prune_distance_ = node_->declare_parameter("planner.prune_distance", 0.5);
    publish_astar_raw_path_ = node_->declare_parameter("planner.publish_astar_raw_path", true);
    allow_raw_fallback_on_smooth_fail_ =
        node_->declare_parameter("planner.allow_raw_fallback_on_smooth_fail", false);
    stair_constraint_mode_ =
        node_->declare_parameter("planner.stair_constraint_mode", std::string("soft"));
    stair_hard_dist_delta_m_ =
        node_->declare_parameter("planner.stair_hard_dist_delta_m", 0.0);
    publish_failure_debug_markers_ =
        node_->declare_parameter("planner.publish_failure_debug_markers", true);
    failure_marker_scale_ =
        node_->declare_parameter("planner.failure_marker_scale", 0.14);
    astar_stair_shape_enable_ =
        node_->declare_parameter("planner.astar_stair_shape_enable", false);
    astar_stair_search_radius_cells_ =
        node_->declare_parameter("planner.astar_stair_search_radius_cells", 4);
    astar_stair_trigger_dist_m_ =
        node_->declare_parameter("planner.astar_stair_trigger_dist_m", 1.2);
    astar_stair_tangent_penalty_weight_ =
        node_->declare_parameter("planner.astar_stair_tangent_penalty_weight", 2.0);
    astar_stair_centerline_penalty_weight_ =
        node_->declare_parameter("planner.astar_stair_centerline_penalty_weight", 1.0);
    astar_stair_lock_cluster_center_ =
        node_->declare_parameter("planner.astar_stair_lock_cluster_center", true);
    use_astar_stair_anchors_ =
        node_->declare_parameter("planner.use_astar_stair_anchors", true);
    astar_stair_anchor_match_max_dist_m_ =
        node_->declare_parameter("planner.astar_stair_anchor_match_max_dist_m", 0.6);

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
    if (astar_stair_search_radius_cells_ < 1) {
        RCLCPP_WARN(node_->get_logger(),
            "planner.astar_stair_search_radius_cells=%d 非法，自动修正为1",
            astar_stair_search_radius_cells_);
        astar_stair_search_radius_cells_ = 1;
    }
    if (astar_stair_trigger_dist_m_ < 0.0) {
        RCLCPP_WARN(node_->get_logger(),
            "planner.astar_stair_trigger_dist_m=%.3f 非法，自动修正为0",
            astar_stair_trigger_dist_m_);
        astar_stair_trigger_dist_m_ = 0.0;
    }
    if (astar_stair_anchor_match_max_dist_m_ < 0.0) {
        astar_stair_anchor_match_max_dist_m_ = 0.0;
    }
    std::transform(stair_constraint_mode_.begin(), stair_constraint_mode_.end(),
                   stair_constraint_mode_.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (stair_constraint_mode_ != "soft" && stair_constraint_mode_ != "hard") {
        RCLCPP_WARN(node_->get_logger(),
            "planner.stair_constraint_mode='%s' 非法，回退为 soft",
            stair_constraint_mode_.c_str());
        stair_constraint_mode_ = "soft";
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
    const double stair_legacy_window_m =
        std::max(0.0, smooth_params.stair_align_expand_points * smooth_params.resample_interval);
    smooth_params.stair_align_up_pre_dist_m =
        node_->declare_parameter("planner.stair_align_up_pre_dist_m", stair_legacy_window_m);
    smooth_params.stair_align_up_post_dist_m =
        node_->declare_parameter("planner.stair_align_up_post_dist_m", stair_legacy_window_m);
    smooth_params.stair_align_down_pre_dist_m =
        node_->declare_parameter("planner.stair_align_down_pre_dist_m", stair_legacy_window_m);
    smooth_params.stair_align_down_post_dist_m =
        node_->declare_parameter("planner.stair_align_down_post_dist_m", stair_legacy_window_m);
    astar_stair_up_pre_dist_m_ = smooth_params.stair_align_up_pre_dist_m;
    astar_stair_up_post_dist_m_ = smooth_params.stair_align_up_post_dist_m;
    astar_stair_down_pre_dist_m_ = smooth_params.stair_align_down_pre_dist_m;
    astar_stair_down_post_dist_m_ = smooth_params.stair_align_down_post_dist_m;
    smooth_params.stair_align_mode =
        node_->declare_parameter("planner.stair_segment_align_mode", std::string("curve_sample"));
    smooth_params.stair_sample_ds_m =
        node_->declare_parameter("planner.stair_sample_ds_m", 0.08);
    smooth_params.lambda_stair_anchor =
        node_->declare_parameter("planner.opt_lambda_stair_anchor", 0.0);
    smooth_params.lambda_stair_lateral =
        node_->declare_parameter("planner.opt_lambda_stair_lateral", 0.0);
    smooth_params.stair_corridor_half_width_m =
        node_->declare_parameter("planner.stair_corridor_half_width_m", 0.30);
    smooth_params.use_astar_stair_anchors = use_astar_stair_anchors_;
    smooth_params.astar_stair_anchor_match_max_dist_m = astar_stair_anchor_match_max_dist_m_;
    smooth_params.astar_anchor_near_obstacle_dist_m =
        node_->declare_parameter("planner.astar_anchor_near_obstacle_dist_m", 0.18);
    smooth_params.astar_anchor_tangent_probe_dist_m =
        node_->declare_parameter("planner.astar_anchor_tangent_probe_dist_m", 0.12);
    smooth_params.astar_anchor_tangent_shift_step_m =
        node_->declare_parameter("planner.astar_anchor_tangent_shift_step_m", 0.08);
    smooth_params.astar_anchor_tangent_shift_max_m =
        node_->declare_parameter("planner.astar_anchor_tangent_shift_max_m", 0.24);
    smooth_params.astar_anchor_tangent_improve_min_m =
        node_->declare_parameter("planner.astar_anchor_tangent_improve_min_m", 0.01);

    smoother_.setParams(smooth_params);
    
    // 创建调试可视化发布器
    astar_raw_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        "plan_astar_raw", 10);
    ctrl_pts_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "debug/control_points", 10);
    stair_debug_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "debug/stair_align", 10);
    plan_failure_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "debug/planner_failures", 10);
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

double SimplePlanner::computeStairShapeCost(int from_x, int from_y,
                                            int to_x, int to_y) {
    if (!astar_stair_shape_enable_ || !map_manager_ ||
        astar_stair_trigger_dist_m_ <= 1e-6 || resolution_ <= 1e-9) {
        return 0.0;
    }

    auto cellToWorld = [this](int mx, int my) -> Eigen::Vector2d {
        return Eigen::Vector2d(origin_x_ + (mx + 0.5) * resolution_,
                               origin_y_ + (my + 0.5) * resolution_);
    };

    const Eigen::Vector2d p_from = cellToWorld(from_x, from_y);
    const Eigen::Vector2d p_to = cellToWorld(to_x, to_y);
    Eigen::Vector2d move = p_to - p_from;
    double mv_norm = move.norm();
    if (mv_norm < 1e-6) {
        return 0.0;
    }
    Eigen::Vector2d move_dir = move / mv_norm;

    const int r = std::max(1, astar_stair_search_radius_cells_);
    bool found = false;
    double min_dist = std::numeric_limits<double>::max();
    double w_sum = 0.0;
    Eigen::Vector2d n_sum = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_sum = Eigen::Vector2d::Zero();

    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            int mx = to_x + dx;
            int my = to_y + dy;
            if (mx < 0 || mx >= width_ || my < 0 || my >= height_) {
                continue;
            }
            const Eigen::Vector2d p = cellToWorld(mx, my);
            double nx = 0.0, ny = 0.0;
            if (!map_manager_->getStairTraverseNormal(p.x(), p.y(), nx, ny)) {
                continue;
            }
            Eigen::Vector2d n(nx, ny);
            double nn = n.norm();
            if (nn < 1e-6) {
                continue;
            }
            n /= nn;
            double d = (p - p_to).norm();
            min_dist = std::min(min_dist, d);
            double w = 1.0 / (d * d + 1e-4);
            w_sum += w;
            n_sum += w * n;
            c_sum += w * p;
            found = true;
        }
    }

    if (!found || w_sum < 1e-9) {
        return 0.0;
    }

    Eigen::Vector2d n = n_sum;
    double nn = n.norm();
    if (nn < 1e-6) {
        return 0.0;
    }
    n /= nn;
    Eigen::Vector2d center = c_sum / w_sum;

    // 首次命中后在单次A*搜索中锁定同一个簇中心和法向
    if (astar_stair_lock_cluster_center_) {
        if (!astar_stair_cluster_locked_) {
            astar_stair_cluster_locked_ = true;
            astar_stair_locked_center_ = center;
            astar_stair_locked_normal_ = n;
        }
        center = astar_stair_locked_center_;
        n = astar_stair_locked_normal_;
    }

    Eigen::Vector2d t(-n.y(), n.x());

    // 与 B-spline 一致：按上下台阶方向选择 pre/post 弧长窗口
    const bool is_uphill = (move_dir.dot(n) >= 0.0);
    const double pre = std::max(0.0,
        is_uphill ? astar_stair_up_pre_dist_m_ : astar_stair_down_pre_dist_m_);
    const double post = std::max(0.0,
        is_uphill ? astar_stair_up_post_dist_m_ : astar_stair_down_post_dist_m_);
    const double dir = is_uphill ? 1.0 : -1.0;
    const double s_n = dir * (p_to - center).dot(n);

    // 外层兜底触发门限（避免远离台阶时误触发）；若设置偏小，自动放宽到窗口尺度
    const double min_required_trigger = std::max(pre, post);
    const double effective_trigger = std::max(astar_stair_trigger_dist_m_, min_required_trigger);
    if (min_dist > effective_trigger) {
        return 0.0;
    }

    // 核心窗口：长度直接来自 stair_align_*_pre/post，与 B-spline 一致
    if (s_n < -pre - 1e-6 || s_n > post + 1e-6) {
        return 0.0;
    }

    double tan_comp = move_dir.dot(t);
    double c_tan = astar_stair_tangent_penalty_weight_ * tan_comp * tan_comp;

    double d_t = (p_to - center).dot(t);
    double denom = std::max(0.05, (pre + post) * 0.5);
    double u = d_t / denom;
    double c_center = astar_stair_centerline_penalty_weight_ * u * u;

    return c_tan + c_center;
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
    clearPlanningFailureMarkers(goal.header.frame_id);
    
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
        publishPlanningFailureMarkers(goal.header.frame_id,
            {{start.pose.position.x, start.pose.position.y, "start(no_map)", 1.0f, 0.1f, 0.1f},
             {goal.pose.position.x, goal.pose.position.y, "goal(no_map)", 1.0f, 0.6f, 0.1f}});
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
        publishPlanningFailureMarkers(goal.header.frame_id,
            {{start.pose.position.x, start.pose.position.y, "start(out_of_map)", 1.0f, 0.1f, 0.1f},
             {goal.pose.position.x, goal.pose.position.y, "goal(out_of_map)", 1.0f, 0.6f, 0.1f}});
        return false;
    }
    
    // 起点终点有效性检查
    if (!isValid(sx, sy)) {
        RCLCPP_ERROR(node_->get_logger(), "起点(%d,%d)在障碍物中", sx, sy);
        publishPlanningFailureMarkers(goal.header.frame_id,
            {{start.pose.position.x, start.pose.position.y, "start(in_obstacle)", 1.0f, 0.1f, 0.1f}});
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
            publishPlanningFailureMarkers(goal.header.frame_id,
                {{goal.pose.position.x, goal.pose.position.y, "goal(in_obstacle)", 1.0f, 0.1f, 0.1f}});
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
        nav_msgs::msg::Path soft_seed_path;
        int fail_best_x = sx;
        int fail_best_y = sy;
        bool astar_success = false;
        if (stair_constraint_mode_ == "hard") {
            astar_success = runAstarWithHardStairConstraint(
                sx, sy, gx, gy, goal.header, raw_path, soft_seed_path,
                &fail_best_x, &fail_best_y);
            if (!astar_success && !soft_seed_path.poses.empty()) {
                raw_path = soft_seed_path;
                astar_success = true;
                RCLCPP_WARN(node_->get_logger(),
                    "硬约束路径失败，自动回退软约束路径: 点数=%zu",
                    raw_path.poses.size());
            }
        } else {
            astar_success = runAstar(
                sx, sy, gx, gy, goal.header, raw_path, &fail_best_x, &fail_best_y);
        }
        
        obstacle_threshold_ = original_threshold;
        
        if (!astar_success) {
            double fw = 0.0;
            double fy = 0.0;
            mapToWorld(fail_best_x, fail_best_y, fw, fy);
            publishPlanningFailureMarkers(goal.header.frame_id,
                {{goal.pose.position.x, goal.pose.position.y, "goal", 1.0f, 0.2f, 0.2f},
                 {fw, fy, "astar_frontier", 1.0f, 1.0f, 0.1f}});
            RCLCPP_WARN(node_->get_logger(), 
                "A*规划失败 (阈值=%d)", current_threshold);
            continue;
        }

        if (publish_astar_raw_path_ && astar_raw_path_pub_) {
            nav_msgs::msg::Path astar_path_msg = raw_path;
            astar_path_msg.header.stamp = node_->now();
            for (auto& pose : astar_path_msg.poses) {
                pose.header.stamp = astar_path_msg.header.stamp;
            }
            astar_raw_path_pub_->publish(astar_path_msg);
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
                publishStairDebugMarkers(smoother_.getStairDiagnostics(), goal.header.frame_id);
            }
        } else {
            smoothed_path = raw_path;
        }
        
        if (validatePath(smoothed_path)) {
            RCLCPP_DEBUG(node_->get_logger(), 
                "规划成功 (阈值=%d): A*=%zu点 -> 平滑=%zu点", 
                current_threshold, raw_path.poses.size(), smoothed_path.poses.size());
            clearPlanningFailureMarkers(goal.header.frame_id);
            path = smoothed_path;
            
            if (enable_path_cache_) {
                cached_path_ = path;
                cached_goal_ = goal;
            }
            return true;
        } else if (enable_smooth_ && allow_raw_fallback_on_smooth_fail_ && validatePath(raw_path)) {
            RCLCPP_WARN(node_->get_logger(),
                "平滑路径验证失败，回退到原始A*路径: A*=%zu点, 平滑=%zu点",
                raw_path.poses.size(), smoothed_path.poses.size());
            clearPlanningFailureMarkers(goal.header.frame_id);
            path = raw_path;

            if (enable_path_cache_) {
                cached_path_ = path;
                cached_goal_ = goal;
            }
            return true;
        } else {
            if (has_last_validate_fail_point_) {
                publishPlanningFailureMarkers(goal.header.frame_id,
                    {{last_validate_fail_x_, last_validate_fail_y_,
                      "invalid_path_pt", 1.0f, 0.3f, 0.1f}});
            } else if (!smoothed_path.poses.empty()) {
                const auto& p = smoothed_path.poses.front().pose.position;
                publishPlanningFailureMarkers(goal.header.frame_id,
                    {{p.x, p.y, "invalid_path_pt(fallback)", 1.0f, 0.3f, 0.1f}});
            }
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
                              nav_msgs::msg::Path& path,
                              int* fail_best_x,
                              int* fail_best_y) {
    // 每次A*搜索开始前重置簇锁定状态
    astar_stair_cluster_locked_ = false;
    astar_stair_locked_center_.setZero();
    astar_stair_locked_normal_.setZero();

    auto cmp = [](Node* a, Node* b) { return a->f() > b->f(); };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);
    std::vector<std::unique_ptr<Node>> all_nodes;
    std::vector<bool> closed(width_ * height_, false);
    std::vector<double> best_g(width_ * height_, std::numeric_limits<double>::infinity());
    std::vector<Node*> best_node(width_ * height_, nullptr);
    
    auto start_node = std::make_unique<Node>();
    start_node->x = sx; start_node->y = sy;
    start_node->g = 0; start_node->h = heuristic(sx, sy, gx, gy);
    int start_idx = sy * width_ + sx;
    best_g[start_idx] = 0.0;
    best_node[start_idx] = start_node.get();
    open.push(start_node.get());
    all_nodes.push_back(std::move(start_node));
    
    const int dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};
    const double move_cost[] = {1, 1.414, 1, 1.414, 1, 1.414, 1, 1.414};
    
    Node* goal_node = nullptr;
    Node* closest_node = nullptr;
    double closest_h = std::numeric_limits<double>::infinity();
    int iterations = 0;
    const int auto_max_iter = std::min(width_ * height_ * 2, 1000000);
    const int max_iter = (astar_max_iterations_ > 0) ? astar_max_iterations_ : auto_max_iter;
    
    while (!open.empty() && iterations++ < max_iter) {
        Node* curr = open.top();
        open.pop();

        if (curr->h < closest_h) {
            closest_h = curr->h;
            closest_node = curr;
        }
        
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

            // 对角扩展时，要求两个正交相邻格都可通行，防止穿角
            if (dx[i] != 0 && dy[i] != 0) {
                const int ortho_x = curr->x + dx[i];
                const int ortho_y = curr->y + dy[i];
                if (!isValid(ortho_x, curr->y) || !isValid(curr->x, ortho_y)) {
                    continue;
                }
            }

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
            double stair_shape_cost = computeStairShapeCost(curr->x, curr->y, nx, ny);
            double total_cost = move_cost[i] + cost_weight_ * cell_cost + stair_shape_cost;
            double tentative_g = curr->g + total_cost;

            if (tentative_g + 1e-9 >= best_g[nidx]) {
                continue;
            }
            
            auto new_node = std::make_unique<Node>();
            new_node->x = nx; new_node->y = ny;
            new_node->g = tentative_g;
            new_node->h = heuristic(nx, ny, gx, gy);
            new_node->parent = curr;
            best_g[nidx] = tentative_g;
            best_node[nidx] = new_node.get();
            
            open.push(new_node.get());
            all_nodes.push_back(std::move(new_node));
        }
    }
    
    if (!goal_node) {
        if (closest_node) {
            if (fail_best_x) {
                *fail_best_x = closest_node->x;
            }
            if (fail_best_y) {
                *fail_best_y = closest_node->y;
            }
        }
        return false;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    for (Node* n = best_node[gy * width_ + gx]; n != nullptr; n = n->parent) {
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

bool SimplePlanner::runAstarWithHardStairConstraint(
    int sx, int sy, int gx, int gy,
    const std_msgs::msg::Header& header,
    nav_msgs::msg::Path& constrained_path,
    nav_msgs::msg::Path& soft_seed_path,
    int* fail_best_x,
    int* fail_best_y) {
    constrained_path = nav_msgs::msg::Path();
    soft_seed_path = nav_msgs::msg::Path();

    // 先求软约束路径：用于提取台阶中心点，同时作为硬约束失败的回退种子路径
    if (!runAstar(sx, sy, gx, gy, header, soft_seed_path, fail_best_x, fail_best_y)) {
        return false;
    }

    // 无地图管理器或路径过短时，不施加硬约束
    if (!map_manager_ || soft_seed_path.poses.size() < 3) {
        constrained_path = soft_seed_path;
        return true;
    }

    int stair_start = -1;
    int stair_end = -1;
    Eigen::Vector2d normal_sum = Eigen::Vector2d::Zero();
    Eigen::Vector2d pos_sum = Eigen::Vector2d::Zero();
    int stair_cnt = 0;
    bool in_cluster = false;

    for (int i = 0; i < static_cast<int>(soft_seed_path.poses.size()); ++i) {
        const auto& pose = soft_seed_path.poses[i].pose.position;
        double nx = 0.0, ny = 0.0;
        const bool is_stair = map_manager_->getStairTraverseNormal(pose.x, pose.y, nx, ny);
        if (is_stair) {
            if (!in_cluster) {
                in_cluster = true;
                stair_start = i;
            }
            stair_end = i;
            Eigen::Vector2d n(nx, ny);
            const double nn = n.norm();
            if (nn > 1e-6) {
                normal_sum += n / nn;
            }
            pos_sum += Eigen::Vector2d(pose.x, pose.y);
            stair_cnt++;
        } else if (in_cluster) {
            break;
        }
    }

    // 没有台阶命中，退化为软约束路径
    if (stair_cnt <= 0) {
        constrained_path = soft_seed_path;
        return true;
    }

    Eigen::Vector2d normal = normal_sum;
    const double nn = normal.norm();
    if (nn < 1e-6) {
        constrained_path = soft_seed_path;
        return true;
    }
    normal /= nn;
    const Eigen::Vector2d center = pos_sum / static_cast<double>(stair_cnt);

    const int mid_idx = std::clamp((stair_start + stair_end) / 2,
                                   1,
                                   static_cast<int>(soft_seed_path.poses.size()) - 2);
    const auto& p_prev = soft_seed_path.poses[mid_idx - 1].pose.position;
    const auto& p_next = soft_seed_path.poses[mid_idx + 1].pose.position;
    Eigen::Vector2d tangent(p_next.x - p_prev.x, p_next.y - p_prev.y);
    if (tangent.norm() < 1e-6 && stair_end > stair_start) {
        const auto& p_s = soft_seed_path.poses[stair_start].pose.position;
        const auto& p_e = soft_seed_path.poses[stair_end].pose.position;
        tangent = Eigen::Vector2d(p_e.x - p_s.x, p_e.y - p_s.y);
    }
    if (tangent.norm() < 1e-6) {
        constrained_path = soft_seed_path;
        return true;
    }
    tangent.normalize();

    const bool is_uphill = (tangent.dot(normal) >= 0.0);
    const double base_pre = is_uphill ? astar_stair_up_pre_dist_m_ : astar_stair_down_pre_dist_m_;
    const double base_post = is_uphill ? astar_stair_up_post_dist_m_ : astar_stair_down_post_dist_m_;
    const double pre_dist = std::max(0.0, base_pre + stair_hard_dist_delta_m_);
    const double post_dist = std::max(0.0, base_post + stair_hard_dist_delta_m_);

    if (pre_dist <= 1e-6 && post_dist <= 1e-6) {
        constrained_path = soft_seed_path;
        return true;
    }

    const double dir = is_uphill ? 1.0 : -1.0;
    const Eigen::Vector2d pre_pt = center - dir * normal * pre_dist;
    const Eigen::Vector2d post_pt = center + dir * normal * post_dist;

    int pre_x = 0, pre_y = 0, post_x = 0, post_y = 0;
    if (!worldToMap(pre_pt.x(), pre_pt.y(), pre_x, pre_y) ||
        !worldToMap(post_pt.x(), post_pt.y(), post_x, post_y)) {
        if (fail_best_x) *fail_best_x = gx;
        if (fail_best_y) *fail_best_y = gy;
        return false;
    }

    if (!isValid(pre_x, pre_y) && !findNearestFreeCell(pre_x, pre_y, pre_x, pre_y, 10)) {
        if (fail_best_x) *fail_best_x = pre_x;
        if (fail_best_y) *fail_best_y = pre_y;
        return false;
    }
    if (!isValid(post_x, post_y) && !findNearestFreeCell(post_x, post_y, post_x, post_y, 10)) {
        if (fail_best_x) *fail_best_x = post_x;
        if (fail_best_y) *fail_best_y = post_y;
        return false;
    }

    const std::vector<std::pair<int, int>> segment_pts = {
        {sx, sy}, {pre_x, pre_y}, {post_x, post_y}, {gx, gy}};

    nav_msgs::msg::Path stitched;
    stitched.header = header;

    for (size_t i = 0; i + 1 < segment_pts.size(); ++i) {
        const int from_x = segment_pts[i].first;
        const int from_y = segment_pts[i].second;
        const int to_x = segment_pts[i + 1].first;
        const int to_y = segment_pts[i + 1].second;

        if (from_x == to_x && from_y == to_y) {
            continue;
        }

        nav_msgs::msg::Path seg;
        int seg_fail_x = from_x;
        int seg_fail_y = from_y;
        if (!runAstar(from_x, from_y, to_x, to_y, header, seg, &seg_fail_x, &seg_fail_y)) {
            if (fail_best_x) *fail_best_x = seg_fail_x;
            if (fail_best_y) *fail_best_y = seg_fail_y;
            return false;
        }

        if (seg.poses.empty()) {
            continue;
        }

        if (stitched.poses.empty()) {
            stitched.poses = seg.poses;
        } else {
            stitched.poses.insert(stitched.poses.end(), seg.poses.begin() + 1, seg.poses.end());
        }
    }

    if (stitched.poses.empty()) {
        constrained_path = soft_seed_path;
        return true;
    }

    constrained_path = stitched;
    RCLCPP_INFO(node_->get_logger(),
        "硬约束路径生成: stair=%s, pre=%.2fm, post=%.2fm, delta=%.2fm, soft=%zu, hard=%zu",
        is_uphill ? "UP" : "DOWN",
        pre_dist, post_dist, stair_hard_dist_delta_m_,
        soft_seed_path.poses.size(), constrained_path.poses.size());
    return true;
}

void SimplePlanner::clearPlanningFailureMarkers(const std::string& frame_id) {
    if (!publish_failure_debug_markers_ || !plan_failure_pub_ || frame_id.empty()) {
        return;
    }
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker del;
    del.header.frame_id = frame_id;
    del.header.stamp = node_->now();
    del.ns = "planner_failures";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(del);
    plan_failure_pub_->publish(markers);
}

void SimplePlanner::publishPlanningFailureMarkers(const std::string& frame_id,
                                                  const std::vector<FailurePoint>& points) {
    if (!publish_failure_debug_markers_ || !plan_failure_pub_ || frame_id.empty()) {
        return;
    }

    visualization_msgs::msg::MarkerArray markers;
    auto stamp = node_->now();

    visualization_msgs::msg::Marker del;
    del.header.frame_id = frame_id;
    del.header.stamp = stamp;
    del.ns = "planner_failures";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(del);

    int id = 0;
    for (const auto& fp : points) {
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = frame_id;
        sphere.header.stamp = stamp;
        sphere.ns = "planner_failures";
        sphere.id = id++;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = fp.x;
        sphere.pose.position.y = fp.y;
        sphere.pose.position.z = 0.10;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = failure_marker_scale_;
        sphere.scale.y = failure_marker_scale_;
        sphere.scale.z = failure_marker_scale_;
        sphere.color.r = fp.r;
        sphere.color.g = fp.g;
        sphere.color.b = fp.b;
        sphere.color.a = 0.95f;
        markers.markers.push_back(sphere);

        visualization_msgs::msg::Marker text;
        text.header.frame_id = frame_id;
        text.header.stamp = stamp;
        text.ns = "planner_failures";
        text.id = id++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = fp.x;
        text.pose.position.y = fp.y;
        text.pose.position.z = 0.26;
        text.pose.orientation.w = 1.0;
        text.scale.z = std::max(0.08, failure_marker_scale_ * 0.75);
        text.color.r = 1.0f;
        text.color.g = 1.0f;
        text.color.b = 1.0f;
        text.color.a = 0.95f;
        text.text = fp.label;
        markers.markers.push_back(text);
    }

    plan_failure_pub_->publish(markers);
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
    has_last_validate_fail_point_ = false;

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

    auto worldToCostmapCell = [&](double wx, double wy, int& mx, int& my) -> bool {
        mx = static_cast<int>((wx - cm_origin_x) / cm_resolution);
        my = static_cast<int>((wy - cm_origin_y) / cm_resolution);
        return mx >= 0 && mx < cm_width && my >= 0 && my < cm_height;
    };
    
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
            has_last_validate_fail_point_ = true;
            last_validate_fail_x_ = pose.pose.position.x;
            last_validate_fail_y_ = pose.pose.position.y;
            RCLCPP_WARN(node_->get_logger(), 
                "路径点 (%.2f, %.2f) 与障碍物重合: costmap=%d >= %d", 
                pose.pose.position.x, pose.pose.position.y, 
                static_cast<int>(val), static_cast<int>(obstacle_check_threshold_));
            return false;
        }
    }

    if (map_manager_ && path.poses.size() >= 2) {
        for (size_t i = 1; i < path.poses.size(); ++i) {
            int from_x = 0;
            int from_y = 0;
            int to_x = 0;
            int to_y = 0;
            const bool from_ok = worldToCostmapCell(path.poses[i - 1].pose.position.x,
                                                    path.poses[i - 1].pose.position.y,
                                                    from_x,
                                                    from_y);
            const bool to_ok = worldToCostmapCell(path.poses[i].pose.position.x,
                                                  path.poses[i].pose.position.y,
                                                  to_x,
                                                  to_y);
            if (!from_ok || !to_ok) {
                continue;
            }

            if (!map_manager_->isTransitionAllowed(from_x, from_y, to_x, to_y)) {
                has_last_validate_fail_point_ = true;
                last_validate_fail_x_ = path.poses[i].pose.position.x;
                last_validate_fail_y_ = path.poses[i].pose.position.y;
                RCLCPP_WARN(node_->get_logger(),
                            "路径方向约束冲突: idx=%zu, from=(%d,%d) -> to=(%d,%d) 被禁行",
                            i - 1,
                            from_x,
                            from_y,
                            to_x,
                            to_y);
                return false;
            }
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
    
    return dist > goal_change_tolerance_;
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

void SimplePlanner::publishStairDebugMarkers(const StairAlignDiagnostics& diag,
                                              const std::string& frame_id) {
    if (!stair_debug_pub_) return;
    
    visualization_msgs::msg::MarkerArray markers;
    auto stamp = node_->now();
    int id = 0;
    std::vector<LayeredMapManager::StairPrimitive> primitives;
    if (map_manager_) {
        primitives = map_manager_->getStairPrimitives();
    }

    // 清除旧标记
    {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = frame_id;
        del.header.stamp = stamp;
        del.ns = "stair_normals";
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(del);
        del.ns = "stair_samples";
        markers.markers.push_back(del);
        del.ns = "stair_window";
        markers.markers.push_back(del);
        del.ns = "stair_anchors";
        markers.markers.push_back(del);
        del.ns = "stair_corridor";
        markers.markers.push_back(del);
        del.ns = "stair_astar_hits";
        markers.markers.push_back(del);
        del.ns = "stair_anchor_match";
        markers.markers.push_back(del);
        del.ns = "stair_primitive_center";
        markers.markers.push_back(del);
        del.ns = "stair_primitive_normal";
        markers.markers.push_back(del);
        del.ns = "stair_primitive_tangent";
        markers.markers.push_back(del);
        del.ns = "stair_primitive_half_length";
        markers.markers.push_back(del);
        del.ns = "stair_primitive_band";
        markers.markers.push_back(del);
        del.ns = "stair_primitive_id";
        markers.markers.push_back(del);
    }

    if (diag.clusters.empty() && diag.samples.empty() && primitives.empty()) {
        stair_debug_pub_->publish(markers);
        return;
    }

    // StairPrimitive 对象化可视化（center/normal/tangent/half_length/crossing_band）
    if (!primitives.empty()) {
        visualization_msgs::msg::Marker centers;
        centers.header.frame_id = frame_id;
        centers.header.stamp = stamp;
        centers.ns = "stair_primitive_center";
        centers.id = id++;
        centers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        centers.action = visualization_msgs::msg::Marker::ADD;
        centers.scale.x = 0.10;
        centers.scale.y = 0.10;
        centers.scale.z = 0.10;
        centers.color.r = 0.0f;
        centers.color.g = 0.7f;
        centers.color.b = 1.0f;
        centers.color.a = 0.95f;

        for (const auto& p : primitives) {
            geometry_msgs::msg::Point c;
            c.x = p.center.x();
            c.y = p.center.y();
            c.z = 0.18;
            centers.points.push_back(c);

            visualization_msgs::msg::Marker n_arrow;
            n_arrow.header.frame_id = frame_id;
            n_arrow.header.stamp = stamp;
            n_arrow.ns = "stair_primitive_normal";
            n_arrow.id = id++;
            n_arrow.type = visualization_msgs::msg::Marker::ARROW;
            n_arrow.action = visualization_msgs::msg::Marker::ADD;
            n_arrow.scale.x = 0.03;
            n_arrow.scale.y = 0.06;
            n_arrow.scale.z = 0.0;
            n_arrow.color.r = 0.0f;
            n_arrow.color.g = 1.0f;
            n_arrow.color.b = 1.0f;
            n_arrow.color.a = 0.9f;

            geometry_msgs::msg::Point ns, ne;
            ns.x = p.center.x();
            ns.y = p.center.y();
            ns.z = 0.16;
            ne.x = p.center.x() + p.normal.x() * 0.45;
            ne.y = p.center.y() + p.normal.y() * 0.45;
            ne.z = 0.16;
            n_arrow.points.push_back(ns);
            n_arrow.points.push_back(ne);
            markers.markers.push_back(n_arrow);

            visualization_msgs::msg::Marker t_arrow = n_arrow;
            t_arrow.ns = "stair_primitive_tangent";
            t_arrow.id = id++;
            t_arrow.color.r = 0.9f;
            t_arrow.color.g = 0.1f;
            t_arrow.color.b = 0.9f;
            t_arrow.points[1].x = p.center.x() + p.tangent.x() * 0.45;
            t_arrow.points[1].y = p.center.y() + p.tangent.y() * 0.45;
            markers.markers.push_back(t_arrow);

            visualization_msgs::msg::Marker hl;
            hl.header.frame_id = frame_id;
            hl.header.stamp = stamp;
            hl.ns = "stair_primitive_half_length";
            hl.id = id++;
            hl.type = visualization_msgs::msg::Marker::LINE_STRIP;
            hl.action = visualization_msgs::msg::Marker::ADD;
            hl.scale.x = 0.03;
            hl.color.r = 0.3f;
            hl.color.g = 0.9f;
            hl.color.b = 0.2f;
            hl.color.a = 0.85f;
            hl.pose.orientation.w = 1.0;

            geometry_msgs::msg::Point l1, l2;
            l1.x = p.center.x() - p.tangent.x() * p.half_length;
            l1.y = p.center.y() - p.tangent.y() * p.half_length;
            l1.z = 0.14;
            l2.x = p.center.x() + p.tangent.x() * p.half_length;
            l2.y = p.center.y() + p.tangent.y() * p.half_length;
            l2.z = 0.14;
            hl.points.push_back(l1);
            hl.points.push_back(l2);
            markers.markers.push_back(hl);

            const double low = std::max(0.0, p.crossing_band.low_side_dist_m);
            const double high = std::max(0.0, p.crossing_band.high_side_dist_m);
            const double tw = std::max(0.05, p.crossing_band.tangent_half_width_m);
            Eigen::Vector2d c0 = p.center;
            Eigen::Vector2d a = c0 - p.normal * low - p.tangent * tw;
            Eigen::Vector2d b = c0 - p.normal * low + p.tangent * tw;
            Eigen::Vector2d cpt = c0 + p.normal * high + p.tangent * tw;
            Eigen::Vector2d d = c0 + p.normal * high - p.tangent * tw;

            visualization_msgs::msg::Marker band;
            band.header.frame_id = frame_id;
            band.header.stamp = stamp;
            band.ns = "stair_primitive_band";
            band.id = id++;
            band.type = visualization_msgs::msg::Marker::LINE_STRIP;
            band.action = visualization_msgs::msg::Marker::ADD;
            band.scale.x = 0.02;
            band.color.r = 1.0f;
            band.color.g = 0.75f;
            band.color.b = 0.0f;
            band.color.a = 0.75f;
            band.pose.orientation.w = 1.0;

            auto push_pt = [&](const Eigen::Vector2d& v) {
                geometry_msgs::msg::Point q;
                q.x = v.x();
                q.y = v.y();
                q.z = 0.06;
                band.points.push_back(q);
            };
            push_pt(a);
            push_pt(b);
            push_pt(cpt);
            push_pt(d);
            push_pt(a);
            markers.markers.push_back(band);

            visualization_msgs::msg::Marker txt;
            txt.header.frame_id = frame_id;
            txt.header.stamp = stamp;
            txt.ns = "stair_primitive_id";
            txt.id = id++;
            txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::msg::Marker::ADD;
            txt.pose.position.x = p.center.x();
            txt.pose.position.y = p.center.y();
            txt.pose.position.z = 0.30;
            txt.scale.z = 0.11;
            txt.color.r = 1.0f;
            txt.color.g = 1.0f;
            txt.color.b = 1.0f;
            txt.color.a = 0.95f;
            txt.text = std::string("id=") + std::to_string(p.stair_id);
            markers.markers.push_back(txt);
        }
        markers.markers.push_back(centers);
    }

    // A* 台阶命中点（青绿色）
    if (!diag.astar_hit_points.empty()) {
        visualization_msgs::msg::Marker hit_pts;
        hit_pts.header.frame_id = frame_id;
        hit_pts.header.stamp = stamp;
        hit_pts.ns = "stair_astar_hits";
        hit_pts.id = id++;
        hit_pts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        hit_pts.action = visualization_msgs::msg::Marker::ADD;
        hit_pts.scale.x = 0.05;
        hit_pts.scale.y = 0.05;
        hit_pts.scale.z = 0.05;
        hit_pts.color.r = 0.1f;
        hit_pts.color.g = 1.0f;
        hit_pts.color.b = 0.6f;
        hit_pts.color.a = 0.95f;
        for (const auto& hp : diag.astar_hit_points) {
            geometry_msgs::msg::Point p;
            p.x = hp.x();
            p.y = hp.y();
            p.z = 0.11;
            hit_pts.points.push_back(p);
        }
        markers.markers.push_back(hit_pts);
    }

    // 台阶法向箭头（每个 cluster 一个）
    for (size_t i = 0; i < diag.clusters.size(); ++i) {
        const auto& cl = diag.clusters[i];
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = frame_id;
        arrow.header.stamp = stamp;
        arrow.ns = "stair_normals";
        arrow.id = id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.scale.x = 0.04;
        arrow.scale.y = 0.08;
        arrow.scale.z = 0.0;
        arrow.color.r = 0.0f;
        arrow.color.g = 1.0f;
        arrow.color.b = 1.0f;
        arrow.color.a = 1.0f;

        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = cl.center_pos.x();
        p_start.y = cl.center_pos.y();
        p_start.z = 0.12;
        p_end.x = cl.center_pos.x() + cl.normal.x() * 0.5;
        p_end.y = cl.center_pos.y() + cl.normal.y() * 0.5;
        p_end.z = 0.12;
        arrow.points.push_back(p_start);
        arrow.points.push_back(p_end);
        markers.markers.push_back(arrow);

        // 台阶切向箭头（紫色）
        visualization_msgs::msg::Marker t_arrow = arrow;
        t_arrow.id = id++;
        t_arrow.color.r = 0.8f;
        t_arrow.color.g = 0.0f;
        t_arrow.color.b = 0.8f;
        Eigen::Vector2d tangent_dir(-cl.normal.y(), cl.normal.x());
        t_arrow.points[1].x = cl.center_pos.x() + tangent_dir.x() * 0.4;
        t_arrow.points[1].y = cl.center_pos.y() + tangent_dir.y() * 0.4;
        markers.markers.push_back(t_arrow);

        // UP/DOWN 文字标记
        visualization_msgs::msg::Marker text;
        text.header.frame_id = frame_id;
        text.header.stamp = stamp;
        text.ns = "stair_normals";
        text.id = id++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = cl.center_pos.x();
        text.pose.position.y = cl.center_pos.y();
        text.pose.position.z = 0.25;
        text.scale.z = 0.12;
        text.color.r = 1.0f;
        text.color.g = 1.0f;
        text.color.b = 1.0f;
        text.color.a = 1.0f;
        text.text = cl.is_uphill ? "UP" : "DOWN";
        markers.markers.push_back(text);
    }

    // 采样点球体（按航向误差着色：绿→黄→红）
    if (!diag.samples.empty()) {
        visualization_msgs::msg::Marker pts;
        pts.header.frame_id = frame_id;
        pts.header.stamp = stamp;
        pts.ns = "stair_samples";
        pts.id = id++;
        pts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        pts.action = visualization_msgs::msg::Marker::ADD;
        pts.scale.x = 0.04;
        pts.scale.y = 0.04;
        pts.scale.z = 0.04;

        for (const auto& s : diag.samples) {
            geometry_msgs::msg::Point p;
            p.x = s.position.x();
            p.y = s.position.y();
            p.z = 0.08;
            pts.points.push_back(p);

            // 绿(0°) → 黄(8°) → 红(20°+)
            std_msgs::msg::ColorRGBA c;
            double ratio = std::min(1.0, s.heading_err_deg / 20.0);
            c.r = static_cast<float>(std::min(1.0, ratio * 2.0));
            c.g = static_cast<float>(std::min(1.0, (1.0 - ratio) * 2.0));
            c.b = 0.0f;
            c.a = 1.0f;
            pts.colors.push_back(c);
        }
        markers.markers.push_back(pts);

        // 每个采样点的切向小箭头
        for (size_t si = 0; si < diag.samples.size(); si += 3) {
            const auto& s = diag.samples[si];
            visualization_msgs::msg::Marker ta;
            ta.header.frame_id = frame_id;
            ta.header.stamp = stamp;
            ta.ns = "stair_samples";
            ta.id = id++;
            ta.type = visualization_msgs::msg::Marker::ARROW;
            ta.action = visualization_msgs::msg::Marker::ADD;
            ta.scale.x = 0.02;
            ta.scale.y = 0.04;
            ta.scale.z = 0.0;

            double ratio = std::min(1.0, s.heading_err_deg / 20.0);
            ta.color.r = static_cast<float>(std::min(1.0, ratio * 2.0));
            ta.color.g = static_cast<float>(std::min(1.0, (1.0 - ratio) * 2.0));
            ta.color.b = 0.0f;
            ta.color.a = 0.7f;

            geometry_msgs::msg::Point ps, pe;
            ps.x = s.position.x();
            ps.y = s.position.y();
            ps.z = 0.08;
            pe.x = s.position.x() + s.tangent_unit.x() * 0.15;
            pe.y = s.position.y() + s.tangent_unit.y() * 0.15;
            pe.z = 0.08;
            ta.points.push_back(ps);
            ta.points.push_back(pe);
            markers.markers.push_back(ta);
        }
    }

    // 锚点球体（P_pre=红, P_mid=白, P_post=蓝）
    for (size_t i = 0; i < diag.clusters.size(); ++i) {
        const auto& cl = diag.clusters[i];
        struct AnchorVis {
            Eigen::Vector2d pos;
            float r, g, b;
        };
        AnchorVis anchors[3] = {
            {cl.anchor_pre,  1.0f, 0.2f, 0.2f},
            {cl.anchor_mid,  1.0f, 1.0f, 1.0f},
            {cl.anchor_post, 0.2f, 0.4f, 1.0f}
        };
        for (int ai = 0; ai < 3; ++ai) {
            visualization_msgs::msg::Marker sp;
            sp.header.frame_id = frame_id;
            sp.header.stamp = stamp;
            sp.ns = "stair_anchors";
            sp.id = id++;
            sp.type = visualization_msgs::msg::Marker::SPHERE;
            sp.action = visualization_msgs::msg::Marker::ADD;
            sp.pose.position.x = anchors[ai].pos.x();
            sp.pose.position.y = anchors[ai].pos.y();
            sp.pose.position.z = 0.15;
            sp.pose.orientation.w = 1.0;
            sp.scale.x = 0.08;
            sp.scale.y = 0.08;
            sp.scale.z = 0.08;
            sp.color.r = anchors[ai].r;
            sp.color.g = anchors[ai].g;
            sp.color.b = anchors[ai].b;
            sp.color.a = 0.9f;
            markers.markers.push_back(sp);
        }

        // anchor_pre -> A*命中点连线（若匹配成功）
        if (cl.has_astar_match) {
            visualization_msgs::msg::Marker match_line;
            match_line.header.frame_id = frame_id;
            match_line.header.stamp = stamp;
            match_line.ns = "stair_anchor_match";
            match_line.id = id++;
            match_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            match_line.action = visualization_msgs::msg::Marker::ADD;
            match_line.scale.x = 0.02;
            match_line.color.r = 1.0f;
            match_line.color.g = 0.0f;
            match_line.color.b = 1.0f;
            match_line.color.a = 0.9f;
            match_line.pose.orientation.w = 1.0;

            geometry_msgs::msg::Point p1, p2;
            p1.x = cl.anchor_pre.x();
            p1.y = cl.anchor_pre.y();
            p1.z = 0.13;
            p2.x = cl.astar_match_point.x();
            p2.y = cl.astar_match_point.y();
            p2.z = 0.13;
            match_line.points.push_back(p1);
            match_line.points.push_back(p2);
            markers.markers.push_back(match_line);

            visualization_msgs::msg::Marker txt;
            txt.header.frame_id = frame_id;
            txt.header.stamp = stamp;
            txt.ns = "stair_anchor_match";
            txt.id = id++;
            txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::msg::Marker::ADD;
            txt.pose.position.x = 0.5 * (p1.x + p2.x);
            txt.pose.position.y = 0.5 * (p1.y + p2.y);
            txt.pose.position.z = 0.22;
            txt.scale.z = 0.10;
            txt.color.r = 1.0f;
            txt.color.g = 0.9f;
            txt.color.b = 1.0f;
            txt.color.a = 0.95f;
            char buf[32];
            std::snprintf(buf, sizeof(buf), "d=%.2fm", cl.astar_match_dist);
            txt.text = buf;
            markers.markers.push_back(txt);
        }

        // 走廊边界线（两条沿法向偏移 corridor_half_width 的线段）
        Eigen::Vector2d t_stair(-cl.normal.y(), cl.normal.x());
        double hw = smoother_.getParams().stair_corridor_half_width_m;
        for (int side = -1; side <= 1; side += 2) {
            Eigen::Vector2d offset = static_cast<double>(side) * hw * t_stair;
            visualization_msgs::msg::Marker line;
            line.header.frame_id = frame_id;
            line.header.stamp = stamp;
            line.ns = "stair_corridor";
            line.id = id++;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.scale.x = 0.02;
            line.color.r = 1.0f;
            line.color.g = 0.6f;
            line.color.b = 0.0f;
            line.color.a = 0.6f;
            line.pose.orientation.w = 1.0;

            geometry_msgs::msg::Point p1, p2;
            Eigen::Vector2d start = cl.anchor_pre + offset;
            Eigen::Vector2d end = cl.anchor_post + offset;
            p1.x = start.x(); p1.y = start.y(); p1.z = 0.05;
            p2.x = end.x();   p2.y = end.y();   p2.z = 0.05;
            line.points.push_back(p1);
            line.points.push_back(p2);
            markers.markers.push_back(line);
        }
    }

    stair_debug_pub_->publish(markers);
}

}  // namespace nav_components
