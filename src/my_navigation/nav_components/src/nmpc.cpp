// NMPC controller implementation
// 集成 acados 生成的 solver 到 ROS2 导航框架

#include "nav_components/nmpc.hpp"
#include "nav_components/layered_map_manager.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 提供 tf2::fromMsg
#include <tf2/utils.h>
#include <cmath>
#include <fstream>
#include <chrono>

// acados solver C 接口
extern "C" {
    #include "acados_solver_wheelleg_nmpc.h"
}

namespace nav_components {

NMPC::NMPC() {
    last_state_.resize(5, 0.0);
    last_control_.resize(2, 0.0);
}

NMPC::~NMPC() {
    if (acados_ocp_capsule_) {
        wheelleg_nmpc_acados_free(acados_ocp_capsule_);
    }
}

void NMPC::initialize(rclcpp::Node* node) {
    node_ = node;
    
    // ========== 声明所有 ROS 参数 ==========
    // 运动约束
    node_->declare_parameter("nmpc.max_linear_vel", params_.max_linear_vel);
    node_->declare_parameter("nmpc.max_angular_vel", params_.max_angular_vel);
    node_->declare_parameter("nmpc.max_linear_accel", params_.max_linear_accel);
    node_->declare_parameter("nmpc.max_angular_accel", params_.max_angular_accel);
    node_->declare_parameter("nmpc.allow_reverse", params_.allow_reverse);
    
    // 求解器参数 (必须与 export_ocp.py 一致!)
    node_->declare_parameter("nmpc.N_horizon", N_horizon_);
    node_->declare_parameter("nmpc.T_horizon", T_horizon_);
    
    // 局部参考提取
    node_->declare_parameter("nmpc.horizon_length", params_.horizon_length);
    node_->declare_parameter("nmpc.desired_velocity", params_.desired_velocity);
    node_->declare_parameter("nmpc.lateral_error_threshold", params_.lateral_error_threshold);
    
    // 代价权重
    node_->declare_parameter("nmpc.Q_position", params_.Q_position);
    node_->declare_parameter("nmpc.Q_orientation", params_.Q_orientation);
    node_->declare_parameter("nmpc.Q_velocity", params_.Q_velocity);
    node_->declare_parameter("nmpc.R_linear", params_.R_linear);
    node_->declare_parameter("nmpc.R_angular", params_.R_angular);
    node_->declare_parameter("nmpc.terminal_multiplier", params_.terminal_multiplier);
    
    // ESDF 障碍物代价参数
    node_->declare_parameter("nmpc.esdf_weight", params_.esdf_weight);
    node_->declare_parameter("nmpc.esdf_safe_dist", params_.esdf_safe_dist);
    node_->declare_parameter("nmpc.enable_esdf_cost", params_.enable_esdf_cost);
    node_->declare_parameter("nmpc.near_weight_multiplier", params_.near_weight_multiplier);
    
    // 容差参数
    node_->declare_parameter("nmpc.xy_tolerance", 0.1);
    node_->declare_parameter("nmpc.yaw_tolerance", 0.1);
    
    // 可视化选项
    node_->declare_parameter("nmpc.publish_predicted_path", true);
    node_->declare_parameter("nmpc.publish_debug_markers", false);
    
    // ========== 读取参数 ==========
    params_.max_linear_vel = node_->get_parameter("nmpc.max_linear_vel").as_double();
    params_.max_angular_vel = node_->get_parameter("nmpc.max_angular_vel").as_double();
    params_.max_linear_accel = node_->get_parameter("nmpc.max_linear_accel").as_double();
    params_.max_angular_accel = node_->get_parameter("nmpc.max_angular_accel").as_double();
    params_.allow_reverse = node_->get_parameter("nmpc.allow_reverse").as_bool();
    
    N_horizon_ = node_->get_parameter("nmpc.N_horizon").as_int();
    T_horizon_ = node_->get_parameter("nmpc.T_horizon").as_double();
    
    params_.horizon_length = node_->get_parameter("nmpc.horizon_length").as_double();
    params_.desired_velocity = node_->get_parameter("nmpc.desired_velocity").as_double();
    params_.lateral_error_threshold = node_->get_parameter("nmpc.lateral_error_threshold").as_double();
    
    params_.Q_position = node_->get_parameter("nmpc.Q_position").as_double();
    params_.Q_orientation = node_->get_parameter("nmpc.Q_orientation").as_double();
    params_.Q_velocity = node_->get_parameter("nmpc.Q_velocity").as_double();
    params_.R_linear = node_->get_parameter("nmpc.R_linear").as_double();
    params_.R_angular = node_->get_parameter("nmpc.R_angular").as_double();
    params_.terminal_multiplier = node_->get_parameter("nmpc.terminal_multiplier").as_double();
    
    params_.esdf_weight = node_->get_parameter("nmpc.esdf_weight").as_double();
    params_.esdf_safe_dist = node_->get_parameter("nmpc.esdf_safe_dist").as_double();
    params_.enable_esdf_cost = node_->get_parameter("nmpc.enable_esdf_cost").as_bool();
    params_.near_weight_multiplier = node_->get_parameter("nmpc.near_weight_multiplier").as_double();
    
    xy_tolerance_ = node_->get_parameter("nmpc.xy_tolerance").as_double();
    yaw_tolerance_ = node_->get_parameter("nmpc.yaw_tolerance").as_double();
    
    bool publish_path = node_->get_parameter("nmpc.publish_predicted_path").as_bool();
    bool publish_markers = node_->get_parameter("nmpc.publish_debug_markers").as_bool();
    
    // ========== 创建 acados solver ==========
    acados_ocp_capsule_ = wheelleg_nmpc_acados_create_capsule();
    int status = wheelleg_nmpc_acados_create(acados_ocp_capsule_);
    
    if (status != 0) {
        RCLCPP_ERROR(node_->get_logger(), "❌ NMPC: acados solver 创建失败");
        throw std::runtime_error("acados solver creation failed");
    }
    
    // 更新运行时参数到 solver
    updateNMPCParameters();
    
    // ========== 创建发布器 ==========
    if (publish_path) {
        predicted_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
            "nmpc/predicted_path", 10);
    }
    if (publish_markers) {
        debug_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "nmpc/debug_markers", 10);
    }
    
    // ========== 订阅里程计（来自 small_point_lio）==========
    // 用于获取真实速度反馈，实现闭环控制
    std::string odom_topic = node_->declare_parameter("nmpc.odom_topic", "/Odometry");
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(),
        std::bind(&NMPC::odomCallback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "NMPC: 订阅里程计话题 %s", odom_topic.c_str());
    
    RCLCPP_INFO(node_->get_logger(), 
        "✓ NMPC Controller initialized (N=%d, T=%.2f s)", 
        N_horizon_, T_horizon_);
    
    initialized_ = true;
}

void NMPC::setPath(const nav_msgs::msg::Path& path) {
    if (path.poses.empty()) {
        global_path_.poses.clear();
        return;
    }
    
    // 保留完整路径用于后续处理
    global_path_ = path;
    nearest_idx_ = 0;
    
    RCLCPP_INFO(node_->get_logger(), 
        "NMPC: 接收新路径, %zu 个点", path.poses.size());
}

void NMPC::setMap(nav_core::MapInterface::Ptr map) {
    map_ = map;
    // 尝试向下转型为 LayeredMapManager 以获取 ESDF 查询接口
    map_manager_ = std::dynamic_pointer_cast<LayeredMapManager>(map);
    if (map_manager_) {
        RCLCPP_INFO(node_->get_logger(), 
            "NMPC: ESDF 地图已连接 (hasEsdf=%d)", map_manager_->hasEsdf());
    } else {
        RCLCPP_WARN(node_->get_logger(), 
            "NMPC: 地图接口不是 LayeredMapManager，ESDF 代价将禁用");
    }
}

nav_core::ControlResult NMPC::computeVelocity(
    const geometry_msgs::msg::PoseStamped& current_pose,
    geometry_msgs::msg::Twist& cmd_vel) 
{
    static int call_count = 0;
    call_count++;
    
    if (!initialized_ || global_path_.poses.empty()) {
        if (call_count % 50 == 1) {
            RCLCPP_WARN(node_->get_logger(), 
                "NMPC: 未初始化或无路径 (init=%d, path_size=%zu)", 
                initialized_, global_path_.poses.size());
        }
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return nav_core::ControlResult::FAILED;
    }
    
    // 1. 提取当前状态 [x, y, theta, v, omega]
    std::vector<double> x0(5);
    x0[0] = current_pose.pose.position.x;
    x0[1] = current_pose.pose.position.y;
    x0[2] = tf2::getYaw(current_pose.pose.orientation);
    
    // 从里程计获取真实速度（闭环反馈）
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (odom_received_) {
            // small_point_lio 输出的速度在 body 坐标系下
            // linear.x = 前向速度, angular.z = 角速度
            x0[3] = latest_odom_.twist.twist.linear.x;
            x0[4] = latest_odom_.twist.twist.angular.z;
        } else {
            // 里程计尚未收到，使用上次输出（开环估计）
            x0[3] = last_state_[3];
            x0[4] = last_state_[4];
            if (call_count % 100 == 1) {
                RCLCPP_WARN(node_->get_logger(), 
                    "NMPC: 未收到里程计，使用开环速度估计");
            }
        }
    }
    
    // 2. 检查是否到达目标（仅检查 xy 距离，不要求 yaw 对齐）
    const auto& goal = global_path_.poses.back().pose;
    double dx = goal.position.x - x0[0];
    double dy = goal.position.y - x0[1];
    double dist = std::hypot(dx, dy);
    
    if (dist < xy_tolerance_) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        RCLCPP_INFO(node_->get_logger(), "NMPC: 到达目标 (xy距离=%.3fm)", dist);
        return nav_core::ControlResult::SUCCEEDED;
    }
    
    // 3. 提取局部参考轨迹
    auto yref_sequence = extractLocalReference(current_pose.pose, params_.horizon_length);
    
    // ✅ FIX: 验证参考轨迹质量
    if (yref_sequence.size() < static_cast<size_t>(N_horizon_ + 1)) {
        RCLCPP_WARN(node_->get_logger(), 
            "NMPC: 参考轨迹点不足 (%zu < %d)", 
            yref_sequence.size(), N_horizon_ + 1);
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return nav_core::ControlResult::FAILED;
    }
    
    // ✅ FIX: 检查参考轨迹连续性（防止跳变）
    for (size_t i = 1; i < yref_sequence.size(); ++i) {
        double dx = yref_sequence[i][0] - yref_sequence[i-1][0];
        double dy = yref_sequence[i][1] - yref_sequence[i-1][1];
        double step_dist = std::hypot(dx, dy);
        
        // 单步距离不应超过 desired_velocity * dt * 1.5（允许15%误差）
        double max_step = params_.desired_velocity * (T_horizon_ / N_horizon_) * 1.5;
        if (step_dist > max_step) {
            RCLCPP_WARN(node_->get_logger(),
                "NMPC: 参考轨迹不连续 (step %zu: %.2f m > %.2f m)",
                i, step_dist, max_step);
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            return nav_core::ControlResult::FAILED;
        }
    }
    
    // 4. 求解 NMPC
    std::vector<double> u_opt;
    auto t_start = std::chrono::high_resolution_clock::now();
    
    int status = solveNMPC(x0, yref_sequence, u_opt);
    
    auto t_end = std::chrono::high_resolution_clock::now();
    double solve_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    // 更新统计
    stats_.solve_count++;
    stats_.avg_solve_time_ms = (stats_.avg_solve_time_ms * (stats_.solve_count - 1) + solve_time_ms) / stats_.solve_count;
    stats_.max_solve_time_ms = std::max(stats_.max_solve_time_ms, solve_time_ms);
    
    if (status != 0 || u_opt.size() != 2) {
        stats_.consecutive_failures++;
        RCLCPP_WARN(node_->get_logger(), 
            "NMPC: 求解失败 (status=%d, 连续失败=%d, solve_time=%.2f ms)", 
            status, stats_.consecutive_failures, solve_time_ms);
        
        // 连续失败超过阈值，直接返回失败
        const int max_consecutive_failures = 10;  // 约 0.5 秒 @ 20Hz
        if (stats_.consecutive_failures >= max_consecutive_failures) {
            RCLCPP_ERROR(node_->get_logger(), 
                "NMPC: 连续求解失败 %d 次，停止控制", stats_.consecutive_failures);
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            return nav_core::ControlResult::FAILED;
        }
        
        // 平滑停车：逐渐减速而非突然停止
        double decay = 0.8;  // 每周期衰减到 80%
        cmd_vel.linear.x = last_state_[3] * decay;
        cmd_vel.angular.z = last_state_[4] * decay;
        last_state_[3] = cmd_vel.linear.x;
        last_state_[4] = cmd_vel.angular.z;
        return nav_core::ControlResult::RUNNING;  // 继续尝试
    }
    
    // 求解成功，重置失败计数
    stats_.consecutive_failures = 0;
    
    // 5. 提取并发布预测轨迹（如果启用）
    if (predicted_path_pub_) {
        publishPredictedPath(current_pose.header, x0);
    }
    
    // 6. 应用控制
    // u_opt = [a_lin, alpha_ang] (加速度)
    // 积分得到速度: v += a * dt
    double dt = T_horizon_ / N_horizon_;  // 与 solver 步长一致
    double v_cmd = x0[3] + u_opt[0] * dt;
    double omega_cmd = x0[4] + u_opt[1] * dt;
    
    // 限幅
    v_cmd = std::clamp(v_cmd, -params_.max_linear_vel, params_.max_linear_vel);
    omega_cmd = std::clamp(omega_cmd, -params_.max_angular_vel, params_.max_angular_vel);
    
    cmd_vel.linear.x = v_cmd;
    cmd_vel.angular.z = omega_cmd;
    
    // 更新状态
    last_state_ = x0;
    last_state_[3] = v_cmd;
    last_state_[4] = omega_cmd;
    last_control_ = u_opt;
    
    // 发布调试信息
    if (stats_.solve_count % 20 == 0) {
        RCLCPP_INFO(node_->get_logger(),
            "NMPC: v=%.2f m/s, ω=%.2f rad/s, solve_time=%.2f ms (avg=%.2f, max=%.2f)",
            v_cmd, omega_cmd, solve_time_ms, stats_.avg_solve_time_ms, stats_.max_solve_time_ms);
    }
    
    return nav_core::ControlResult::RUNNING;
}

void NMPC::setTolerance(double xy_tol, double yaw_tol) {
    xy_tolerance_ = xy_tol;
    yaw_tolerance_ = yaw_tol;
}

void NMPC::reset() {
    nearest_idx_ = 0;
    std::fill(last_state_.begin(), last_state_.end(), 0.0);
    std::fill(last_control_.begin(), last_control_.end(), 0.0);
    stats_ = {};
    odom_received_ = false;
    
    // 重置 solver
    if (acados_ocp_capsule_) {
        wheelleg_nmpc_acados_reset(acados_ocp_capsule_, 1);
    }
}

void NMPC::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = *msg;
    odom_received_ = true;
}

// ========== 私有方法实现 ==========

int NMPC::solveNMPC(
    const std::vector<double>& x0,
    const std::vector<std::vector<double>>& yref,
    std::vector<double>& u_opt)
{
    if (!acados_ocp_capsule_) return -1;
    
    // 设置初始状态约束
    double x0_array[5] = {x0[0], x0[1], x0[2], x0[3], x0[4]};
    
    // 获取 acados 接口
    ocp_nlp_config* nlp_config = wheelleg_nmpc_acados_get_nlp_config(acados_ocp_capsule_);
    ocp_nlp_dims* nlp_dims = wheelleg_nmpc_acados_get_nlp_dims(acados_ocp_capsule_);
    ocp_nlp_in* nlp_in = wheelleg_nmpc_acados_get_nlp_in(acados_ocp_capsule_);
    ocp_nlp_out* nlp_out = wheelleg_nmpc_acados_get_nlp_out(acados_ocp_capsule_);
    
    // 设置初始状态约束 (等式约束: x0 必须等于当前状态)
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0_array);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0_array);
    
    // ========== 角度连续化 ==========
    // 关键修复：相对于前一个点逐步连续化，避免轨迹内部的 ±π 跳变
    // 这样可以保证整个参考轨迹在 solver 看来是平滑的
    std::vector<double> theta_adjusted(yref.size());
    
    if (!yref.empty()) {
        // 第一个点相对当前状态 wrap
        double diff = yref[0][2] - x0[2];
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        theta_adjusted[0] = x0[2] + diff;
        
        // 后续点相对前一个 adjusted 点连续化
        for (size_t i = 1; i < yref.size(); ++i) {
            double diff_to_prev = yref[i][2] - yref[i-1][2];
            while (diff_to_prev > M_PI) diff_to_prev -= 2.0 * M_PI;
            while (diff_to_prev < -M_PI) diff_to_prev += 2.0 * M_PI;
            theta_adjusted[i] = theta_adjusted[i-1] + diff_to_prev;
        }
    }

    // 注入参考与 ESDF 参数（external cost）
    injectEsdfParameters(yref, theta_adjusted);
    
    // 求解
    int status = wheelleg_nmpc_acados_solve(acados_ocp_capsule_);
    
    // 提取最优控制
    if (status == 0) {
        double u[2];
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u);
        u_opt = {u[0], u[1]};
    }
    
    return status;
}

std::vector<std::vector<double>> NMPC::extractLocalReference(
    const geometry_msgs::msg::Pose& current_pose,
    double horizon_length)
{
    std::vector<std::vector<double>> yref;
    
    if (global_path_.poses.empty()) return yref;
    
    // 查找最近点并剪枝:只保留从最近点到终点的路径
    nearest_idx_ = findNearestPathPoint(current_pose);
    
    // 提前终止:如果最近点已经接近路径末尾,直接使用剩余路径
    size_t pruned_start_idx = (nearest_idx_ > 5) ? (nearest_idx_ - 5) : 0;
    
    // 沿路径前向采样 N+1 个点 (在 odom 坐标系下)
    double dt = T_horizon_ / N_horizon_;
    
    // 计算到终点的总距离（用于渐进减速）
    double total_dist_to_goal = 0.0;
    const auto& goal_pos = global_path_.poses.back().pose.position;
    const auto& nearest_pos = global_path_.poses[nearest_idx_].pose.position;
    for (size_t k = nearest_idx_; k < global_path_.poses.size() - 1; ++k) {
        auto& p1 = global_path_.poses[k].pose.position;
        auto& p2 = global_path_.poses[k + 1].pose.position;
        total_dist_to_goal += std::hypot(p2.x - p1.x, p2.y - p1.y);
    }
    
    // 记住上一个找到的索引，确保参考轨迹单调递增
    int last_found_idx = nearest_idx_;
    double accumulated_dist = 0.0;  // 从 nearest_idx_ 开始的累积距离
    
    for (int i = 0; i <= N_horizon_; ++i) {
        // 计算期望的前向距离
        double forward_dist = params_.desired_velocity * i * dt;
        
        // ✅ 从上一个点继续搜索（保证单调性）
        int idx = last_found_idx;
        double dist = accumulated_dist;
        
        while (idx < static_cast<int>(global_path_.poses.size()) - 1) {
            auto& p1 = global_path_.poses[idx].pose.position;
            auto& p2 = global_path_.poses[idx + 1].pose.position;
            double seg_dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
            
            if (dist + seg_dist >= forward_dist) {
                // 插值 (在 odom 坐标系下)
                double ratio = (forward_dist - dist) / seg_dist;
                double x = p1.x + ratio * (p2.x - p1.x);
                double y = p1.y + ratio * (p2.y - p1.y);
                
                // ========== 角度插值：保证连续性 ==========
                double theta1 = tf2::getYaw(global_path_.poses[idx].pose.orientation);
                double theta2 = tf2::getYaw(global_path_.poses[idx + 1].pose.orientation);
                
                // 计算最短角度差
                double theta_diff = theta2 - theta1;
                while (theta_diff > M_PI) theta_diff -= 2.0 * M_PI;
                while (theta_diff < -M_PI) theta_diff += 2.0 * M_PI;
                
                // 线性插值角度
                double theta = theta1 + ratio * theta_diff;
                
                // 渐进减速逻辑：根据到终点的距离调整期望速度
                double remaining_dist = total_dist_to_goal - dist;
                double desired_v = params_.desired_velocity;
                
                // 在 2.0m 范围内开始减速，线性降到 0.3m/s
                const double decel_start_dist = 2.0;
                const double min_speed = 0.3;
                if (remaining_dist < decel_start_dist) {
                    desired_v = min_speed + 
                        (params_.desired_velocity - min_speed) * (remaining_dist / decel_start_dist);
                }
                
                // ========== 横向误差自适应速度缩减 (首次规划 i=0 时) ==========
                // 根据图片策略：当 e_cross 较大时，通过速度夹角 α 降低纵向速度
                if (i == 0) {
                    // 计算当前位置到参考点的横向误差 e_cross
                    double dx_robot_ref = x - current_pose.position.x;
                    double dy_robot_ref = y - current_pose.position.y;
                    double e_cross = std::abs(
                        -std::sin(theta) * dx_robot_ref + std::cos(theta) * dy_robot_ref
                    );
                    
                    // 获取当前机器人速度（body frame）
                    double v_robot_x = last_state_[3];  // 前向速度
                    double v_robot_y = 0.0;             // 差速模型侧向速度为0
                    
                    // 参考点速度（沿轨迹切线方向）
                    double v_ref_x = desired_v * std::cos(theta);
                    double v_ref_y = desired_v * std::sin(theta);
                    
                    // 计算速度夹角 α (在全局坐标系下)
                    double robot_theta = tf2::getYaw(current_pose.orientation);
                    double v_robot_global_x = v_robot_x * std::cos(robot_theta);
                    double v_robot_global_y = v_robot_x * std::sin(robot_theta);
                    
                    double v_prim_norm = std::hypot(v_robot_global_x, v_robot_global_y);
                    double v_ref_norm = std::hypot(v_ref_x, v_ref_y);
                    
                    double alpha = 0.0;
                    if (v_prim_norm > 0.01 && v_ref_norm > 0.01) {
                        double dot_product = v_robot_global_x * v_ref_x + v_robot_global_y * v_ref_y;
                        double cross_product = v_robot_global_x * v_ref_y - v_robot_global_y * v_ref_x;
                        alpha = std::max(0.0, std::atan2(std::abs(cross_product), dot_product));
                    }
                    
                    // 速度缩减系数：当 e_cross 较大时，通过 α 降低纵向速度
                    if (e_cross > params_.lateral_error_threshold) {
                        double shrink_ratio = std::max(0.0, 
                            (v_prim_norm * std::cos(alpha)) / (v_ref_norm + 1e-6)
                        );
                        shrink_ratio = std::clamp(shrink_ratio, 0.3, 1.0);  // 限制最低 30% 速度
                        desired_v *= shrink_ratio;
                        
                        if (stats_.solve_count % 20 == 0) {
                            RCLCPP_INFO(node_->get_logger(),
                                "NMPC: 横向误差=%.2fm > %.2f, α=%.1f°, 速度缩减至%.2f (%.0f%%)",
                                e_cross, params_.lateral_error_threshold,
                                alpha * 180.0 / M_PI, desired_v, shrink_ratio * 100.0);
                        }
                    }
                }
                
                yref.push_back({x, y, theta, desired_v, 0.0});
                
                // ✅ 更新索引和累积距离
                last_found_idx = idx;
                accumulated_dist = dist;
                break;
            }
            
            dist += seg_dist;
            idx++;
        }
        
        // 如果到达路径末端
        if (idx >= static_cast<int>(global_path_.poses.size()) - 1) {
            auto& last = global_path_.poses.back().pose;
            yref.push_back({
                last.position.x, 
                last.position.y,
                tf2::getYaw(last.orientation),
                0.0,  // 终点速度为0
                0.0
            });
            // ✅ 到达终点后，后续所有参考点都是终点
            break;
        }
    }
    
    // ✅ 如果提前到达终点，填充剩余参考点为终点（保证 N+1 个点）
    while (yref.size() < static_cast<size_t>(N_horizon_ + 1)) {
        auto& last = global_path_.poses.back().pose;
        yref.push_back({
            last.position.x, 
            last.position.y,
            tf2::getYaw(last.orientation),
            0.0,
            0.0
        });
    }
    
    return yref;
}

int NMPC::findNearestPathPoint(const geometry_msgs::msg::Pose& pose) {
    double min_dist = std::numeric_limits<double>::max();
    int nearest = nearest_idx_;
    
    // 阶段1: 局部窗口搜索 (快速路径)
    int local_backward = 10;  // 向后搜索10个点
    int local_forward = 30;   // 向前搜索30个点
    int search_start = std::max(0, nearest_idx_ - local_backward);
    int search_end = std::min(static_cast<int>(global_path_.poses.size()), 
                              nearest_idx_ + local_forward);
    
    for (int i = search_start; i < search_end; ++i) {
        double dx = global_path_.poses[i].pose.position.x - pose.position.x;
        double dy = global_path_.poses[i].pose.position.y - pose.position.y;
        double dist = std::hypot(dx, dy);
        
        if (dist < min_dist) {
            min_dist = dist;
            nearest = i;
        }
    }
    
    // 阶段2: 回退机制 - 如果偏离过远,执行全局搜索
    const double deviation_threshold = 2.0;  // 2米阈值
    if (min_dist > deviation_threshold) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "NMPC: 机器人偏离路径过远 (%.2f m > %.2f m)", 
            min_dist, deviation_threshold);
        
        min_dist = std::numeric_limits<double>::max();
        nearest = 0;
        
        // 全局搜索最近点
        for (size_t i = 0; i < global_path_.poses.size(); ++i) {
            double dx = global_path_.poses[i].pose.position.x - pose.position.x;
            double dy = global_path_.poses[i].pose.position.y - pose.position.y;
            double dist = std::hypot(dx, dy);
            
            if (dist < min_dist) {
                min_dist = dist;
                nearest = static_cast<int>(i);
            }
        }
        
        RCLCPP_INFO(node_->get_logger(), 
            "NMPC: 最近点索引=%d, 距离=%.2f m", nearest, min_dist);
    }
    
    return nearest;
}

void NMPC::updateNMPCParameters() {
    if (!acados_ocp_capsule_) {
        RCLCPP_WARN(node_->get_logger(), "NMPC: acados solver 未初始化,跳过参数更新");
        return;
    }
    
    // 获取 acados 接口
    ocp_nlp_config* nlp_config = wheelleg_nmpc_acados_get_nlp_config(acados_ocp_capsule_);
    ocp_nlp_dims* nlp_dims = wheelleg_nmpc_acados_get_nlp_dims(acados_ocp_capsule_);
    ocp_nlp_in* nlp_in = wheelleg_nmpc_acados_get_nlp_in(acados_ocp_capsule_);
    ocp_nlp_out* nlp_out = wheelleg_nmpc_acados_get_nlp_out(acados_ocp_capsule_);
    
    // ========== 1. 更新状态约束 (速度限制) ==========
    double v_lower = params_.allow_reverse ? -params_.max_linear_vel : 0.0;
    for (int i = 1; i < N_horizon_; ++i) {
        double lbx[2] = {v_lower, -params_.max_angular_vel};
        double ubx[2] = {params_.max_linear_vel, params_.max_angular_vel};
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubx", ubx);
    }
    
    // ========== 2. 更新控制约束 (加速度限制) ==========
    for (int i = 0; i < N_horizon_; ++i) {
        double lbu[2] = {-params_.max_linear_accel, -params_.max_angular_accel};
        double ubu[2] = {params_.max_linear_accel, params_.max_angular_accel};
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubu", ubu);
    }
    
    RCLCPP_INFO(node_->get_logger(), 
        "✓ NMPC 参数已更新: v_max=%.1f, ω_max=%.1f, Q_pos=%.1f, R_lin=%.2f, esdf_w=%.1f",
        params_.max_linear_vel, params_.max_angular_vel, params_.Q_position, 
        params_.R_linear, params_.esdf_weight);
}

void NMPC::injectEsdfParameters(const std::vector<std::vector<double>>& yref,
                              const std::vector<double>& theta_adjusted) {
    if (!acados_ocp_capsule_) return;

    // 参数布局: [x_ref, y_ref, theta_ref, v_ref, omega_ref, a_ref, alpha_ref, d_esdf, weight_scale]
    double p_default[NP_PARAM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 1.0};

    bool has_esdf = params_.enable_esdf_cost && map_manager_ && map_manager_->hasEsdf();
    auto esdf = has_esdf ? map_manager_->getEsdf() : nullptr;

    // 近端权重递增: 前 1/4 时域内乘以 near_weight_multiplier
    int near_end = std::max(1, N_horizon_ / 4);

    // 获取 solver 上一轮的预测轨迹用于查询点混合
    ocp_nlp_config* nlp_config = wheelleg_nmpc_acados_get_nlp_config(acados_ocp_capsule_);
    ocp_nlp_dims* nlp_dims = wheelleg_nmpc_acados_get_nlp_dims(acados_ocp_capsule_);
    ocp_nlp_out* nlp_out = wheelleg_nmpc_acados_get_nlp_out(acados_ocp_capsule_);

    for (int i = 0; i <= N_horizon_; ++i) {
        double p_values[NP_PARAM];
        std::copy(p_default, p_default + NP_PARAM, p_values);

        int idx = std::min(i, static_cast<int>(yref.size()) - 1);

        // 参考注入
        p_values[0] = yref[idx][0];
        p_values[1] = yref[idx][1];
        p_values[2] = theta_adjusted[idx];
        p_values[3] = yref[idx][3];
        p_values[4] = yref[idx][4];
        p_values[5] = 0.0;  // 控制参考恒为 0 (希望减小加速度)
        p_values[6] = 0.0;

        // 近端/终端权重缩放
        double weight_scale = 1.0;
        if (i < near_end) {
            double ratio = static_cast<double>(near_end - i) / near_end;
            weight_scale = 1.0 + (params_.near_weight_multiplier - 1.0) * ratio;
        }
        if (i == N_horizon_) {
            weight_scale = params_.terminal_multiplier;
        }
        p_values[8] = weight_scale;

        // ESDF 查询 (仅距离)
        double dist = 10.0;
        if (has_esdf && esdf) {
            double x_pred[5] = {0};
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", x_pred);

            double alpha = (stats_.solve_count == 0) ? 1.0 : 0.7;
            double qx = alpha * yref[idx][0] + (1.0 - alpha) * x_pred[0];
            double qy = alpha * yref[idx][1] + (1.0 - alpha) * x_pred[1];

            double gx = 0.0, gy = 0.0;
            if (esdf->getDistanceAndGradient(qx, qy, dist, gx, gy)) {
                // dist 已填充
            }
        }
        p_values[7] = dist;

        wheelleg_nmpc_acados_update_params(acados_ocp_capsule_, i, p_values, NP_PARAM);
    }
}

void NMPC::publishPredictedPath(const std_msgs::msg::Header& header, const std::vector<double>& x0) {
    if (!predicted_path_pub_ || !acados_ocp_capsule_) {
        return;
    }
    
    ocp_nlp_config* nlp_config = wheelleg_nmpc_acados_get_nlp_config(acados_ocp_capsule_);
    ocp_nlp_dims* nlp_dims = wheelleg_nmpc_acados_get_nlp_dims(acados_ocp_capsule_);
    ocp_nlp_out* nlp_out = wheelleg_nmpc_acados_get_nlp_out(acados_ocp_capsule_);
    
    nav_msgs::msg::Path predicted_path;
    predicted_path.header = header;
    predicted_path.header.frame_id = "map";
    
    // 提取预测状态 x = [x, y, theta, v, omega]
    std::vector<double> x_pred(5);
    
    for (int i = 0; i <= N_horizon_; ++i) {
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", x_pred.data());
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header = predicted_path.header;
        pose.pose.position.x = x_pred[0];
        pose.pose.position.y = x_pred[1];
        pose.pose.position.z = 0.0;
        
        // theta 转四元数
        tf2::Quaternion q;
        q.setRPY(0, 0, x_pred[2]);
        pose.pose.orientation = tf2::toMsg(q);
        
        predicted_path.poses.push_back(pose);
    }
    
    predicted_path_pub_->publish(predicted_path);
}

}  // namespace nav_components
