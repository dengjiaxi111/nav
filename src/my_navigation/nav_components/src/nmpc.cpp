// NMPC controller implementation
// 集成 acados 生成的 solver 到 ROS2 导航框架

#include "nav_components/nmpc.hpp"
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
    
    // 局部参考提取
    node_->declare_parameter("nmpc.horizon_length", params_.horizon_length);
    node_->declare_parameter("nmpc.desired_velocity", params_.desired_velocity);
    
    // 代价权重
    node_->declare_parameter("nmpc.Q_position", params_.Q_position);
    node_->declare_parameter("nmpc.Q_orientation", params_.Q_orientation);
    node_->declare_parameter("nmpc.Q_velocity", params_.Q_velocity);
    node_->declare_parameter("nmpc.R_linear", params_.R_linear);
    node_->declare_parameter("nmpc.R_angular", params_.R_angular);
    node_->declare_parameter("nmpc.terminal_multiplier", params_.terminal_multiplier);
    
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
    
    params_.horizon_length = node_->get_parameter("nmpc.horizon_length").as_double();
    params_.desired_velocity = node_->get_parameter("nmpc.desired_velocity").as_double();
    
    params_.Q_position = node_->get_parameter("nmpc.Q_position").as_double();
    params_.Q_orientation = node_->get_parameter("nmpc.Q_orientation").as_double();
    params_.Q_velocity = node_->get_parameter("nmpc.Q_velocity").as_double();
    params_.R_linear = node_->get_parameter("nmpc.R_linear").as_double();
    params_.R_angular = node_->get_parameter("nmpc.R_angular").as_double();
    params_.terminal_multiplier = node_->get_parameter("nmpc.terminal_multiplier").as_double();
    
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
    debug_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "nmpc/debug_markers", 10);
    
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

nav_core::ControlResult NMPC::computeVelocity(
    const geometry_msgs::msg::PoseStamped& current_pose,
    geometry_msgs::msg::Twist& cmd_vel) 
{
    static int call_count = 0;
    
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
    
    // 1. 提取当前状态 [x, y, theta, v, omega] (odom 坐标系)
    std::vector<double> x0(5);
    x0[0] = current_pose.pose.position.x;
    x0[1] = current_pose.pose.position.y;
    x0[2] = tf2::getYaw(current_pose.pose.orientation);
    x0[3] = last_state_[3];  // 使用上一次速度 (或从 odometry 获取)
    x0[4] = last_state_[4];  // 使用上一次角速度
    
    // 2. 检查是否到达目标
    const auto& goal = global_path_.poses.back().pose;
    double dx = goal.position.x - x0[0];
    double dy = goal.position.y - x0[1];
    double dist = std::hypot(dx, dy);
    double yaw_err = std::abs(tf2::getYaw(goal.orientation) - x0[2]);
    
    if (dist < xy_tolerance_ && yaw_err < yaw_tolerance_) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        RCLCPP_INFO(node_->get_logger(), "NMPC: 到达目标");
        return nav_core::ControlResult::SUCCEEDED;
    }
    
    // 3. 提取局部参考轨迹
    auto yref_sequence = extractLocalReference(current_pose.pose, params_.horizon_length);
    
    if (yref_sequence.empty()) {
        RCLCPP_WARN(node_->get_logger(), "NMPC: 无法提取参考轨迹");
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return nav_core::ControlResult::FAILED;
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
        RCLCPP_WARN(node_->get_logger(), 
            "NMPC: 求解失败 (status=%d, solve_time=%.2f ms)", status, solve_time_ms);
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return nav_core::ControlResult::FAILED;
    }
    
    // 5. 应用控制
    // u_opt = [a_lin, alpha_ang] (加速度)
    // 积分得到速度 (简化: 假设控制周期 dt = 0.05s)
    double dt = 0.05;
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
    
    // 重置 solver
    if (acados_ocp_capsule_) {
        wheelleg_nmpc_acados_reset(acados_ocp_capsule_, 1);
    }
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
    
    // 设置参考轨迹
    for (int i = 0; i < N_horizon_ && i < static_cast<int>(yref.size()); ++i) {
        double yref_i[7] = {
            yref[i][0], yref[i][1], yref[i][2], yref[i][3], yref[i][4],
            0.0, 0.0  // 控制参考 (期望为0)
        };
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_i);
    }
    
    // 终端参考
    if (!yref.empty()) {
        double yref_e[5] = {yref.back()[0], yref.back()[1], yref.back()[2], 
                            yref.back()[3], yref.back()[4]};
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N_horizon_, "yref", yref_e);
    }
    
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
    
    for (int i = 0; i <= N_horizon_; ++i) {
        // 计算期望的前向距离
        double forward_dist = params_.desired_velocity * i * dt;
        
        // 在路径上查找对应点 (从剪枝后的起点开始)
        int idx = nearest_idx_;
        double dist = 0.0;
        
        while (idx < static_cast<int>(global_path_.poses.size()) - 1) {
            auto& p1 = global_path_.poses[idx].pose.position;
            auto& p2 = global_path_.poses[idx + 1].pose.position;
            double seg_dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
            
            if (dist + seg_dist >= forward_dist) {
                // 插值 (在 odom 坐标系下)
                double ratio = (forward_dist - dist) / seg_dist;
                double x = p1.x + ratio * (p2.x - p1.x);
                double y = p1.y + ratio * (p2.y - p1.y);
                double theta = tf2::getYaw(global_path_.poses[idx + 1].pose.orientation);
                
                yref.push_back({x, y, theta, params_.desired_velocity, 0.0});
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
        }
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
    // 状态约束作用于所有 shooting nodes (1 到 N-1)
    for (int i = 1; i < N_horizon_; ++i) {
        // idxbx = [3, 4] 对应 [v, omega]
        double lbx[2] = {-params_.max_linear_vel, -params_.max_angular_vel};
        double ubx[2] = {params_.max_linear_vel, params_.max_angular_vel};
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubx", ubx);
    }
    
    // ========== 2. 更新控制约束 (加速度限制) ==========
    // 控制约束作用于所有控制输入 (0 到 N-1)
    for (int i = 0; i < N_horizon_; ++i) {
        double lbu[2] = {-params_.max_linear_accel, -params_.max_angular_accel};
        double ubu[2] = {params_.max_linear_accel, params_.max_angular_accel};
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubu", ubu);
    }
    
    // ========== 3. 更新代价权重矩阵 ==========
    // 构造 Q 矩阵 (状态权重)
    double Q_diag[5] = {
        params_.Q_position,    // x
        params_.Q_position,    // y
        params_.Q_orientation, // theta
        params_.Q_velocity,    // v
        params_.Q_velocity     // omega
    };
    
    // 构造 R 矩阵 (控制权重)
    double R_diag[2] = {
        params_.R_linear,      // a_lin
        params_.R_angular      // alpha_ang
    };
    
    // LINEAR_LS 代价需要设置 W 矩阵 (7x7 对角阵 = [Q, R])
    double W[49] = {0};  // 7x7 矩阵
    for (int i = 0; i < 5; ++i) {
        W[i * 7 + i] = Q_diag[i];  // 对角线元素 Q
    }
    for (int i = 0; i < 2; ++i) {
        W[(5 + i) * 7 + (5 + i)] = R_diag[i];  // 对角线元素 R
    }
    
    // 初始阶段代价 (stage 0)
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W);
    
    // 中间阶段代价 (stage 1 到 N-1)
    for (int i = 1; i < N_horizon_; ++i) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    
    // 终端代价 (stage N) - 只有状态权重,乘以 terminal_multiplier
    double W_e[25] = {0};  // 5x5 矩阵
    for (int i = 0; i < 5; ++i) {
        W_e[i * 5 + i] = Q_diag[i] * params_.terminal_multiplier;
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N_horizon_, "W", W_e);
    
    RCLCPP_INFO(node_->get_logger(), 
        "✓ NMPC 参数已更新: v_max=%.1f, ω_max=%.1f, Q_pos=%.1f, R_lin=%.2f",
        params_.max_linear_vel, params_.max_angular_vel, params_.Q_position, params_.R_linear);
}

}  // namespace nav_components
