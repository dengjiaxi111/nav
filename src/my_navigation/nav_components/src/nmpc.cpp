// NMPC controller implementation
// 集成 acados 生成的 solver 到 ROS2 导航框架

#include "nav_components/nmpc.hpp"
#include "nav_components/layered_map_manager.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 提供 tf2::fromMsg
#include <tf2/utils.h>
#include <cmath>
#include <fstream>
#include <chrono>
#include <limits>
#include <algorithm>

// acados solver C 接口
extern "C" {
    #include "acados_solver_wheelleg_nmpc.h"
}

namespace nav_components {

NMPC::NMPC() {
    last_state_.resize(NX_STATE, 0.0);
    last_control_.resize(2, 0.0);
}

NMPC::~NMPC() {
    if (acados_ocp_capsule_) {
        wheelleg_nmpc_acados_free(acados_ocp_capsule_);
    }
}

void NMPC::initialize(rclcpp::Node* node) {
    static_assert(WHEELLEG_NMPC_NX == NX_STATE,
        "acados solver NX is stale; regenerate model_ocp/export_ocp.py after NMPC model changes");
    static_assert(WHEELLEG_NMPC_NP == NP_PARAM,
        "acados solver NP is stale; regenerate model_ocp/export_ocp.py after NMPC parameter changes");

    node_ = node;
    
    // ========== 声明所有 ROS 参数 ==========
    // 运动约束
    node_->declare_parameter("nmpc.max_linear_vel", params_.max_linear_vel);
    node_->declare_parameter("nmpc.max_state_linear_vel", params_.max_state_linear_vel);
    node_->declare_parameter("nmpc.min_state_linear_vel", params_.min_state_linear_vel);
    node_->declare_parameter("nmpc.max_angular_vel", params_.max_angular_vel);
    node_->declare_parameter("nmpc.max_linear_accel", params_.max_linear_accel);
    node_->declare_parameter("nmpc.max_linear_decel", params_.max_linear_decel);
    node_->declare_parameter("nmpc.max_angular_accel", params_.max_angular_accel);
    node_->declare_parameter("nmpc.allow_reverse", params_.allow_reverse);
    
    // 局部参考提取
    node_->declare_parameter("nmpc.horizon_length", params_.horizon_length);
    node_->declare_parameter("nmpc.desired_velocity", params_.desired_velocity);
    node_->declare_parameter("nmpc.use_omega_ref_from_path", params_.use_omega_ref_from_path);
    node_->declare_parameter("nmpc.goal_crawl_speed", params_.goal_crawl_speed);
    node_->declare_parameter("nmpc.enable_goal_speed_limit", params_.enable_goal_speed_limit);
    node_->declare_parameter("nmpc.goal_slowdown_dist", params_.goal_slowdown_dist);
    node_->declare_parameter("nmpc.goal_min_moving_speed", params_.goal_min_moving_speed);
    node_->declare_parameter("nmpc.goal_max_slow_speed", params_.goal_max_slow_speed);
    node_->declare_parameter("nmpc.pivot_turn_heading_thresh", params_.pivot_turn_heading_thresh);
    node_->declare_parameter("nmpc.pivot_turn_startup_only", params_.pivot_turn_startup_only);
    node_->declare_parameter("nmpc.startup_align.enable", params_.startup_align_enable);
    node_->declare_parameter("nmpc.startup_align.enter_thresh",
                             params_.startup_align_enter_thresh);
    node_->declare_parameter("nmpc.startup_align.exit_thresh",
                             params_.startup_align_exit_thresh);
    node_->declare_parameter("nmpc.startup_align.lookahead",
                             params_.startup_align_lookahead);
    node_->declare_parameter("nmpc.startup_align.kp", params_.startup_align_kp);
    node_->declare_parameter("nmpc.startup_align.min_angular_vel",
                             params_.startup_align_min_angular_vel);
    node_->declare_parameter("nmpc.startup_align.max_angular_vel",
                             params_.startup_align_max_angular_vel);
    node_->declare_parameter("nmpc.speed_profile.enable", params_.speed_profile_enable);
    node_->declare_parameter("nmpc.speed_profile.v_cruise", params_.speed_profile_v_cruise);
    node_->declare_parameter("nmpc.speed_profile.v_min", params_.speed_profile_v_min);
    node_->declare_parameter("nmpc.speed_profile.max_lateral_accel",
                             params_.speed_profile_max_lateral_accel);
    node_->declare_parameter("nmpc.speed_profile.kappa_epsilon",
                             params_.speed_profile_kappa_epsilon);
    node_->declare_parameter("nmpc.speed_profile.curvature_window_m",
                             params_.speed_profile_curvature_window_m);
    node_->declare_parameter("nmpc.enable_curvature_horizon_adapt", params_.enable_curvature_horizon_adapt);
    node_->declare_parameter("nmpc.horizon_kappa_scale", params_.horizon_kappa_scale);
    node_->declare_parameter("nmpc.horizon_min_length", params_.horizon_min_length);
    node_->declare_parameter("nmpc.odom_feedback_alpha", params_.odom_feedback_alpha);
    node_->declare_parameter("nmpc.velocity_feedback_source", params_.velocity_feedback_source);
    node_->declare_parameter("nmpc.chassis_velocity_timeout_sec",
                             params_.chassis_velocity_timeout_sec);
    node_->declare_parameter("nmpc.chassis_velocity_filter_alpha",
                             params_.chassis_velocity_filter_alpha);
    node_->declare_parameter("nmpc.vel_lag_tau", params_.vel_lag_tau);
    node_->declare_parameter("nmpc.omega_lag_tau", params_.omega_lag_tau);
    node_->declare_parameter("nmpc.vel_lag_zeta", params_.vel_lag_zeta);
    node_->declare_parameter("nmpc.omega_lag_zeta", params_.omega_lag_zeta);
    node_->declare_parameter("nmpc.capacitor_limit.v_safe", params_.capacitor_v_safe);
    node_->declare_parameter("nmpc.capacitor_limit.v_low", params_.capacitor_v_low);
    node_->declare_parameter("nmpc.capacitor_limit.protect_scale",
                             params_.capacitor_protect_scale);
    node_->declare_parameter("nmpc.capacitor_limit.low_scale",
                             params_.capacitor_low_scale);
    node_->declare_parameter("nmpc.capacitor_limit.filter_alpha",
                             params_.capacitor_filter_alpha);

    // 代价权重（运行时注入）
    node_->declare_parameter("nmpc.Q_position", params_.Q_position);
    node_->declare_parameter("nmpc.Q_orientation", params_.Q_orientation);
    node_->declare_parameter("nmpc.Q_velocity", params_.Q_velocity);
    node_->declare_parameter("nmpc.Q_omega", params_.Q_omega);
    node_->declare_parameter("nmpc.R_linear", params_.R_linear);
    node_->declare_parameter("nmpc.R_angular", params_.R_angular);

    // ESDF / 轮廓代价参数（运行时注入）
    node_->declare_parameter("nmpc.esdf_weight", params_.esdf_weight);
    node_->declare_parameter("nmpc.esdf_safe_dist", params_.esdf_safe_dist);
    node_->declare_parameter("nmpc.contouring_weight", params_.contouring_weight);

    // 权重缩放
    node_->declare_parameter("nmpc.terminal_multiplier", params_.terminal_multiplier);
    
    // ESDF 障碍物代价参数（权重/安全距离由 solver 固化）
    node_->declare_parameter("nmpc.enable_esdf_cost", params_.enable_esdf_cost);
    node_->declare_parameter("nmpc.near_weight_multiplier", params_.near_weight_multiplier);
    
    // 可视化选项
    node_->declare_parameter("nmpc.publish_predicted_path", true);
    node_->declare_parameter("nmpc.publish_speed_observation", true);
    
    // ========== 读取参数 ==========
    params_.max_linear_vel = node_->get_parameter("nmpc.max_linear_vel").as_double();
    params_.max_state_linear_vel =
        node_->get_parameter("nmpc.max_state_linear_vel").as_double();
    params_.min_state_linear_vel =
        node_->get_parameter("nmpc.min_state_linear_vel").as_double();
    params_.max_angular_vel = node_->get_parameter("nmpc.max_angular_vel").as_double();
    params_.max_linear_accel = node_->get_parameter("nmpc.max_linear_accel").as_double();
    params_.max_linear_decel = node_->get_parameter("nmpc.max_linear_decel").as_double();
    params_.max_angular_accel = node_->get_parameter("nmpc.max_angular_accel").as_double();
    params_.allow_reverse = node_->get_parameter("nmpc.allow_reverse").as_bool();
    
    params_.horizon_length = node_->get_parameter("nmpc.horizon_length").as_double();
    params_.desired_velocity = node_->get_parameter("nmpc.desired_velocity").as_double();
    params_.use_omega_ref_from_path =
        node_->get_parameter("nmpc.use_omega_ref_from_path").as_bool();
    params_.goal_crawl_speed = node_->get_parameter("nmpc.goal_crawl_speed").as_double();
    params_.enable_goal_speed_limit =
        node_->get_parameter("nmpc.enable_goal_speed_limit").as_bool();
    params_.goal_slowdown_dist = node_->get_parameter("nmpc.goal_slowdown_dist").as_double();
    params_.goal_min_moving_speed =
        node_->get_parameter("nmpc.goal_min_moving_speed").as_double();
    params_.goal_max_slow_speed =
        node_->get_parameter("nmpc.goal_max_slow_speed").as_double();
    params_.pivot_turn_heading_thresh =
        node_->get_parameter("nmpc.pivot_turn_heading_thresh").as_double();
    params_.pivot_turn_startup_only =
        node_->get_parameter("nmpc.pivot_turn_startup_only").as_bool();
    params_.startup_align_enable =
        node_->get_parameter("nmpc.startup_align.enable").as_bool();
    params_.startup_align_enter_thresh =
        node_->get_parameter("nmpc.startup_align.enter_thresh").as_double();
    params_.startup_align_exit_thresh =
        node_->get_parameter("nmpc.startup_align.exit_thresh").as_double();
    params_.startup_align_lookahead =
        node_->get_parameter("nmpc.startup_align.lookahead").as_double();
    params_.startup_align_kp =
        node_->get_parameter("nmpc.startup_align.kp").as_double();
    params_.startup_align_min_angular_vel =
        node_->get_parameter("nmpc.startup_align.min_angular_vel").as_double();
    params_.startup_align_max_angular_vel =
        node_->get_parameter("nmpc.startup_align.max_angular_vel").as_double();
    params_.speed_profile_enable = node_->get_parameter("nmpc.speed_profile.enable").as_bool();
    params_.speed_profile_v_cruise =
        node_->get_parameter("nmpc.speed_profile.v_cruise").as_double();
    params_.speed_profile_v_min = node_->get_parameter("nmpc.speed_profile.v_min").as_double();
    params_.speed_profile_max_lateral_accel =
        node_->get_parameter("nmpc.speed_profile.max_lateral_accel").as_double();
    params_.speed_profile_kappa_epsilon =
        node_->get_parameter("nmpc.speed_profile.kappa_epsilon").as_double();
    params_.speed_profile_curvature_window_m =
        node_->get_parameter("nmpc.speed_profile.curvature_window_m").as_double();
    params_.odom_feedback_alpha = node_->get_parameter("nmpc.odom_feedback_alpha").as_double();
    params_.odom_feedback_alpha = std::clamp(params_.odom_feedback_alpha, 0.0, 1.0);
    params_.goal_crawl_speed = std::max(0.0, params_.goal_crawl_speed);
    params_.goal_slowdown_dist = std::max(xy_tolerance_, params_.goal_slowdown_dist);
    params_.goal_min_moving_speed = std::max(0.0, params_.goal_min_moving_speed);
    params_.goal_max_slow_speed =
        std::max(params_.goal_min_moving_speed, params_.goal_max_slow_speed);
    params_.pivot_turn_heading_thresh = std::clamp(params_.pivot_turn_heading_thresh, 0.0, M_PI);
    params_.startup_align_enter_thresh =
        std::clamp(params_.startup_align_enter_thresh, 0.0, M_PI);
    params_.startup_align_exit_thresh =
        std::clamp(params_.startup_align_exit_thresh, 0.0, params_.startup_align_enter_thresh);
    params_.startup_align_lookahead = std::max(0.05, params_.startup_align_lookahead);
    params_.startup_align_kp = std::max(0.0, params_.startup_align_kp);
    params_.startup_align_min_angular_vel =
        std::max(0.0, params_.startup_align_min_angular_vel);
    params_.startup_align_max_angular_vel =
        std::max(params_.startup_align_min_angular_vel, params_.startup_align_max_angular_vel);
    params_.speed_profile_v_cruise = std::max(0.0, params_.speed_profile_v_cruise);
    params_.speed_profile_v_min = std::max(0.0, params_.speed_profile_v_min);
    params_.speed_profile_v_min =
        std::min(params_.speed_profile_v_min, params_.speed_profile_v_cruise);
    params_.speed_profile_max_lateral_accel =
        std::max(0.05, params_.speed_profile_max_lateral_accel);
    params_.speed_profile_kappa_epsilon =
        std::max(1e-4, params_.speed_profile_kappa_epsilon);
    params_.speed_profile_curvature_window_m =
        std::max(0.05, params_.speed_profile_curvature_window_m);
    params_.velocity_feedback_source =
        node_->get_parameter("nmpc.velocity_feedback_source").as_string();
    params_.chassis_velocity_timeout_sec =
        std::max(0.0, node_->get_parameter("nmpc.chassis_velocity_timeout_sec").as_double());
    params_.chassis_velocity_filter_alpha =
        std::clamp(node_->get_parameter("nmpc.chassis_velocity_filter_alpha").as_double(),
                   0.0, 1.0);
    params_.enable_curvature_horizon_adapt =
        node_->get_parameter("nmpc.enable_curvature_horizon_adapt").as_bool();
    params_.horizon_kappa_scale =
        std::max(0.0, node_->get_parameter("nmpc.horizon_kappa_scale").as_double());
    params_.horizon_min_length =
        std::max(0.1, node_->get_parameter("nmpc.horizon_min_length").as_double());
    params_.vel_lag_tau = std::max(0.05, node_->get_parameter("nmpc.vel_lag_tau").as_double());
    params_.omega_lag_tau =
        std::max(0.05, node_->get_parameter("nmpc.omega_lag_tau").as_double());
    params_.vel_lag_zeta =
        std::max(0.05, node_->get_parameter("nmpc.vel_lag_zeta").as_double());
    params_.omega_lag_zeta =
        std::max(0.05, node_->get_parameter("nmpc.omega_lag_zeta").as_double());
    params_.max_linear_vel = std::max(0.0, params_.max_linear_vel);
    params_.max_state_linear_vel =
        std::max(params_.max_linear_vel, params_.max_state_linear_vel);
    if (params_.min_state_linear_vel > params_.max_state_linear_vel) {
        params_.min_state_linear_vel = -0.3;
    }
    params_.capacitor_v_safe =
        node_->get_parameter("nmpc.capacitor_limit.v_safe").as_double();
    params_.capacitor_v_low =
        node_->get_parameter("nmpc.capacitor_limit.v_low").as_double();
    params_.capacitor_protect_scale =
        node_->get_parameter("nmpc.capacitor_limit.protect_scale").as_double();
    params_.capacitor_low_scale =
        node_->get_parameter("nmpc.capacitor_limit.low_scale").as_double();
    params_.capacitor_filter_alpha =
        node_->get_parameter("nmpc.capacitor_limit.filter_alpha").as_double();
    params_.capacitor_v_low = std::max(0.0, params_.capacitor_v_low);
    params_.capacitor_v_safe = std::max(params_.capacitor_v_low + 0.1,
                                        params_.capacitor_v_safe);
    params_.capacitor_low_scale = std::clamp(params_.capacitor_low_scale, 0.0, 1.0);
    params_.capacitor_protect_scale =
        std::clamp(params_.capacitor_protect_scale, params_.capacitor_low_scale, 1.0);
    params_.capacitor_filter_alpha =
        std::clamp(params_.capacitor_filter_alpha, 0.0, 1.0);

    params_.Q_position = node_->get_parameter("nmpc.Q_position").as_double();
    params_.Q_orientation = node_->get_parameter("nmpc.Q_orientation").as_double();
    params_.Q_velocity = node_->get_parameter("nmpc.Q_velocity").as_double();
    params_.Q_omega = node_->get_parameter("nmpc.Q_omega").as_double();
    params_.R_linear = node_->get_parameter("nmpc.R_linear").as_double();
    params_.R_angular = node_->get_parameter("nmpc.R_angular").as_double();

    params_.esdf_weight = node_->get_parameter("nmpc.esdf_weight").as_double();
    params_.esdf_safe_dist = node_->get_parameter("nmpc.esdf_safe_dist").as_double();
    params_.contouring_weight = node_->get_parameter("nmpc.contouring_weight").as_double();

    params_.terminal_multiplier = node_->get_parameter("nmpc.terminal_multiplier").as_double();

    params_.enable_esdf_cost = node_->get_parameter("nmpc.enable_esdf_cost").as_bool();
    params_.near_weight_multiplier = node_->get_parameter("nmpc.near_weight_multiplier").as_double();
    
    bool publish_path = node_->get_parameter("nmpc.publish_predicted_path").as_bool();
    bool publish_speed_observation =
        node_->get_parameter("nmpc.publish_speed_observation").as_bool();
    
    // ========== 创建 acados solver ==========
    acados_ocp_capsule_ = wheelleg_nmpc_acados_create_capsule();
    int status = wheelleg_nmpc_acados_create(acados_ocp_capsule_);
    
    if (status != 0) {
        RCLCPP_ERROR(node_->get_logger(), "❌ NMPC: acados solver 创建失败");
        throw std::runtime_error("acados solver creation failed");
    }

    // 求解器离散参数由生成代码固化，这里从 solver 真值读取，避免 YAML 与实际不一致
    N_horizon_ = WHEELLEG_NMPC_N;
    {
        ocp_nlp_config* nlp_config = wheelleg_nmpc_acados_get_nlp_config(acados_ocp_capsule_);
        ocp_nlp_dims* nlp_dims = wheelleg_nmpc_acados_get_nlp_dims(acados_ocp_capsule_);
        ocp_nlp_in* nlp_in = wheelleg_nmpc_acados_get_nlp_in(acados_ocp_capsule_);
        double solver_dt = 0.0;
        ocp_nlp_in_get(nlp_config, nlp_dims, nlp_in, 0, "Ts", &solver_dt);
        if (solver_dt > 0.0) {
            T_horizon_ = solver_dt * static_cast<double>(N_horizon_);
        } else {
            RCLCPP_WARN(node_->get_logger(),
                "NMPC: 读取 solver Ts 失败，沿用默认 T_horizon=%.3f", T_horizon_);
        }
    }
    
    // 更新运行时参数到 solver
    updateNMPCParameters();
    
    // ========== 创建发布器 ==========
    if (publish_path) {
        predicted_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
            "nmpc/predicted_path", 10);
    }
    if (publish_speed_observation) {
        speed_observation_pub_ =
            node_->create_publisher<geometry_msgs::msg::TwistStamped>(
                "nmpc/speed_observation", 20);
    }
    
    // ========== 订阅里程计（来自 small_point_lio）==========
    // 用于获取真实速度反馈，实现闭环控制
    std::string odom_topic = node_->declare_parameter("nmpc.odom_topic", "/Odometry");
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(),
        std::bind(&NMPC::odomCallback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "NMPC: 订阅里程计话题 %s", odom_topic.c_str());

    std::string chassis_odom_topic =
        node_->declare_parameter("nmpc.chassis_odom_topic", "/Odometry");
    chassis_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        chassis_odom_topic, rclcpp::SensorDataQoS(),
        std::bind(&NMPC::chassisOdomCallback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(),
        "NMPC: 订阅底盘观测话题 %s (仅调试，不参与控制)",
        chassis_odom_topic.c_str());

    std::string default_chassis_feedback_topic =
        node_->declare_parameter("nmpc.capacitor_voltage_topic", "/ChassisOdom");
    std::string chassis_velocity_topic =
        node_->declare_parameter("nmpc.chassis_velocity_topic", default_chassis_feedback_topic);
    chassis_feedback_sub_ = node_->create_subscription<robots_msgs::msg::ChassisOdom>(
        chassis_velocity_topic, rclcpp::SensorDataQoS(),
        std::bind(&NMPC::chassisFeedbackCallback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(),
        "NMPC: ChassisOdom反馈订阅 %s, velocity_source=%s, timeout=%.2fs, vel_filter_alpha=%.2f",
        chassis_velocity_topic.c_str(),
        params_.velocity_feedback_source.c_str(),
        params_.chassis_velocity_timeout_sec,
        params_.chassis_velocity_filter_alpha);
    RCLCPP_INFO(node_->get_logger(),
        "NMPC: 电容限幅使用 %s, V_low=%.2f, V_safe=%.2f, scale=(%.2f/%.2f/1.00), hysteresis=%.2fV, filter_alpha=%.2f",
        chassis_velocity_topic.c_str(),
        params_.capacitor_v_low,
        params_.capacitor_v_safe,
        params_.capacitor_low_scale,
        params_.capacitor_protect_scale,
        kCapacitorHysteresisVoltage,
        params_.capacitor_filter_alpha);
    
    RCLCPP_INFO(node_->get_logger(), 
        "✓ NMPC Controller initialized (N=%d, T=%.2f s, dt=%.3f s)", 
        N_horizon_, T_horizon_, T_horizon_ / static_cast<double>(N_horizon_));
    RCLCPP_INFO(node_->get_logger(),
        "NMPC 权重: Qp=%.2f Qt=%.2f Qv=%.2f Qw=%.2f Rl=%.3f Ra=%.3f EsdfW=%.2f EsdfSafe=%.2f Contour=%.2f",
        params_.Q_position, params_.Q_orientation, params_.Q_velocity, params_.Q_omega,
        params_.R_linear, params_.R_angular,
        params_.esdf_weight, params_.esdf_safe_dist, params_.contouring_weight);
    RCLCPP_INFO(node_->get_logger(),
        "NMPC 二阶滞后: tau_v=%.3f s, tau_w=%.3f s, zeta_v=%.3f, zeta_w=%.3f",
        params_.vel_lag_tau, params_.omega_lag_tau,
        params_.vel_lag_zeta, params_.omega_lag_zeta);
    RCLCPP_INFO(node_->get_logger(),
        "NMPC speed_profile: enable=%d, v_cruise=%.2f, v_min=%.2f, ay_max=%.2f, kappa_eps=%.3f, window=%.2fm",
        params_.speed_profile_enable,
        params_.speed_profile_v_cruise,
        params_.speed_profile_v_min,
        params_.speed_profile_max_lateral_accel,
        params_.speed_profile_kappa_epsilon,
        params_.speed_profile_curvature_window_m);
    if (speed_observation_pub_) {
        RCLCPP_INFO(node_->get_logger(),
            "NMPC 观测发布: nmpc/speed_observation "
            "(lx=cmd_v, ly=chassis_v, lz=v_pred_1step, ax=a_cmd, ay=tau_v, az=v_cmd_pred_1step)");
    }

    // 参数热更新：支持运行中通过 `ros2 param set` 实时调参
    on_set_params_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&NMPC::onParametersChanged, this, std::placeholders::_1));
    
    initialized_ = true;
}

void NMPC::setPath(const nav_msgs::msg::Path& path) {
    if (path.poses.empty()) {
        global_path_.poses.clear();
        pivot_turn_active_ = false;
        pivot_turn_heading_error_ = 0.0;
        startup_pivot_phase_active_ = false;
        startup_align_active_ = false;
        startup_align_engaged_ = false;
        startup_align_heading_error_ = 0.0;
        path_remaining_dist_ = std::numeric_limits<double>::infinity();
        return;
    }
    
    // 保留完整路径用于后续处理
    global_path_ = path;
    nearest_idx_ = 0;
    pivot_turn_active_ = false;
    pivot_turn_heading_error_ = 0.0;
    startup_pivot_phase_active_ = true;
    startup_align_active_ = params_.startup_align_enable;
    startup_align_engaged_ = false;
    startup_align_heading_error_ = 0.0;
    
    RCLCPP_INFO(node_->get_logger(), 
        "NMPC: 接收新路径, %zu 个点", path.poses.size());
}

void NMPC::setMap(nav_core::MapInterface::Ptr map) {
    map_ = map;
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
    const double dt = T_horizon_ / static_cast<double>(std::max(1, N_horizon_));
    
    if (!initialized_ || global_path_.poses.empty()) {
        if (call_count % 50 == 1) {
            RCLCPP_WARN(node_->get_logger(), 
                "NMPC: 未初始化或无路径 (init=%d, path_size=%zu)", 
                initialized_, global_path_.poses.size());
        }
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        updateLastStateFromCommand(last_state_, 0.0, 0.0, dt);
        std::fill(last_control_.begin(), last_control_.end(), 0.0);
        predicted_stage1_valid_ = false;
        return nav_core::ControlResult::FAILED;
    }
    
    // 1. 提取当前状态 [x, y, theta, v, omega, a_v, alpha_w, v_cmd, omega_cmd]
    std::vector<double> x0(NX_STATE);
    x0[0] = current_pose.pose.position.x;
    x0[1] = current_pose.pose.position.y;
    x0[2] = tf2::getYaw(current_pose.pose.orientation);
    
    // 速度反馈融合：优先使用 ChassisOdom 实车速度；超时/无效时回退到 last_state_[3]/[4]。
    double measured_v = last_state_[3];
    double measured_w = last_state_[4];
    const bool has_measured_velocity = getMeasuredVelocity(measured_v, measured_w);
    const double velocity_feedback_alpha = params_.odom_feedback_alpha;
    x0[3] = velocity_feedback_alpha * measured_v +
        (1.0 - velocity_feedback_alpha) * last_state_[3];
    x0[4] = velocity_feedback_alpha * measured_w +
        (1.0 - velocity_feedback_alpha) * last_state_[4];
    x0[5] = last_state_[5];  // a_v
    x0[6] = last_state_[6];  // alpha_w
    x0[7] = last_state_[7];  // v_cmd
    x0[8] = last_state_[8];  // omega_cmd
    
    // 2. 检查是否到达目标（仅检查 xy 距离，不要求 yaw 对齐）
    const auto& goal = global_path_.poses.back().pose;
    double dx = goal.position.x - x0[0];
    double dy = goal.position.y - x0[1];
    double dist = std::hypot(dx, dy);
    
    if (dist < xy_tolerance_) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        updateLastStateFromCommand(x0, 0.0, 0.0, dt);
        std::fill(last_control_.begin(), last_control_.end(), 0.0);
        predicted_stage1_valid_ = false;
        RCLCPP_INFO(node_->get_logger(), "NMPC: 到达目标 (xy距离=%.3fm)", dist);
        return nav_core::ControlResult::SUCCEEDED;
    }

    if (applyStartupAlignmentGate(current_pose, cmd_vel, x0)) {
        return nav_core::ControlResult::RUNNING;
    }
    
    // 3. 提取局部参考轨迹
    auto yref_sequence = extractLocalReference(current_pose.pose, params_.horizon_length);
    
    // 验证参考轨迹质量
    if (yref_sequence.size() < static_cast<size_t>(N_horizon_ + 1)) {
        RCLCPP_WARN(node_->get_logger(), 
            "NMPC: 参考轨迹点不足 (%zu < %d)", 
            yref_sequence.size(), N_horizon_ + 1);
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        updateLastStateFromCommand(x0, 0.0, 0.0, dt);
        std::fill(last_control_.begin(), last_control_.end(), 0.0);
        predicted_stage1_valid_ = false;
        return nav_core::ControlResult::FAILED;
    }
    
    // 检查参考轨迹连续性（防止跳变）
    for (size_t i = 1; i < yref_sequence.size(); ++i) {
        double dx = yref_sequence[i][0] - yref_sequence[i-1][0];
        double dy = yref_sequence[i][1] - yref_sequence[i-1][1];
        double step_dist = std::hypot(dx, dy);
        
        // 单步距离不应超过 step_dist * 1.5（允许50%弹性，兼容路径稀疏段）
        double max_step = (params_.horizon_length / N_horizon_) * 1.5;
        if (step_dist > max_step) {
            RCLCPP_WARN(node_->get_logger(),
                "NMPC: 参考轨迹不连续 (step %zu: %.2f m > %.2f m)",
                i, step_dist, max_step);
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            updateLastStateFromCommand(x0, 0.0, 0.0, dt);
            std::fill(last_control_.begin(), last_control_.end(), 0.0);
            predicted_stage1_valid_ = false;
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
            updateLastStateFromCommand(x0, 0.0, 0.0, dt);
            std::fill(last_control_.begin(), last_control_.end(), 0.0);
            predicted_stage1_valid_ = false;
            return nav_core::ControlResult::FAILED;
        }
        
        // 平滑停车：逐渐减速而非突然停止
        double decay = 0.8;  // 每周期衰减到 80%
        cmd_vel.linear.x = last_state_[7] * decay;
        cmd_vel.angular.z = last_state_[8] * decay;
        if (!params_.allow_reverse) {
            cmd_vel.linear.x = std::max(0.0, cmd_vel.linear.x);
        }
        cmd_vel.linear.x = std::clamp(cmd_vel.linear.x,
                                      -params_.max_linear_vel,
                                      params_.max_linear_vel);
        cmd_vel.angular.z = std::clamp(cmd_vel.angular.z,
                                       -params_.max_angular_vel,
                                       params_.max_angular_vel);
        updateLastStateFromCommand(x0, cmd_vel.linear.x, cmd_vel.angular.z, dt);
        std::fill(last_control_.begin(), last_control_.end(), 0.0);
        predicted_stage1_valid_ = false;
        return nav_core::ControlResult::RUNNING;  // 继续尝试
    }
    
    // 求解成功，重置失败计数
    stats_.consecutive_failures = 0;
    
    // 5. 提取并发布预测轨迹（如果启用）
    if (predicted_path_pub_) {
        publishPredictedPath(current_pose.header, x0);
    }
    
    // 6. 应用控制
    // u_opt = [a_cmd, alpha_cmd] (加速度命令)
    double a_cmd = u_opt[0];
    double alpha_cmd = u_opt[1];
    double v_cmd = x0[7] + a_cmd * dt;
    double omega_cmd = x0[8] + alpha_cmd * dt;
    
    // 限幅
    if (!params_.allow_reverse) {
        v_cmd = std::max(0.0, v_cmd);
    }
    v_cmd = std::clamp(v_cmd, -params_.max_linear_vel, params_.max_linear_vel);
    omega_cmd = std::clamp(omega_cmd, -params_.max_angular_vel, params_.max_angular_vel);
    const double goal_v_limit = computeGoalApproachSpeedLimit(path_remaining_dist_);
    if (std::isfinite(goal_v_limit)) {
        v_cmd = std::min(v_cmd, goal_v_limit);
    }
    if (pivot_turn_active_) {
        v_cmd = 0.0;
    }
    
    cmd_vel.linear.x = v_cmd;
    cmd_vel.angular.z = omega_cmd;
    applyCapacitorOutputLimit(dt, cmd_vel);
    v_cmd = cmd_vel.linear.x;
    omega_cmd = cmd_vel.angular.z;

    if (startup_pivot_phase_active_) {
        const bool started_tracking =
            (std::abs(cmd_vel.linear.x) > 0.12) || (nearest_idx_ > 3);
        if (started_tracking) {
            startup_pivot_phase_active_ = false;
            RCLCPP_INFO(node_->get_logger(),
                "NMPC: 退出起步原地对齐阶段 (v=%.2f, nearest_idx=%d)",
                cmd_vel.linear.x, nearest_idx_);
        }
    }

    if (speed_observation_pub_) {
        geometry_msgs::msg::TwistStamped obs;
        obs.header = current_pose.header;
        if (obs.header.stamp.nanosec == 0 && obs.header.stamp.sec == 0) {
            obs.header.stamp = node_->now();
        }
        obs.header.frame_id = "base_link";
        obs.twist.linear.x = cmd_vel.linear.x;
        obs.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
        obs.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
        obs.twist.angular.x = a_cmd;
        obs.twist.angular.y = params_.vel_lag_tau;
        obs.twist.angular.z = std::numeric_limits<double>::quiet_NaN();

        if (has_measured_velocity) {
            obs.twist.linear.y = measured_v;
        }
        if (predicted_stage1_valid_) {
            obs.twist.linear.z = predicted_stage1_state_[3];
            obs.twist.angular.z = predicted_stage1_state_[7];
        }

        speed_observation_pub_->publish(obs);
    }

    updateLastStateFromCommand(x0, v_cmd, omega_cmd, dt);
    last_control_ = u_opt;
    
    // 发布调试信息
    if (stats_.solve_count % 20 == 0) {
        // RCLCPP_INFO(node_->get_logger(),
        //     "NMPC: v=%.2f m/s, ω=%.2f rad/s, solve_time=%.2f ms (avg=%.2f, max=%.2f)",
        //     v_cmd, omega_cmd, solve_time_ms, stats_.avg_solve_time_ms, stats_.max_solve_time_ms);
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
    chassis_odom_received_ = false;
    chassis_feedback_received_ = false;
    chassis_velocity_valid_ = false;
    chassis_filter_initialized_ = false;
    filtered_chassis_v_ = 0.0;
    filtered_chassis_w_ = 0.0;
    predicted_stage1_valid_ = false;
    path_remaining_dist_ = std::numeric_limits<double>::infinity();
    startup_pivot_phase_active_ = false;
    startup_align_active_ = false;
    startup_align_engaged_ = false;
    startup_align_heading_error_ = 0.0;
    capacitor_limit_level_ = CapacitorLimitLevel::NORMAL;
    capacitor_limit_scale_ = 1.0;
    
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

void NMPC::chassisOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_chassis_odom_ = *msg;
    chassis_odom_received_ = true;
}

void NMPC::chassisFeedbackCallback(const robots_msgs::msg::ChassisOdom::SharedPtr msg) {
    const double speed_x = static_cast<double>(msg->speed_x);
    const double speed_w = static_cast<double>(msg->speed_w);
    const double voltage = static_cast<double>(msg->capacitor_voltage);
    const double alpha = params_.chassis_velocity_filter_alpha;
    std::lock_guard<std::mutex> lock(odom_mutex_);

    latest_chassis_feedback_ = *msg;
    latest_chassis_feedback_stamp_ = node_->now();
    chassis_feedback_received_ = true;

    if (std::isfinite(speed_x) && std::isfinite(speed_w)) {
        chassis_velocity_valid_ = true;
        if (!chassis_filter_initialized_ || alpha >= 0.999) {
            filtered_chassis_v_ = speed_x;
            filtered_chassis_w_ = speed_w;
            chassis_filter_initialized_ = true;
        } else {
            filtered_chassis_v_ += alpha * (speed_x - filtered_chassis_v_);
            filtered_chassis_w_ += alpha * (speed_w - filtered_chassis_w_);
        }
    } else {
        chassis_velocity_valid_ = false;
    }

    if (std::isfinite(voltage) && voltage >= 0.0) {
        capacitor_voltage_raw_ = voltage;
        if (!capacitor_voltage_received_) {
            capacitor_voltage_filtered_ = voltage;
            capacitor_voltage_received_ = true;
            return;
        }
        capacitor_voltage_filtered_ +=
            params_.capacitor_filter_alpha * (voltage - capacitor_voltage_filtered_);
    }
}

bool NMPC::getMeasuredVelocity(double& v, double& w) {
    if (params_.velocity_feedback_source != "chassis_odom") {
        return false;
    }

    bool received = false;
    bool valid = false;
    bool stale = false;
    double age = 0.0;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        received = chassis_feedback_received_;
        valid = chassis_velocity_valid_;
        if (received) {
            age = (node_->now() - latest_chassis_feedback_stamp_).seconds();
            stale = !std::isfinite(age) || age < 0.0 ||
                age > params_.chassis_velocity_timeout_sec;
        }

        if (received && valid && !stale) {
            v = filtered_chassis_v_;
            w = filtered_chassis_w_;
            return true;
        }
    }

    if (!received) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "NMPC velocity feedback: 未收到 ChassisOdom，fallback 到 last_state");
    } else if (!valid) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "NMPC velocity feedback: ChassisOdom 速度无效，fallback 到 last_state");
    } else if (stale) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "NMPC velocity feedback stale (age=%.3fs > %.3fs), fallback 到 last_state",
            age, params_.chassis_velocity_timeout_sec);
    }
    return false;
}

void NMPC::updateLastStateFromCommand(
    const std::vector<double>& base_state,
    double v_cmd,
    double omega_cmd,
    double dt)
{
    if (last_state_.size() < NX_STATE) {
        last_state_.resize(NX_STATE, 0.0);
    }
    if (base_state.size() >= NX_STATE) {
        last_state_ = base_state;
    } else if (base_state.size() >= 7) {
        for (size_t i = 0; i < 5; ++i) {
            last_state_[i] = base_state[i];
        }
        last_state_[7] = base_state[5];
        last_state_[8] = base_state[6];
    }
    if (!std::isfinite(v_cmd)) {
        v_cmd = 0.0;
    }
    if (!std::isfinite(omega_cmd)) {
        omega_cmd = 0.0;
    }
    if (!std::isfinite(dt) || dt <= 0.0) {
        dt = T_horizon_ / static_cast<double>(std::max(1, N_horizon_));
    }

    // last_state_[3]/[4] 是实际速度估计，不能直接写成命令速度。
    const double tau_v = std::max(0.05, params_.vel_lag_tau);
    const double tau_w = std::max(0.05, params_.omega_lag_tau);
    const double zeta_v = std::max(0.05, params_.vel_lag_zeta);
    const double zeta_w = std::max(0.05, params_.omega_lag_zeta);
    double a_v = std::isfinite(last_state_[5]) ? last_state_[5] : 0.0;
    double alpha_w = std::isfinite(last_state_[6]) ? last_state_[6] : 0.0;

    const double a_v_dot =
        (v_cmd - last_state_[3] - 2.0 * zeta_v * tau_v * a_v) / (tau_v * tau_v);
    const double alpha_w_dot =
        (omega_cmd - last_state_[4] - 2.0 * zeta_w * tau_w * alpha_w) /
        (tau_w * tau_w);

    a_v += a_v_dot * dt;
    alpha_w += alpha_w_dot * dt;

    double v_est_next = last_state_[3] + a_v * dt;
    double w_est_next = last_state_[4] + alpha_w * dt;
    if (!std::isfinite(v_est_next)) {
        v_est_next = 0.0;
        a_v = 0.0;
    }
    if (!std::isfinite(w_est_next)) {
        w_est_next = 0.0;
        alpha_w = 0.0;
    }

    last_state_[3] = std::clamp(
        v_est_next, params_.min_state_linear_vel, params_.max_state_linear_vel);
    last_state_[4] = std::clamp(w_est_next, -params_.max_angular_vel, params_.max_angular_vel);
    last_state_[5] = std::isfinite(a_v) ? a_v : 0.0;
    last_state_[6] = std::isfinite(alpha_w) ? alpha_w : 0.0;
    last_state_[7] = v_cmd;
    last_state_[8] = omega_cmd;
}

double NMPC::getCapacitorLimitScale(double filtered_voltage) {
    const auto previous_level = capacitor_limit_level_;

    switch (capacitor_limit_level_) {
        case CapacitorLimitLevel::NORMAL:
            if (filtered_voltage < params_.capacitor_v_low) {
                capacitor_limit_level_ = CapacitorLimitLevel::LOW;
            } else if (filtered_voltage < params_.capacitor_v_safe) {
                capacitor_limit_level_ = CapacitorLimitLevel::PROTECT;
            }
            break;
        case CapacitorLimitLevel::PROTECT:
            if (filtered_voltage < params_.capacitor_v_low) {
                capacitor_limit_level_ = CapacitorLimitLevel::LOW;
            } else if (filtered_voltage > params_.capacitor_v_safe + kCapacitorHysteresisVoltage) {
                capacitor_limit_level_ = CapacitorLimitLevel::NORMAL;
            }
            break;
        case CapacitorLimitLevel::LOW:
            if (filtered_voltage > params_.capacitor_v_low + kCapacitorHysteresisVoltage) {
                capacitor_limit_level_ = CapacitorLimitLevel::PROTECT;
            }
            break;
    }

    switch (capacitor_limit_level_) {
        case CapacitorLimitLevel::NORMAL:
            capacitor_limit_scale_ = 1.0;
            break;
        case CapacitorLimitLevel::PROTECT:
            capacitor_limit_scale_ = params_.capacitor_protect_scale;
            break;
        case CapacitorLimitLevel::LOW:
            capacitor_limit_scale_ = params_.capacitor_low_scale;
            break;
    }

    if (previous_level != capacitor_limit_level_) {
        RCLCPP_INFO(
            node_->get_logger(),
            "NMPC 电容限幅档位切换: %d -> %d, Vcap=%.2fV, scale=%.2f",
            static_cast<int>(previous_level),
            static_cast<int>(capacitor_limit_level_),
            filtered_voltage,
            capacitor_limit_scale_);
    }

    return capacitor_limit_scale_;
}

double NMPC::applyCapacitorOutputLimit(
    double dt,
    geometry_msgs::msg::Twist& cmd_vel)
{
    double filtered_voltage = 0.0;
    bool voltage_received = false;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        filtered_voltage = capacitor_voltage_filtered_;
        voltage_received = capacitor_voltage_received_;
    }

    if (!voltage_received) {
        return 1.0;
    }
    if (!std::isfinite(dt) || dt <= 0.0) {
        dt = T_horizon_ / static_cast<double>(N_horizon_);
    }

    const double scale = getCapacitorLimitScale(filtered_voltage);
    if (scale >= 0.999) {
        return scale;
    }

    const double prev_v = last_state_.size() > 7 ? last_state_[7] : 0.0;
    const double prev_w = last_state_.size() > 8 ? last_state_[8] : 0.0;
    const double dv_limit = params_.max_linear_accel * scale * dt;
    const double dw_limit = params_.max_angular_accel * scale * dt;

    if (dv_limit >= 0.0) {
        cmd_vel.linear.x = prev_v + std::clamp(cmd_vel.linear.x - prev_v,
                                               -dv_limit, dv_limit);
    }
    if (dw_limit >= 0.0) {
        cmd_vel.angular.z = prev_w + std::clamp(cmd_vel.angular.z - prev_w,
                                                -dw_limit, dw_limit);
    }

    return scale;
}

rcl_interfaces::msg::SetParametersResult NMPC::onParametersChanged(
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";

    bool need_update_constraints = false;
    bool changed = false;

    for (const auto& p : parameters) {
        const auto& name = p.get_name();
        try {
            if (name == "nmpc.max_linear_vel") {
                params_.max_linear_vel = p.as_double();
                need_update_constraints = true;
                changed = true;
            } else if (name == "nmpc.max_state_linear_vel") {
                params_.max_state_linear_vel = p.as_double();
                need_update_constraints = true;
                changed = true;
            } else if (name == "nmpc.min_state_linear_vel") {
                params_.min_state_linear_vel = p.as_double();
                need_update_constraints = true;
                changed = true;
            } else if (name == "nmpc.max_angular_vel") {
                params_.max_angular_vel = p.as_double();
                need_update_constraints = true;
                changed = true;
            } else if (name == "nmpc.max_linear_accel") {
                params_.max_linear_accel = p.as_double();
                need_update_constraints = true;
                changed = true;
            } else if (name == "nmpc.max_linear_decel") {
                params_.max_linear_decel = p.as_double();
                need_update_constraints = true;
                changed = true;
            } else if (name == "nmpc.max_angular_accel") {
                params_.max_angular_accel = p.as_double();
                need_update_constraints = true;
                changed = true;
            } else if (name == "nmpc.allow_reverse") {
                params_.allow_reverse = p.as_bool();
                need_update_constraints = true;
                changed = true;
            } else if (name == "nmpc.horizon_length") {
                params_.horizon_length = p.as_double();
                changed = true;
            } else if (name == "nmpc.desired_velocity") {
                params_.desired_velocity = p.as_double();
                changed = true;
            } else if (name == "nmpc.use_omega_ref_from_path") {
                params_.use_omega_ref_from_path = p.as_bool();
                changed = true;
            } else if (name == "nmpc.goal_crawl_speed") {
                params_.goal_crawl_speed = p.as_double();
                changed = true;
            } else if (name == "nmpc.enable_goal_speed_limit") {
                params_.enable_goal_speed_limit = p.as_bool();
                changed = true;
            } else if (name == "nmpc.goal_slowdown_dist") {
                params_.goal_slowdown_dist = p.as_double();
                changed = true;
            } else if (name == "nmpc.goal_min_moving_speed") {
                params_.goal_min_moving_speed = p.as_double();
                changed = true;
            } else if (name == "nmpc.goal_max_slow_speed") {
                params_.goal_max_slow_speed = p.as_double();
                changed = true;
            } else if (name == "nmpc.pivot_turn_heading_thresh") {
                params_.pivot_turn_heading_thresh = p.as_double();
                changed = true;
            } else if (name == "nmpc.pivot_turn_startup_only") {
                params_.pivot_turn_startup_only = p.as_bool();
                changed = true;
            } else if (name == "nmpc.startup_align.enable") {
                params_.startup_align_enable = p.as_bool();
                changed = true;
            } else if (name == "nmpc.startup_align.enter_thresh") {
                params_.startup_align_enter_thresh = p.as_double();
                changed = true;
            } else if (name == "nmpc.startup_align.exit_thresh") {
                params_.startup_align_exit_thresh = p.as_double();
                changed = true;
            } else if (name == "nmpc.startup_align.lookahead") {
                params_.startup_align_lookahead = p.as_double();
                changed = true;
            } else if (name == "nmpc.startup_align.kp") {
                params_.startup_align_kp = p.as_double();
                changed = true;
            } else if (name == "nmpc.startup_align.min_angular_vel") {
                params_.startup_align_min_angular_vel = p.as_double();
                changed = true;
            } else if (name == "nmpc.startup_align.max_angular_vel") {
                params_.startup_align_max_angular_vel = p.as_double();
                changed = true;
            } else if (name == "nmpc.speed_profile.enable") {
                params_.speed_profile_enable = p.as_bool();
                changed = true;
            } else if (name == "nmpc.speed_profile.v_cruise") {
                params_.speed_profile_v_cruise = p.as_double();
                changed = true;
            } else if (name == "nmpc.speed_profile.v_min") {
                params_.speed_profile_v_min = p.as_double();
                changed = true;
            } else if (name == "nmpc.speed_profile.max_lateral_accel") {
                params_.speed_profile_max_lateral_accel = p.as_double();
                changed = true;
            } else if (name == "nmpc.speed_profile.kappa_epsilon") {
                params_.speed_profile_kappa_epsilon = p.as_double();
                changed = true;
            } else if (name == "nmpc.speed_profile.curvature_window_m") {
                params_.speed_profile_curvature_window_m = p.as_double();
                changed = true;
            } else if (name == "nmpc.enable_curvature_horizon_adapt") {
                params_.enable_curvature_horizon_adapt = p.as_bool();
                changed = true;
            } else if (name == "nmpc.horizon_kappa_scale") {
                params_.horizon_kappa_scale = p.as_double();
                changed = true;
            } else if (name == "nmpc.horizon_min_length") {
                params_.horizon_min_length = p.as_double();
                changed = true;
            } else if (name == "nmpc.odom_feedback_alpha") {
                params_.odom_feedback_alpha = p.as_double();
                changed = true;
            } else if (name == "nmpc.velocity_feedback_source") {
                params_.velocity_feedback_source = p.as_string();
                changed = true;
            } else if (name == "nmpc.chassis_velocity_timeout_sec") {
                params_.chassis_velocity_timeout_sec = p.as_double();
                changed = true;
            } else if (name == "nmpc.chassis_velocity_filter_alpha") {
                params_.chassis_velocity_filter_alpha = p.as_double();
                changed = true;
            } else if (name == "nmpc.vel_lag_tau") {
                params_.vel_lag_tau = p.as_double();
                changed = true;
            } else if (name == "nmpc.omega_lag_tau") {
                params_.omega_lag_tau = p.as_double();
                changed = true;
            } else if (name == "nmpc.vel_lag_zeta") {
                params_.vel_lag_zeta = p.as_double();
                changed = true;
            } else if (name == "nmpc.omega_lag_zeta") {
                params_.omega_lag_zeta = p.as_double();
                changed = true;
            } else if (name == "nmpc.capacitor_limit.v_safe") {
                params_.capacitor_v_safe = p.as_double();
                changed = true;
            } else if (name == "nmpc.capacitor_limit.v_low") {
                params_.capacitor_v_low = p.as_double();
                changed = true;
            } else if (name == "nmpc.capacitor_limit.protect_scale") {
                params_.capacitor_protect_scale = p.as_double();
                changed = true;
            } else if (name == "nmpc.capacitor_limit.low_scale") {
                params_.capacitor_low_scale = p.as_double();
                changed = true;
            } else if (name == "nmpc.capacitor_limit.filter_alpha") {
                params_.capacitor_filter_alpha = p.as_double();
                changed = true;
            } else if (name == "nmpc.Q_position") {
                params_.Q_position = p.as_double();
                changed = true;
            } else if (name == "nmpc.Q_orientation") {
                params_.Q_orientation = p.as_double();
                changed = true;
            } else if (name == "nmpc.Q_velocity") {
                params_.Q_velocity = p.as_double();
                changed = true;
            } else if (name == "nmpc.Q_omega") {
                params_.Q_omega = p.as_double();
                changed = true;
            } else if (name == "nmpc.R_linear") {
                params_.R_linear = p.as_double();
                changed = true;
            } else if (name == "nmpc.R_angular") {
                params_.R_angular = p.as_double();
                changed = true;
            } else if (name == "nmpc.esdf_weight") {
                params_.esdf_weight = p.as_double();
                changed = true;
            } else if (name == "nmpc.esdf_safe_dist") {
                params_.esdf_safe_dist = p.as_double();
                changed = true;
            } else if (name == "nmpc.contouring_weight") {
                params_.contouring_weight = p.as_double();
                changed = true;
            } else if (name == "nmpc.terminal_multiplier") {
                params_.terminal_multiplier = p.as_double();
                changed = true;
            } else if (name == "nmpc.enable_esdf_cost") {
                params_.enable_esdf_cost = p.as_bool();
                changed = true;
            } else if (name == "nmpc.near_weight_multiplier") {
                params_.near_weight_multiplier = p.as_double();
                changed = true;
            }
        } catch (const std::exception& e) {
            result.successful = false;
            result.reason = std::string("参数类型错误: ") + name + ", " + e.what();
            return result;
        }
    }

    if (!changed) {
        return result;
    }

    // 与 initialize() 中保持一致的参数防护
    params_.max_linear_vel = std::max(0.0, params_.max_linear_vel);
    params_.max_state_linear_vel =
        std::max(params_.max_linear_vel, params_.max_state_linear_vel);
    if (params_.min_state_linear_vel > params_.max_state_linear_vel) {
        params_.min_state_linear_vel = -0.3;
    }
    params_.max_angular_vel = std::max(0.0, params_.max_angular_vel);
    params_.max_linear_accel = std::max(0.0, params_.max_linear_accel);
    params_.max_linear_decel = std::max(0.0, params_.max_linear_decel);
    params_.max_angular_accel = std::max(0.0, params_.max_angular_accel);
    params_.odom_feedback_alpha = std::clamp(params_.odom_feedback_alpha, 0.0, 1.0);
    params_.chassis_velocity_timeout_sec =
        std::max(0.0, params_.chassis_velocity_timeout_sec);
    params_.chassis_velocity_filter_alpha =
        std::clamp(params_.chassis_velocity_filter_alpha, 0.0, 1.0);
    params_.goal_crawl_speed = std::max(0.0, params_.goal_crawl_speed);
    params_.goal_slowdown_dist = std::max(xy_tolerance_, params_.goal_slowdown_dist);
    params_.goal_min_moving_speed = std::max(0.0, params_.goal_min_moving_speed);
    params_.goal_max_slow_speed =
        std::max(params_.goal_min_moving_speed, params_.goal_max_slow_speed);
    params_.pivot_turn_heading_thresh = std::clamp(params_.pivot_turn_heading_thresh, 0.0, M_PI);
    params_.startup_align_enter_thresh =
        std::clamp(params_.startup_align_enter_thresh, 0.0, M_PI);
    params_.startup_align_exit_thresh =
        std::clamp(params_.startup_align_exit_thresh, 0.0, params_.startup_align_enter_thresh);
    params_.startup_align_lookahead = std::max(0.05, params_.startup_align_lookahead);
    params_.startup_align_kp = std::max(0.0, params_.startup_align_kp);
    params_.startup_align_min_angular_vel =
        std::max(0.0, params_.startup_align_min_angular_vel);
    params_.startup_align_max_angular_vel =
        std::max(params_.startup_align_min_angular_vel, params_.startup_align_max_angular_vel);
    params_.speed_profile_v_cruise = std::max(0.0, params_.speed_profile_v_cruise);
    params_.speed_profile_v_min = std::max(0.0, params_.speed_profile_v_min);
    params_.speed_profile_v_min =
        std::min(params_.speed_profile_v_min, params_.speed_profile_v_cruise);
    params_.speed_profile_max_lateral_accel =
        std::max(0.05, params_.speed_profile_max_lateral_accel);
    params_.speed_profile_kappa_epsilon = std::max(1e-4, params_.speed_profile_kappa_epsilon);
    params_.speed_profile_curvature_window_m =
        std::max(0.05, params_.speed_profile_curvature_window_m);
    params_.horizon_kappa_scale = std::max(0.0, params_.horizon_kappa_scale);
    params_.horizon_min_length = std::max(0.1, params_.horizon_min_length);
    params_.vel_lag_tau = std::max(0.05, params_.vel_lag_tau);
    params_.omega_lag_tau = std::max(0.05, params_.omega_lag_tau);
    params_.vel_lag_zeta = std::max(0.05, params_.vel_lag_zeta);
    params_.omega_lag_zeta = std::max(0.05, params_.omega_lag_zeta);
    params_.capacitor_v_low = std::max(0.0, params_.capacitor_v_low);
    params_.capacitor_v_safe = std::max(params_.capacitor_v_low + 0.1,
                                        params_.capacitor_v_safe);
    params_.capacitor_low_scale = std::clamp(params_.capacitor_low_scale, 0.0, 1.0);
    params_.capacitor_protect_scale =
        std::clamp(params_.capacitor_protect_scale, params_.capacitor_low_scale, 1.0);
    params_.capacitor_filter_alpha =
        std::clamp(params_.capacitor_filter_alpha, 0.0, 1.0);

    if (need_update_constraints && acados_ocp_capsule_) {
        updateNMPCParameters();
    }

    return result;
}

// ========== 私有方法实现 ==========

int NMPC::solveNMPC(
    const std::vector<double>& x0,
    const std::vector<std::vector<double>>& yref,
    std::vector<double>& u_opt)
{
    if (!acados_ocp_capsule_) return -1;
    
    // 设置初始状态约束
    double x0_array[NX_STATE] = {
        x0[0], x0[1], x0[2], x0[3], x0[4],
        x0[5], x0[6], x0[7], x0[8]
    };
    
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

        double x1[NX_STATE] = {0.0};
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", x1);
        for (int i = 0; i < NX_STATE; ++i) {
            predicted_stage1_state_[i] = x1[i];
        }
        predicted_stage1_valid_ = true;
    } else {
        predicted_stage1_valid_ = false;
    }
    
    return status;
}

bool NMPC::computeStartupAlignmentTargetYaw(
    const geometry_msgs::msg::Pose& current_pose,
    double& target_yaw)
{
    if (global_path_.poses.size() < 2) {
        return false;
    }

    const int nearest_idx = findNearestPathPoint(current_pose);
    const int start_idx = std::clamp(
        nearest_idx, 0, static_cast<int>(global_path_.poses.size()) - 1);
    const auto& start = global_path_.poses[start_idx].pose.position;

    double accum_dist = 0.0;
    int target_idx = start_idx;
    for (int i = start_idx; i < static_cast<int>(global_path_.poses.size()) - 1; ++i) {
        const auto& p1 = global_path_.poses[i].pose.position;
        const auto& p2 = global_path_.poses[i + 1].pose.position;
        const double seg_dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
        if (seg_dist < 1e-4) {
            continue;
        }
        accum_dist += seg_dist;
        target_idx = i + 1;
        if (accum_dist >= params_.startup_align_lookahead) {
            break;
        }
    }

    const auto& target = global_path_.poses[target_idx].pose.position;
    const double dx = target.x - start.x;
    const double dy = target.y - start.y;
    if (std::hypot(dx, dy) < 1e-4) {
        target_yaw = tf2::getYaw(global_path_.poses[start_idx].pose.orientation);
    } else {
        target_yaw = std::atan2(dy, dx);
    }
    nearest_idx_ = std::max(nearest_idx_, nearest_idx);
    return true;
}

bool NMPC::applyStartupAlignmentGate(
    const geometry_msgs::msg::PoseStamped& current_pose,
    geometry_msgs::msg::Twist& cmd_vel,
    const std::vector<double>& x0)
{
    if (!params_.startup_align_enable || !startup_align_active_) {
        return false;
    }

    double target_yaw = 0.0;
    if (!computeStartupAlignmentTargetYaw(current_pose.pose, target_yaw)) {
        startup_align_active_ = false;
        startup_align_engaged_ = false;
        return false;
    }

    const double current_yaw = tf2::getYaw(current_pose.pose.orientation);
    double heading_error = target_yaw - current_yaw;
    while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

    startup_align_heading_error_ = std::abs(heading_error);
    const double active_thresh = startup_align_engaged_
        ? params_.startup_align_exit_thresh
        : params_.startup_align_enter_thresh;

    if (startup_align_heading_error_ <= active_thresh) {
        startup_align_active_ = false;
        startup_align_engaged_ = false;
        RCLCPP_INFO(node_->get_logger(),
            "NMPC startup align: done, heading_err=%.1fdeg <= %.1fdeg",
            startup_align_heading_error_ * 180.0 / M_PI,
            active_thresh * 180.0 / M_PI);
        return false;
    }

    startup_align_engaged_ = true;

    double omega_cmd = params_.startup_align_kp * heading_error;
    const double max_w = std::min(
        params_.startup_align_max_angular_vel, params_.max_angular_vel);
    omega_cmd = std::clamp(omega_cmd, -max_w, max_w);
    if (std::abs(omega_cmd) < params_.startup_align_min_angular_vel &&
        startup_align_heading_error_ > params_.startup_align_exit_thresh) {
        omega_cmd = std::copysign(params_.startup_align_min_angular_vel, heading_error);
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = omega_cmd;
    const double dt = T_horizon_ / static_cast<double>(std::max(1, N_horizon_));
    applyCapacitorOutputLimit(dt, cmd_vel);
    omega_cmd = cmd_vel.angular.z;

    updateLastStateFromCommand(x0, cmd_vel.linear.x, omega_cmd, dt);
    last_control_[0] = 0.0;
    last_control_[1] = 0.0;
    predicted_stage1_valid_ = false;

    if (speed_observation_pub_) {
        geometry_msgs::msg::TwistStamped obs;
        obs.header = current_pose.header;
        if (obs.header.stamp.nanosec == 0 && obs.header.stamp.sec == 0) {
            obs.header.stamp = node_->now();
        }
        obs.header.frame_id = "base_link";
        obs.twist.linear.x = cmd_vel.linear.x;
        obs.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
        obs.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
        obs.twist.angular.x = 0.0;
        obs.twist.angular.y = params_.vel_lag_tau;
        obs.twist.angular.z = omega_cmd;

        double measured_v = 0.0;
        double measured_w = 0.0;
        if (getMeasuredVelocity(measured_v, measured_w)) {
            obs.twist.linear.y = measured_v;
        }
        speed_observation_pub_->publish(obs);
    }

    return true;
}

double NMPC::computeGoalApproachSpeedLimit(double path_remaining_dist) const {
    if (!params_.enable_goal_speed_limit) {
        return std::numeric_limits<double>::infinity();
    }
    if (!std::isfinite(path_remaining_dist)) {
        return std::numeric_limits<double>::infinity();
    }

    const double stop_dist = xy_tolerance_;
    const double slowdown_dist = std::max(stop_dist, params_.goal_slowdown_dist);
    if (path_remaining_dist >= slowdown_dist) {
        return std::numeric_limits<double>::infinity();
    }
    if (path_remaining_dist <= stop_dist) {
        return 0.0;
    }

    const double ratio =
        (path_remaining_dist - stop_dist) / std::max(1e-3, slowdown_dist - stop_dist);
    const double v_min = std::min(params_.goal_min_moving_speed, params_.goal_max_slow_speed);
    const double v_limit = v_min + std::clamp(ratio, 0.0, 1.0) *
        (params_.goal_max_slow_speed - v_min);
    return std::clamp(v_limit, 0.0, params_.max_linear_vel);
}

std::vector<std::vector<double>> NMPC::extractLocalReference(
    const geometry_msgs::msg::Pose& current_pose,
    double horizon_length)
{
    std::vector<std::vector<double>> yref;
    pivot_turn_active_ = false;
    pivot_turn_heading_error_ = 0.0;
    
    if (global_path_.poses.empty()) return yref;
    
    // 查找最近点并剪枝:只保留从最近点到终点的路径
    nearest_idx_ = findNearestPathPoint(current_pose);

    // 当离散最近点已经落到终点时，从机器人在最后一段上的投影位置开始采样。
    // 这样既保留最后一段的进场方向，也不会把倒数第二点作为后方参考。
    int reference_start_idx = nearest_idx_;
    double reference_start_ratio = 0.0;
    double reference_start_seg_dist = 0.0;
    if (global_path_.poses.size() >= 2 &&
        nearest_idx_ >= static_cast<int>(global_path_.poses.size()) - 1) {
        reference_start_idx = static_cast<int>(global_path_.poses.size()) - 2;
        const auto& p1 = global_path_.poses[reference_start_idx].pose.position;
        const auto& p2 = global_path_.poses[reference_start_idx + 1].pose.position;
        const double seg_x = p2.x - p1.x;
        const double seg_y = p2.y - p1.y;
        const double seg_len_sq = seg_x * seg_x + seg_y * seg_y;
        reference_start_seg_dist = std::sqrt(seg_len_sq);
        if (seg_len_sq > 1e-8) {
            reference_start_ratio = std::clamp(
                ((current_pose.position.x - p1.x) * seg_x +
                 (current_pose.position.y - p1.y) * seg_y) / seg_len_sq,
                0.0, 1.0);
        }
    }
    
    // 沿路径前向采样 N+1 个点 (在 odom 坐标系下)
    double dt = T_horizon_ / N_horizon_;
    
    // 计算路径后缀距离，用沿路径剩余距离做终点减速与限速。
    std::vector<double> suffix_dist(global_path_.poses.size(), 0.0);
    for (int k = static_cast<int>(global_path_.poses.size()) - 2; k >= 0; --k) {
        const auto& p1 = global_path_.poses[k].pose.position;
        const auto& p2 = global_path_.poses[k + 1].pose.position;
        suffix_dist[k] = suffix_dist[k + 1] + std::hypot(p2.x - p1.x, p2.y - p1.y);
    }
    path_remaining_dist_ = std::max(
        0.0,
        suffix_dist[std::clamp(
            reference_start_idx, 0, static_cast<int>(global_path_.poses.size()) - 1)] -
        reference_start_ratio * reference_start_seg_dist);
    // 超出路径末端的预测节点必须保持零速，避免参考速度在末端出现
    // “减速到 0 -> 重复终点又恢复爬行速度”的不连续。
    const double terminal_ref_v = 0.0;

    const double base_cruise_v = params_.speed_profile_enable
        ? params_.speed_profile_v_cruise
        : params_.desired_velocity;
    // 终点平滑减速（阶段三）：
    // 采用显式制动起点 S_brake，当 remaining_dist <= S_brake 时使用
    // v_ref_goal = sqrt(2 * a_d * remaining_dist)
    const double a_d = std::max(0.05, params_.max_linear_decel);
    const double s_brake = (base_cruise_v > 0.0)
        ? (base_cruise_v * base_cruise_v) / (2.0 * a_d)
        : 0.0;
    
    // 曲率自适应 horizon：弯道时缩短参考覆盖弧长，避免 solver 跨弯道求捷径
    double effective_horizon = params_.horizon_length;
    if (params_.enable_curvature_horizon_adapt) {
        double max_kappa = computeLocalMaxCurvature(nearest_idx_, 30);
        effective_horizon /= (1.0 + params_.horizon_kappa_scale * max_kappa);
        effective_horizon = std::max(params_.horizon_min_length, effective_horizon);
    }

    // 参考点位置间距：horizon 均匀分配到 N_horizon 个节点
    // 注意：step_dist 与 desired_velocity 完全解耦
    //   - step_dist 决定 solver 能"看到"多远的路径（空间前视距离）
    //   - desired_velocity 只影响每个节点的速度参考值
    const double step_dist = effective_horizon / N_horizon_;

    int last_found_idx = reference_start_idx;
    // 负偏移使 forward_dist=0 对应最后一段上的投影位置。
    double accumulated_dist =
        -reference_start_ratio * reference_start_seg_dist;
    
    const double robot_theta_now = tf2::getYaw(current_pose.orientation);

    for (int i = 0; i <= N_horizon_; ++i) {
        double forward_dist = step_dist * i;
        
        // 从上一个点继续搜索（保证单调性）
        int idx = last_found_idx;
        double dist = accumulated_dist;
        
        while (idx < static_cast<int>(global_path_.poses.size()) - 1) {
            auto& p1 = global_path_.poses[idx].pose.position;
            auto& p2 = global_path_.poses[idx + 1].pose.position;
            double seg_dist = std::hypot(p2.x - p1.x, p2.y - p1.y);

            // 路径中可能存在重复点，跳过零长度段以避免插值除零。
            if (seg_dist <= 1e-8) {
                idx++;
                continue;
            }
            
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
                
                // 终点平滑刹车：显式 S_brake + sqrt 刹车曲线
                double remaining_dist = std::max(0.0, suffix_dist[idx] - ratio * seg_dist);
                double desired_v = base_cruise_v;
                bool remaining_dist_stop = false;
                bool pivot_turn_stop = false;
                double v_curve_limit = params_.max_linear_vel;

                // 1) 远离制动区：维持巡航
                // 2) 进入制动区：v_ref_goal = sqrt(2 * a_d * remaining_dist)
                double v_limit = base_cruise_v;
                if (remaining_dist <= s_brake) {
                    v_limit = std::sqrt(std::max(0.0, 2.0 * a_d * remaining_dist));
                }
                desired_v = std::min(base_cruise_v, v_limit);

                // 3) 终点临界处理
                // 当距离小于 0.05m 时，直接给 0，消除末端积分漂移
                if (remaining_dist <= 0.05) {
                    desired_v = 0.0;
                    remaining_dist_stop = true;
                }

                const double goal_v_limit = computeGoalApproachSpeedLimit(remaining_dist);
                if (std::isfinite(goal_v_limit)) {
                    desired_v = std::min(desired_v, goal_v_limit);
                }

                // 大角度场景：按参数选择仅起步阶段或控制全程优先原地转向对齐
                double d_theta = theta - robot_theta_now;
                while (d_theta > M_PI) d_theta -= 2.0 * M_PI;
                while (d_theta < -M_PI) d_theta += 2.0 * M_PI;
                double heading_err = std::abs(d_theta);
                if (heading_err > params_.pivot_turn_heading_thresh) {
                    if (!params_.pivot_turn_startup_only || startup_pivot_phase_active_) {
                        desired_v = 0.0;
                        pivot_turn_stop = true;
                    }
                }
                if (i == 0) {
                    pivot_turn_active_ = pivot_turn_stop;
                    pivot_turn_heading_error_ = heading_err;
                }

                // Speed profile: 基于横向加速度上限的曲率限速（与几何前视解耦）
                if (params_.speed_profile_enable) {
                    double window_kappa = computeLocalMaxCurvatureByDistance(
                        idx, params_.speed_profile_curvature_window_m);
                    double seg_len = std::hypot(p2.x - p1.x, p2.y - p1.y);
                    if (window_kappa < 1e-6 && seg_len > 1e-3) {
                        window_kappa = std::abs(theta_diff) / seg_len;
                    }
                    double kappa_denom = std::max(window_kappa, params_.speed_profile_kappa_epsilon);
                    v_curve_limit = std::sqrt(params_.speed_profile_max_lateral_accel / kappa_denom);
                    desired_v = std::min(desired_v, v_curve_limit);
                }

                if (params_.speed_profile_enable && !pivot_turn_stop &&
                    !std::isfinite(goal_v_limit) &&
                    remaining_dist > s_brake) {
                    desired_v = std::max(params_.speed_profile_v_min, desired_v);
                }

                // 速度-几何解耦：
                // 不再使用 implicit_v = step_dist / dt 对 desired_v 做硬钳制，
                // 避免“前视长度同时决定几何与速度上限”的参数耦合。
                // 这里仅保留观测日志，便于评估速度参考与几何采样的一致性风险。
                const double implicit_v = step_dist / dt;
                if (i == 0 && desired_v > implicit_v + 1e-3) {
                    RCLCPP_DEBUG_THROTTLE(
                        node_->get_logger(), *node_->get_clock(), 1000,
                        "NMPC ref decoupled: desired_v(%.3f) > implicit_v(%.3f), "
                        "horizon=%.3f, step=%.4f, dt=%.4f",
                        desired_v, implicit_v, effective_horizon, step_dist, dt);
                }

                // 仍由现有运动约束参数兜底，保证参考速度不超过物理上限。
                desired_v = std::clamp(desired_v, 0.0, params_.max_linear_vel);

                // 角速度参考（默认关闭，避免离散路径噪声引入尖峰）
                double omega_ref = 0.0;
                if (params_.use_omega_ref_from_path && !yref.empty()) {
                    double dtheta = theta - yref.back()[2];
                    while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
                    while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
                    omega_ref = dtheta / dt;
                    omega_ref = std::clamp(omega_ref, -params_.max_angular_vel, params_.max_angular_vel);
                }

                yref.push_back({x, y, theta, desired_v, omega_ref});
                
                // 更新索引和累积距离
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
                terminal_ref_v,
                0.0
            });
            // 到达终点后，后续所有参考点都是终点
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
            terminal_ref_v,
            0.0
        });
    }
    
    return yref;
}

int NMPC::findNearestPathPoint(const geometry_msgs::msg::Pose& pose) {
    double min_dist = std::numeric_limits<double>::max();
    int nearest = nearest_idx_;
    bool used_global_relocalization = false;
    
    // 阶段1: 局部窗口搜索 (快速路径)
    int local_backward = 0;   // 默认不允许回退，抑制近终点索引抖动
    int local_forward = 30;   // 向前搜索30个点
    int monotonic_floor = std::max(0, nearest_idx_ - local_backward);
    int search_start = monotonic_floor;
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
        used_global_relocalization = true;
        
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
    
    if (used_global_relocalization) {
        return nearest;
    }
    return std::max(nearest_idx_, nearest);
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
    
    // ========== 1. 更新状态约束 (真实速度 + 命令速度) ==========
    double v_cmd_lower = params_.allow_reverse ? -params_.max_linear_vel : 0.0;
    for (int i = 1; i < N_horizon_; ++i) {
        double lbx[4] = {
            params_.min_state_linear_vel, -params_.max_angular_vel,
            v_cmd_lower, -params_.max_angular_vel
        };
        double ubx[4] = {
            params_.max_state_linear_vel, params_.max_angular_vel,
            params_.max_linear_vel, params_.max_angular_vel
        };
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubx", ubx);
    }
    
    // ========== 2. 更新控制约束 (加速度限制) ==========
    for (int i = 0; i < N_horizon_; ++i) {
    double lbu[2] = {-params_.max_linear_decel, -params_.max_angular_accel};
        double ubu[2] = {params_.max_linear_accel, params_.max_angular_accel};
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubu", ubu);
    }
    
    // RCLCPP_INFO(node_->get_logger(), 
    //     "NMPC 参数已更新: v_max=%.2f, ω_max=%.2f, a_max=%.2f, α_max=%.2f, tau_v=%.2f, tau_w=%.2f, reverse=%d, Qp=%.2f, Rl=%.3f",
    //     params_.max_linear_vel, params_.max_angular_vel,
    //     params_.max_linear_accel, params_.max_angular_accel,
    //     params_.vel_lag_tau, params_.omega_lag_tau,
    //     params_.allow_reverse,
    //     params_.Q_position, params_.R_linear);
}

void NMPC::injectEsdfParameters(const std::vector<std::vector<double>>& yref,
                              const std::vector<double>& theta_adjusted) {
    if (!acados_ocp_capsule_) return;

    // 参数布局（与 model.py self.params 顺序严格对应）:
    // [0..6]  xref: x, y, theta, v, omega, a, alpha
    // [7]     d_esdf at linearization point
    // [8..9]  x_esdf_lin, y_esdf_lin
    // [10..11] grad_esdf_x, grad_esdf_y
    // [12]    weight_scale
    // [13..15] q_pos, q_theta, q_vel
    // [16..17] r_lin, r_ang
    // [18..20] esdf_weight, esdf_safe_dist, contouring_weight
    // [21..22] vel_lag_tau, omega_lag_tau
    // [23..24] vel_lag_zeta, omega_lag_zeta
    // [25]    q_omega
    double p_default[NP_PARAM] = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        10.0, 0.0, 0.0, 0.0, 0.0,
        1.0,
        params_.Q_position, params_.Q_orientation, params_.Q_velocity,
        params_.R_linear, params_.R_angular,
        params_.esdf_weight, params_.esdf_safe_dist, params_.contouring_weight,
        params_.vel_lag_tau, params_.omega_lag_tau,
        params_.vel_lag_zeta, params_.omega_lag_zeta,
        params_.Q_omega
    };

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
        // 控制参考使用加速度前馈（由相邻速度参考差分得到）
        int idx_next = std::min(idx + 1, static_cast<int>(yref.size()) - 1);
        double dt_ref = std::max(1e-3, T_horizon_ / static_cast<double>(N_horizon_));
        double a_ref = (yref[idx_next][3] - yref[idx][3]) / dt_ref;
        double alpha_ref = (yref[idx_next][4] - yref[idx][4]) / dt_ref;
        p_values[5] = std::clamp(a_ref, -params_.max_linear_decel, params_.max_linear_accel);
        p_values[6] = std::clamp(alpha_ref, -params_.max_angular_accel, params_.max_angular_accel);

        // 近端/终端权重缩放
        double weight_scale = 1.0;
        if (i < near_end) {
            double ratio = static_cast<double>(near_end - i) / near_end;
            weight_scale = 1.0 + (params_.near_weight_multiplier - 1.0) * ratio;
        }
        if (i == N_horizon_) {
            weight_scale = params_.terminal_multiplier;
        }
        p_values[12] = weight_scale;

        // 运行时权重注入
        p_values[13] = params_.Q_position;
        p_values[14] = params_.Q_orientation;
        p_values[15] = params_.Q_velocity;
        p_values[16] = params_.R_linear;
        p_values[17] = params_.R_angular;
        p_values[18] = params_.esdf_weight;
        p_values[19] = params_.esdf_safe_dist;
        p_values[20] = params_.contouring_weight;
        p_values[21] = params_.vel_lag_tau;
        p_values[22] = params_.omega_lag_tau;
        p_values[23] = params_.vel_lag_zeta;
        p_values[24] = params_.omega_lag_zeta;
        p_values[25] = params_.Q_omega;

        // ESDF 查询：在参考点/上一轮预测点混合位置做一阶线性化
        double dist = 10.0;
        double qx = yref[idx][0];
        double qy = yref[idx][1];
        double gx = 0.0;
        double gy = 0.0;
        if (has_esdf && esdf) {
            double x_pred[NX_STATE] = {0};
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", x_pred);

            double alpha = (stats_.solve_count == 0) ? 1.0 : 0.7;
            qx = alpha * yref[idx][0] + (1.0 - alpha) * x_pred[0];
            qy = alpha * yref[idx][1] + (1.0 - alpha) * x_pred[1];

            if (esdf->getDistanceAndGradient(qx, qy, dist, gx, gy)) {
                const double grad_norm = std::hypot(gx, gy);
                if (grad_norm > 1e-6) {
                    gx /= grad_norm;
                    gy /= grad_norm;
                } else {
                    gx = 0.0;
                    gy = 0.0;
                }
            } else {
                dist = 10.0;
                gx = 0.0;
                gy = 0.0;
            }
        }
        p_values[7] = dist;
        p_values[8] = qx;
        p_values[9] = qy;
        p_values[10] = gx;
        p_values[11] = gy;

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
    
    // 提取预测状态 x = [x, y, theta, v, omega, a_v, alpha_w, v_cmd, omega_cmd]
    std::vector<double> x_pred(NX_STATE);
    
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

double NMPC::computeLocalMaxCurvature(int start_idx, int num_points) const {
    if (global_path_.poses.size() < 2) return 0.0;

    double max_kappa = 0.0;
    int end = std::min(start_idx + num_points,
                       static_cast<int>(global_path_.poses.size()) - 1);

    for (int k = start_idx; k < end; ++k) {
        const auto& p1 = global_path_.poses[k].pose;
        const auto& p2 = global_path_.poses[k + 1].pose;

        double dx = p2.position.x - p1.position.x;
        double dy = p2.position.y - p1.position.y;
        double seg_len = std::hypot(dx, dy);

        if (seg_len < 1e-4) continue;

        double t1 = tf2::getYaw(p1.orientation);
        double t2 = tf2::getYaw(p2.orientation);
        double dtheta = t2 - t1;
        while (dtheta >  M_PI) dtheta -= 2.0 * M_PI;
        while (dtheta < -M_PI) dtheta += 2.0 * M_PI;

        double kappa = std::abs(dtheta) / seg_len;
        max_kappa = std::max(max_kappa, kappa);
    }
    return max_kappa;
}

double NMPC::computeLocalMaxCurvatureByDistance(int start_idx, double window_dist_m) const {
    if (global_path_.poses.size() < 2 || window_dist_m <= 0.0) {
        return 0.0;
    }

    double max_kappa = 0.0;
    double accum_dist = 0.0;
    int begin = std::max(0, start_idx);
    int end = static_cast<int>(global_path_.poses.size()) - 1;

    for (int k = begin; k < end; ++k) {
        const auto& p1 = global_path_.poses[k].pose;
        const auto& p2 = global_path_.poses[k + 1].pose;
        double seg_len = std::hypot(
            p2.position.x - p1.position.x,
            p2.position.y - p1.position.y);

        if (seg_len < 1e-4) {
            continue;
        }

        double t1 = tf2::getYaw(p1.orientation);
        double t2 = tf2::getYaw(p2.orientation);
        double dtheta = t2 - t1;
        while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
        while (dtheta < -M_PI) dtheta += 2.0 * M_PI;

        max_kappa = std::max(max_kappa, std::abs(dtheta) / seg_len);
        accum_dist += seg_len;
        if (accum_dist >= window_dist_m) {
            break;
        }
    }

    return max_kappa;
}

}  // namespace nav_components
