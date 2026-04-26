// nav_components/src/stair_controller.cpp

#include "nav_components/stair_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace nav_components {

void StairController::initialize(rclcpp::Node* node) {
    node_ = node;

    auto declare_if_needed = [this](const std::string& name, const auto& v) {
        if (!node_->has_parameter(name)) {
            node_->declare_parameter(name, v);
        }
    };

    declare_if_needed("special_terrain.enable_stair_mode_detection", true);
    declare_if_needed("special_terrain.enable_fly_slope_mode_detection", true);
    declare_if_needed("special_terrain.enable_stair_fsm", true);
    declare_if_needed("special_terrain.stair_mode_trigger_distance_m", 1.0);
    declare_if_needed("special_terrain.stair_mode_lookahead_dist_m", 3.0);
    declare_if_needed("special_terrain.stair_mode_sample_step_m", 0.10);
    declare_if_needed("special_terrain.stair_mode_entry_heading_error_max_rad", 0.35);
    declare_if_needed("special_terrain.stair_mode_release_grace_cycles", 5);
    declare_if_needed("special_terrain.stair_mode_min_hold_sec", 0.35);
    declare_if_needed("special_terrain.stair_mode_reassert_block_sec", 0.0);
    declare_if_needed("special_terrain.stair_mode_force_release_distance_m", 2.5);
    declare_if_needed("special_terrain.stair_mode_max_assert_sec", 6.0);
    declare_if_needed("special_terrain.stair_mode_omega_limit_rad_s", 0.20);
    declare_if_needed("special_terrain.stair_mode_omega_slew_rate_rad_s2", 1.2);
    declare_if_needed("special_terrain.enable_stair_fixed_velocity_strategy", false);
    declare_if_needed("special_terrain.stair_fixed_velocity_trigger_distance_m", 0.35);
    declare_if_needed("special_terrain.stair_raise_leg_distance_m", 0.40);
    declare_if_needed("special_terrain.stair_fixed_linear_vel", 0.35);
    declare_if_needed("special_terrain.stair_level2_raise_leg_distance_m", 0.40);
    declare_if_needed("special_terrain.stair_level2_fixed_velocity_trigger_distance_m", 0.35);
    declare_if_needed("special_terrain.stair_level2_fixed_linear_vel", 0.35);
    declare_if_needed("special_terrain.stair_fixed_heading_kp", 1.8);
    declare_if_needed("special_terrain.stair_fixed_max_angular_vel", 0.8);
    declare_if_needed("special_terrain.stair_fixed_heading_deadband", 0.05);
    declare_if_needed("special_terrain.stair_contact_distance_m", 0.25);
    declare_if_needed("special_terrain.stair_commit_success_dist_m", 0.18);
    declare_if_needed("special_terrain.stair_verify_timeout_sec", 1.2);
    declare_if_needed("special_terrain.stair_progress_timeout_sec", 1.2);
    declare_if_needed("special_terrain.stair_progress_min_arc_m", 0.18);
    declare_if_needed("special_terrain.stair_fail_a_precontact_miss_cycles", 3);
    declare_if_needed("special_terrain.stair_backoff_distance_m", 0.60);
    declare_if_needed("special_terrain.stair_backoff_tangent_search_half_width_m", 0.30);
    declare_if_needed("special_terrain.stair_backoff_linear_vel", 0.30);
    declare_if_needed("special_terrain.stair_backoff_heading_kp", 2.0);
    declare_if_needed("special_terrain.stair_backoff_max_angular_vel", 1.0);
    declare_if_needed("special_terrain.stair_backoff_pos_tolerance_m", 0.08);
    declare_if_needed("special_terrain.stair_retry_max_attempts", 3);
    declare_if_needed("special_terrain.stair_request_recovery_on_max_attempts", true);
    declare_if_needed("special_terrain.enable_stair_cooldown", true);
    declare_if_needed("special_terrain.stair_cooldown_fail_threshold", 3);
    declare_if_needed("special_terrain.stair_cooldown_duration_sec", 30.0);

    declare_if_needed("special_terrain.fly_slope_mode_trigger_distance_m", 1.0);
    declare_if_needed("special_terrain.fly_slope_mode_lookahead_dist_m", 3.0);
    declare_if_needed("special_terrain.fly_slope_mode_sample_step_m", 0.10);
    declare_if_needed("special_terrain.fly_slope_mode_entry_heading_error_max_rad", 0.35);
    declare_if_needed("special_terrain.fly_slope_mode_release_grace_cycles", 5);
    declare_if_needed("special_terrain.fly_slope_mode_min_hold_sec", 0.35);
    declare_if_needed("special_terrain.fly_slope_mode_reassert_block_sec", 0.0);
    declare_if_needed("special_terrain.fly_slope_mode_force_release_distance_m", 2.5);
    declare_if_needed("special_terrain.fly_slope_mode_max_assert_sec", 6.0);
    declare_if_needed("special_terrain.fly_slope_mode_omega_limit_rad_s", 0.20);
    declare_if_needed("special_terrain.fly_slope_mode_omega_slew_rate_rad_s2", 1.2);
    declare_if_needed("special_terrain.fly_slope_pre_align_lateral_error_max_m", 0.08);
    declare_if_needed("special_terrain.fly_slope_pre_align_return_max_linear_vel", 0.35);
    declare_if_needed("special_terrain.fly_slope_pre_align_return_max_angular_vel", 1.0);
    declare_if_needed("special_terrain.enable_fly_slope_pre_align_turn_in_place", true);
    declare_if_needed("special_terrain.fly_slope_pre_align_heading_kp", 2.0);
    declare_if_needed("special_terrain.fly_slope_pre_align_max_angular_vel", 1.2);
    declare_if_needed("special_terrain.enable_fly_slope_fixed_velocity_strategy", false);
    declare_if_needed("special_terrain.fly_slope_fixed_velocity_trigger_distance_m", 0.35);
    declare_if_needed("special_terrain.fly_slope_raise_leg_distance_m", 0.40);
    declare_if_needed("special_terrain.fly_slope_fixed_linear_vel", 0.35);
    declare_if_needed("special_terrain.fly_slope_fixed_heading_kp", 1.8);
    declare_if_needed("special_terrain.fly_slope_fixed_max_angular_vel", 0.8);
    declare_if_needed("special_terrain.fly_slope_fixed_heading_deadband", 0.05);
    declare_if_needed("special_terrain.fly_slope_contact_distance_m", 0.25);
    declare_if_needed("special_terrain.fly_slope_commit_success_dist_m", 0.18);
    declare_if_needed("special_terrain.fly_slope_verify_timeout_sec", 1.2);
    declare_if_needed("special_terrain.fly_slope_progress_timeout_sec", 1.2);
    declare_if_needed("special_terrain.fly_slope_progress_min_arc_m", 0.18);
    declare_if_needed("special_terrain.fly_slope_fail_a_precontact_miss_cycles", 3);
    declare_if_needed("special_terrain.fly_slope_backoff_distance_m", 0.60);
    declare_if_needed("special_terrain.fly_slope_backoff_tangent_search_half_width_m", 0.30);
    declare_if_needed("special_terrain.fly_slope_backoff_linear_vel", 0.30);
    declare_if_needed("special_terrain.fly_slope_backoff_heading_kp", 2.0);
    declare_if_needed("special_terrain.fly_slope_backoff_max_angular_vel", 1.0);
    declare_if_needed("special_terrain.fly_slope_backoff_pos_tolerance_m", 0.08);
    declare_if_needed("special_terrain.fly_slope_retry_max_attempts", 3);
    declare_if_needed("special_terrain.fly_slope_request_recovery_on_max_attempts", true);
    declare_if_needed("special_terrain.enable_fly_slope_cooldown", true);
    declare_if_needed("special_terrain.fly_slope_cooldown_fail_threshold", 3);
    declare_if_needed("special_terrain.fly_slope_cooldown_duration_sec", 30.0);

    enable_stair_mode_detection_ =
        node_->get_parameter("special_terrain.enable_stair_mode_detection").as_bool();
    enable_fly_slope_mode_detection_ =
        node_->get_parameter("special_terrain.enable_fly_slope_mode_detection").as_bool();
    enable_stair_fsm_ =
        node_->get_parameter("special_terrain.enable_stair_fsm").as_bool();
    stair_mode_trigger_distance_m_ =
        node_->get_parameter("special_terrain.stair_mode_trigger_distance_m").as_double();
    stair_mode_lookahead_dist_m_ =
        node_->get_parameter("special_terrain.stair_mode_lookahead_dist_m").as_double();
    stair_mode_sample_step_m_ =
        node_->get_parameter("special_terrain.stair_mode_sample_step_m").as_double();
    stair_mode_entry_heading_error_max_rad_ =
        node_->get_parameter("special_terrain.stair_mode_entry_heading_error_max_rad").as_double();
    stair_mode_release_grace_cycles_ =
        node_->get_parameter("special_terrain.stair_mode_release_grace_cycles").as_int();
    stair_mode_min_hold_sec_ =
        node_->get_parameter("special_terrain.stair_mode_min_hold_sec").as_double();
    stair_mode_reassert_block_sec_ =
        node_->get_parameter("special_terrain.stair_mode_reassert_block_sec").as_double();
    stair_mode_force_release_distance_m_ =
        node_->get_parameter("special_terrain.stair_mode_force_release_distance_m").as_double();
    stair_mode_max_assert_sec_ =
        node_->get_parameter("special_terrain.stair_mode_max_assert_sec").as_double();
    stair_mode_omega_limit_rad_s_ =
        node_->get_parameter("special_terrain.stair_mode_omega_limit_rad_s").as_double();
    stair_mode_omega_slew_rate_rad_s2_ =
        node_->get_parameter("special_terrain.stair_mode_omega_slew_rate_rad_s2").as_double();
    enable_stair_fixed_velocity_strategy_ =
        node_->get_parameter("special_terrain.enable_stair_fixed_velocity_strategy").as_bool();
    stair_fixed_velocity_trigger_distance_m_ =
        node_->get_parameter("special_terrain.stair_fixed_velocity_trigger_distance_m").as_double();
    stair_raise_leg_distance_m_ =
        node_->get_parameter("special_terrain.stair_raise_leg_distance_m").as_double();
    stair_fixed_linear_vel_ =
        node_->get_parameter("special_terrain.stair_fixed_linear_vel").as_double();
    stair_level2_raise_leg_distance_m_ =
        node_->get_parameter("special_terrain.stair_level2_raise_leg_distance_m").as_double();
    stair_level2_fixed_velocity_trigger_distance_m_ =
        node_->get_parameter("special_terrain.stair_level2_fixed_velocity_trigger_distance_m").as_double();
    stair_level2_fixed_linear_vel_ =
        node_->get_parameter("special_terrain.stair_level2_fixed_linear_vel").as_double();
    stair_fixed_heading_kp_ =
        node_->get_parameter("special_terrain.stair_fixed_heading_kp").as_double();
    stair_fixed_max_angular_vel_ =
        node_->get_parameter("special_terrain.stair_fixed_max_angular_vel").as_double();
    stair_fixed_heading_deadband_ =
        node_->get_parameter("special_terrain.stair_fixed_heading_deadband").as_double();
    stair_contact_distance_m_ =
        node_->get_parameter("special_terrain.stair_contact_distance_m").as_double();
    stair_commit_success_dist_m_ =
        node_->get_parameter("special_terrain.stair_commit_success_dist_m").as_double();
    stair_verify_timeout_sec_ =
        node_->get_parameter("special_terrain.stair_verify_timeout_sec").as_double();
    stair_progress_timeout_sec_ =
        node_->get_parameter("special_terrain.stair_progress_timeout_sec").as_double();
    stair_progress_min_arc_m_ =
        node_->get_parameter("special_terrain.stair_progress_min_arc_m").as_double();
    stair_fail_a_precontact_miss_cycles_ =
        node_->get_parameter("special_terrain.stair_fail_a_precontact_miss_cycles").as_int();
    stair_backoff_distance_m_ =
        node_->get_parameter("special_terrain.stair_backoff_distance_m").as_double();
    stair_backoff_tangent_search_half_width_m_ =
        node_->get_parameter("special_terrain.stair_backoff_tangent_search_half_width_m").as_double();
    stair_backoff_linear_vel_ =
        node_->get_parameter("special_terrain.stair_backoff_linear_vel").as_double();
    stair_backoff_heading_kp_ =
        node_->get_parameter("special_terrain.stair_backoff_heading_kp").as_double();
    stair_backoff_max_angular_vel_ =
        node_->get_parameter("special_terrain.stair_backoff_max_angular_vel").as_double();
    stair_backoff_pos_tolerance_m_ =
        node_->get_parameter("special_terrain.stair_backoff_pos_tolerance_m").as_double();
    stair_retry_max_attempts_ =
        node_->get_parameter("special_terrain.stair_retry_max_attempts").as_int();
    stair_request_recovery_on_max_attempts_ =
        node_->get_parameter("special_terrain.stair_request_recovery_on_max_attempts").as_bool();
    enable_stair_cooldown_ =
        node_->get_parameter("special_terrain.enable_stair_cooldown").as_bool();
    stair_cooldown_fail_threshold_ =
        node_->get_parameter("special_terrain.stair_cooldown_fail_threshold").as_int();
    stair_cooldown_duration_sec_ =
        node_->get_parameter("special_terrain.stair_cooldown_duration_sec").as_double();

    fly_slope_mode_trigger_distance_m_ =
        node_->get_parameter("special_terrain.fly_slope_mode_trigger_distance_m").as_double();
    fly_slope_mode_lookahead_dist_m_ =
        node_->get_parameter("special_terrain.fly_slope_mode_lookahead_dist_m").as_double();
    fly_slope_mode_sample_step_m_ =
        node_->get_parameter("special_terrain.fly_slope_mode_sample_step_m").as_double();
    fly_slope_mode_entry_heading_error_max_rad_ =
        node_->get_parameter("special_terrain.fly_slope_mode_entry_heading_error_max_rad").as_double();
    fly_slope_mode_release_grace_cycles_ =
        node_->get_parameter("special_terrain.fly_slope_mode_release_grace_cycles").as_int();
    fly_slope_mode_min_hold_sec_ =
        node_->get_parameter("special_terrain.fly_slope_mode_min_hold_sec").as_double();
    fly_slope_mode_reassert_block_sec_ =
        node_->get_parameter("special_terrain.fly_slope_mode_reassert_block_sec").as_double();
    fly_slope_mode_force_release_distance_m_ =
        node_->get_parameter("special_terrain.fly_slope_mode_force_release_distance_m").as_double();
    fly_slope_mode_max_assert_sec_ =
        node_->get_parameter("special_terrain.fly_slope_mode_max_assert_sec").as_double();
    fly_slope_mode_omega_limit_rad_s_ =
        node_->get_parameter("special_terrain.fly_slope_mode_omega_limit_rad_s").as_double();
    fly_slope_mode_omega_slew_rate_rad_s2_ =
        node_->get_parameter("special_terrain.fly_slope_mode_omega_slew_rate_rad_s2").as_double();
    fly_slope_pre_align_lateral_error_max_m_ =
        node_->get_parameter("special_terrain.fly_slope_pre_align_lateral_error_max_m").as_double();
    fly_slope_pre_align_return_max_linear_vel_ =
        node_->get_parameter("special_terrain.fly_slope_pre_align_return_max_linear_vel").as_double();
    fly_slope_pre_align_return_max_angular_vel_ =
        node_->get_parameter("special_terrain.fly_slope_pre_align_return_max_angular_vel").as_double();
    fly_slope_pre_align_lateral_error_max_m_ =
        std::max(0.0, fly_slope_pre_align_lateral_error_max_m_);
    fly_slope_pre_align_return_max_linear_vel_ =
        std::max(0.0, fly_slope_pre_align_return_max_linear_vel_);
    fly_slope_pre_align_return_max_angular_vel_ =
        std::max(0.0, fly_slope_pre_align_return_max_angular_vel_);
    enable_fly_slope_pre_align_turn_in_place_ =
        node_->get_parameter("special_terrain.enable_fly_slope_pre_align_turn_in_place").as_bool();
    fly_slope_pre_align_heading_kp_ =
        node_->get_parameter("special_terrain.fly_slope_pre_align_heading_kp").as_double();
    fly_slope_pre_align_max_angular_vel_ =
        node_->get_parameter("special_terrain.fly_slope_pre_align_max_angular_vel").as_double();
    fly_slope_pre_align_max_angular_vel_ =
        std::max(0.0, fly_slope_pre_align_max_angular_vel_);
    enable_fly_slope_fixed_velocity_strategy_ =
        node_->get_parameter("special_terrain.enable_fly_slope_fixed_velocity_strategy").as_bool();
    fly_slope_fixed_velocity_trigger_distance_m_ =
        node_->get_parameter("special_terrain.fly_slope_fixed_velocity_trigger_distance_m").as_double();
    fly_slope_raise_leg_distance_m_ =
        node_->get_parameter("special_terrain.fly_slope_raise_leg_distance_m").as_double();
    fly_slope_fixed_linear_vel_ =
        node_->get_parameter("special_terrain.fly_slope_fixed_linear_vel").as_double();
    fly_slope_fixed_heading_kp_ =
        node_->get_parameter("special_terrain.fly_slope_fixed_heading_kp").as_double();
    fly_slope_fixed_max_angular_vel_ =
        node_->get_parameter("special_terrain.fly_slope_fixed_max_angular_vel").as_double();
    fly_slope_fixed_heading_deadband_ =
        node_->get_parameter("special_terrain.fly_slope_fixed_heading_deadband").as_double();
    fly_slope_contact_distance_m_ =
        node_->get_parameter("special_terrain.fly_slope_contact_distance_m").as_double();
    fly_slope_commit_success_dist_m_ =
        node_->get_parameter("special_terrain.fly_slope_commit_success_dist_m").as_double();
    fly_slope_verify_timeout_sec_ =
        node_->get_parameter("special_terrain.fly_slope_verify_timeout_sec").as_double();
    fly_slope_progress_timeout_sec_ =
        node_->get_parameter("special_terrain.fly_slope_progress_timeout_sec").as_double();
    fly_slope_progress_min_arc_m_ =
        node_->get_parameter("special_terrain.fly_slope_progress_min_arc_m").as_double();
    fly_slope_fail_a_precontact_miss_cycles_ =
        node_->get_parameter("special_terrain.fly_slope_fail_a_precontact_miss_cycles").as_int();
    fly_slope_backoff_distance_m_ =
        node_->get_parameter("special_terrain.fly_slope_backoff_distance_m").as_double();
    fly_slope_backoff_tangent_search_half_width_m_ =
        node_->get_parameter("special_terrain.fly_slope_backoff_tangent_search_half_width_m").as_double();
    fly_slope_backoff_linear_vel_ =
        node_->get_parameter("special_terrain.fly_slope_backoff_linear_vel").as_double();
    fly_slope_backoff_heading_kp_ =
        node_->get_parameter("special_terrain.fly_slope_backoff_heading_kp").as_double();
    fly_slope_backoff_max_angular_vel_ =
        node_->get_parameter("special_terrain.fly_slope_backoff_max_angular_vel").as_double();
    fly_slope_backoff_pos_tolerance_m_ =
        node_->get_parameter("special_terrain.fly_slope_backoff_pos_tolerance_m").as_double();
    fly_slope_retry_max_attempts_ =
        node_->get_parameter("special_terrain.fly_slope_retry_max_attempts").as_int();
    fly_slope_request_recovery_on_max_attempts_ =
        node_->get_parameter("special_terrain.fly_slope_request_recovery_on_max_attempts").as_bool();
    enable_fly_slope_cooldown_ =
        node_->get_parameter("special_terrain.enable_fly_slope_cooldown").as_bool();
    fly_slope_cooldown_fail_threshold_ =
        node_->get_parameter("special_terrain.fly_slope_cooldown_fail_threshold").as_int();
    fly_slope_cooldown_duration_sec_ =
        node_->get_parameter("special_terrain.fly_slope_cooldown_duration_sec").as_double();

    stair_mode_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("stair_mode", 10);
    stair_fsm_state_pub_ = node_->create_publisher<std_msgs::msg::String>("stair_fsm_state", 10);
    stair_transition_debug_pub_ = node_->create_publisher<std_msgs::msg::String>("stair_attempt_debug", 10);
    stair_cooldown_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "stair/cooldown_markers", rclcpp::QoS(1).transient_local().reliable());

    if (enable_stair_mode_detection_) {
        RCLCPP_INFO(node_->get_logger(),
                    "stair_mode 检测启用: trigger=%.2fm, lookahead=%.2fm, sample=%.2fm, uphill(dot>=0), entry_heading_err_max=%.2f rad, grace=%d, min_hold=%.2fs, reassert_block=%.2fs, force_release_dist=%.2fm, max_assert=%.2fs, omega_limit=%.2f rad/s, omega_slew=%.2f rad/s^2",
                    stair_mode_trigger_distance_m_,
                    stair_mode_lookahead_dist_m_,
                    stair_mode_sample_step_m_,
                    stair_mode_entry_heading_error_max_rad_,
                    stair_mode_release_grace_cycles_,
                    stair_mode_min_hold_sec_,
                    stair_mode_reassert_block_sec_,
                    stair_mode_force_release_distance_m_,
                    stair_mode_max_assert_sec_,
                    stair_mode_omega_limit_rad_s_,
                    stair_mode_omega_slew_rate_rad_s2_);
    }
    if (enable_stair_fixed_velocity_strategy_) {
        RCLCPP_INFO(node_->get_logger(),
                    "stair_mode 固定速度策略启用: trigger=%.2fm, v=%.2f m/s, level2(trigger=%.2fm,v=%.2f m/s), kp=%.2f, wz_max=%.2f rad/s, deadband=%.3f rad",
                    stair_fixed_velocity_trigger_distance_m_,
                    stair_fixed_linear_vel_,
                    stair_level2_fixed_velocity_trigger_distance_m_,
                    stair_level2_fixed_linear_vel_,
                    stair_fixed_heading_kp_,
                    stair_fixed_max_angular_vel_,
                    stair_fixed_heading_deadband_);
    }
    if (enable_stair_fsm_) {
        RCLCPP_INFO(node_->get_logger(),
                    "stair_fsm 启用: contact=%.2fm, success=%.2fm, verify_to=%.2fs, progress_to=%.2fs, progress_min=%.2fm, failA_miss=%d, backoff=%.2fm, retry_max=%d, cooldown(en=%d,th=%d,dur=%.1fs)",
                    stair_contact_distance_m_,
                    stair_commit_success_dist_m_,
                    stair_verify_timeout_sec_,
                    stair_progress_timeout_sec_,
                    stair_progress_min_arc_m_,
                    stair_fail_a_precontact_miss_cycles_,
                    stair_backoff_distance_m_,
                    stair_retry_max_attempts_,
                    enable_stair_cooldown_,
                    stair_cooldown_fail_threshold_,
                    stair_cooldown_duration_sec_);
    }

    if (enable_fly_slope_mode_detection_) {
        RCLCPP_INFO(node_->get_logger(),
                    "fly_slope_mode 检测启用: trigger=%.2fm, lookahead=%.2fm, sample=%.2fm, entry_heading_err_max=%.2f rad, pre_align_lateral_max=%.2fm, return(v_max=%.2f,wz_max=%.2f), pre_align_turn(en=%d,kp=%.2f,wz_max=%.2f)",
                    fly_slope_mode_trigger_distance_m_,
                    fly_slope_mode_lookahead_dist_m_,
                    fly_slope_mode_sample_step_m_,
                    fly_slope_mode_entry_heading_error_max_rad_,
                    fly_slope_pre_align_lateral_error_max_m_,
                    fly_slope_pre_align_return_max_linear_vel_,
                    fly_slope_pre_align_return_max_angular_vel_,
                    enable_fly_slope_pre_align_turn_in_place_,
                    fly_slope_pre_align_heading_kp_,
                    fly_slope_pre_align_max_angular_vel_);
    }
}

void StairController::setMap(nav_core::MapInterface::Ptr map) {
    nav_core::SpecialTerrainController::setMap(map);
    map_manager_ = std::dynamic_pointer_cast<LayeredMapManager>(map_);
}

void StairController::onNavStateChanged(nav_core::NavState state) {
    if (state != nav_core::NavState::CONTROLLING) {
        publishStairMode(0, true, true);
        stair_mode_release_counter_ = 0;
        fsm_state_ = StairFsmState::NORMAL;
        fsm_state_enter_time_ = std::chrono::steady_clock::time_point{};
    active_feature_ = TerrainCandidateInfo{};
    active_terrain_type_ = TerrainType::NONE;
        precontact_miss_counter_ = 0;
        commit_arc_start_m_ = 0.0;
        commit_progress_start_time_ = std::chrono::steady_clock::time_point{};
        cooldown_stair_id_ = -1;
        cooldown_replan_pending_ = false;
    }
}

nav_core::TerrainControlDecision StairController::update(
    const nav_core::TerrainControlContext& context,
    const geometry_msgs::msg::Twist& base_cmd,
    geometry_msgs::msg::Twist& out_cmd) {
    out_cmd = base_cmd;
    const auto now_tp = std::chrono::steady_clock::now();

    if ((!enable_stair_mode_detection_ && !enable_fly_slope_mode_detection_) ||
        !enable_stair_fsm_) {
        transitionFsmState(StairFsmState::NORMAL, "fsm disabled");
        publishStairMode(0, true, true);
        return nav_core::TerrainControlDecision::PASS_THROUGH;
    }

    if (!map_manager_ || context.current_path.poses.size() < 2) {
        transitionFsmState(StairFsmState::NORMAL, "map/path unavailable");
        publishStairMode(0, true, true);
        return nav_core::TerrainControlDecision::PASS_THROUGH;
    }

    TerrainCandidateInfo candidate;
    const bool has_candidate = queryUpcomingTerrainCandidate(context, active_terrain_type_, candidate);

    auto is_fly = [&](TerrainType tt) { return tt == TerrainType::FLY_SLOPE; };
    auto is_level2_stair = [&](TerrainType tt) { return tt == TerrainType::STAIR_LEVEL2; };
    auto mode_of = [&](TerrainType tt) -> uint8_t {
        return (tt == TerrainType::STAIR) ? 1 :
               ((tt == TerrainType::FLY_SLOPE) ? 2 :
                (is_level2_stair(tt) ? 3 : 0));
    };
    auto trigger_dist = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_mode_trigger_distance_m_ : stair_mode_trigger_distance_m_;
    };
    auto contact_dist = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_contact_distance_m_ : stair_contact_distance_m_;
    };
    auto raise_dist = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_raise_leg_distance_m_ :
               (is_level2_stair(tt) ? stair_level2_raise_leg_distance_m_
                                    : stair_raise_leg_distance_m_);
    };
    auto fix_vel_dist = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_fixed_velocity_trigger_distance_m_ :
               (is_level2_stair(tt) ? stair_level2_fixed_velocity_trigger_distance_m_
                                    : stair_fixed_velocity_trigger_distance_m_);
    };
    auto progress_min_arc = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_progress_min_arc_m_ : stair_progress_min_arc_m_;
    };
    auto progress_timeout = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_progress_timeout_sec_ : stair_progress_timeout_sec_;
    };
    auto commit_success_dist = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_commit_success_dist_m_ : stair_commit_success_dist_m_;
    };
    auto verify_timeout = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_verify_timeout_sec_ : stair_verify_timeout_sec_;
    };
    auto miss_cycles = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_fail_a_precontact_miss_cycles_
                          : stair_fail_a_precontact_miss_cycles_;
    };
    auto fixed_vel_enable = [&](TerrainType tt) {
        return is_fly(tt) ? enable_fly_slope_fixed_velocity_strategy_
                          : enable_stair_fixed_velocity_strategy_;
    };
    auto fixed_linear_vel = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_fixed_linear_vel_ :
               (is_level2_stair(tt) ? stair_level2_fixed_linear_vel_
                                    : stair_fixed_linear_vel_);
    };
    auto fixed_heading_kp = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_fixed_heading_kp_ : stair_fixed_heading_kp_;
    };
    auto fixed_wz_max = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_fixed_max_angular_vel_ : stair_fixed_max_angular_vel_;
    };
    auto fixed_heading_deadband = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_fixed_heading_deadband_ : stair_fixed_heading_deadband_;
    };
    auto backoff_distance = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_backoff_distance_m_ : stair_backoff_distance_m_;
    };
    auto backoff_tangent_half = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_backoff_tangent_search_half_width_m_
                          : stair_backoff_tangent_search_half_width_m_;
    };
    auto backoff_pos_tol = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_backoff_pos_tolerance_m_ : stair_backoff_pos_tolerance_m_;
    };
    auto backoff_heading_kp = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_backoff_heading_kp_ : stair_backoff_heading_kp_;
    };
    auto backoff_wz_max = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_backoff_max_angular_vel_ : stair_backoff_max_angular_vel_;
    };
    auto backoff_linear_vel = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_backoff_linear_vel_ : stair_backoff_linear_vel_;
    };
    auto retry_max = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_retry_max_attempts_ : stair_retry_max_attempts_;
    };
    auto recovery_on_max = [&](TerrainType tt) {
        return is_fly(tt) ? fly_slope_request_recovery_on_max_attempts_
                          : stair_request_recovery_on_max_attempts_;
    };

    bool candidate_in_cooldown = false;
    double candidate_cooldown_remain_sec = 0.0;
    if (has_candidate && candidate.feature_id >= 0) {
        if (candidate.terrain_type == TerrainType::FLY_SLOPE) {
            candidate_in_cooldown = isFlySlopeInCooldown(
                candidate.feature_id, now_tp, &candidate_cooldown_remain_sec);
        } else {
            candidate_in_cooldown = isStairInCooldown(
                candidate.feature_id, now_tp, &candidate_cooldown_remain_sec);
        }
    }

    double heading_err = 0.0;
    const bool has_heading_ref = queryHeadingErrorToPathNearRobot(context, heading_err);
    double lateral_err = 0.0;
    const bool has_lateral_ref = queryLateralErrorToPathNearRobot(context, lateral_err);
    const double entry_heading_error_max = is_fly(active_terrain_type_) ? 
        fly_slope_mode_entry_heading_error_max_rad_ : stair_mode_entry_heading_error_max_rad_;
    const bool heading_aligned = has_heading_ref &&
        (std::abs(heading_err) <= entry_heading_error_max);
    const bool fly_slope_lateral_aligned = has_lateral_ref &&
        (lateral_err <= fly_slope_pre_align_lateral_error_max_m_);

    auto enter_fail_backoff = [&](const char* reason) {
        ++active_attempt_count_;
        if (active_terrain_type_ == TerrainType::FLY_SLOPE) {
            onFlySlopeAttemptFailed(active_feature_.feature_id, now_tp);
        } else {
            onStairAttemptFailed(active_feature_.feature_id, now_tp);
        }
        transitionFsmState(StairFsmState::FAIL_RETRY_BACKOFF, reason);
    };

    nav_core::TerrainControlDecision decision = nav_core::TerrainControlDecision::PASS_THROUGH;

    switch (fsm_state_) {
        case StairFsmState::NORMAL: {
            publishStairMode(0, true);
            if (has_candidate && candidate.dist_to_feature_m <= trigger_dist(candidate.terrain_type)) {
                if (candidate_in_cooldown) {
                    cooldown_stair_id_ = candidate.feature_id;
                    active_terrain_type_ = candidate.terrain_type;
                    cooldown_replan_pending_ = true;
                    transitionFsmState(StairFsmState::COOLDOWN_BLOCKED,
                                       "candidate in cooldown");
                    decision = nav_core::TerrainControlDecision::REQUEST_REPLAN;
                    break;
                }
                active_feature_ = candidate;
                active_terrain_type_ = candidate.terrain_type;
                active_attempt_count_ = 0;
                precontact_miss_counter_ = 0;
                transitionFsmState(StairFsmState::PRE_ALIGN, "candidate in trigger window");
            }
            decision = nav_core::TerrainControlDecision::PASS_THROUGH;
            break;
        }

        case StairFsmState::PRE_ALIGN: {
            publishStairMode(0, true); // 抬腿动作已推迟到 COMMIT_ASCENT，此处只做预对准不发1

            if (!has_candidate) {
                transitionFsmState(StairFsmState::NORMAL, "candidate lost in pre-align");
                decision = nav_core::TerrainControlDecision::PASS_THROUGH;
                break;
            }

            if (candidate_in_cooldown) {
                cooldown_stair_id_ = candidate.feature_id;
                active_terrain_type_ = candidate.terrain_type;
                cooldown_replan_pending_ = true;
                transitionFsmState(StairFsmState::COOLDOWN_BLOCKED,
                                   "candidate entered cooldown in pre-align");
                decision = nav_core::TerrainControlDecision::REQUEST_REPLAN;
                break;
            }

            active_feature_ = candidate;
            active_terrain_type_ = candidate.terrain_type;

            // PRE_ALIGN 阶段尚未下令抬腿，因此不应再检查判据A短腿失败
            precontact_miss_counter_ = 0;

            if (active_terrain_type_ == TerrainType::FLY_SLOPE &&
                has_lateral_ref &&
                !fly_slope_lateral_aligned) {
                out_cmd = base_cmd;
                out_cmd.linear.x = std::clamp(
                    out_cmd.linear.x,
                    -fly_slope_pre_align_return_max_linear_vel_,
                    fly_slope_pre_align_return_max_linear_vel_);
                out_cmd.linear.y = 0.0;
                out_cmd.angular.z = std::clamp(
                    out_cmd.angular.z,
                    -fly_slope_pre_align_return_max_angular_vel_,
                    fly_slope_pre_align_return_max_angular_vel_);
                decision = nav_core::TerrainControlDecision::OVERRIDE_CMD;
                break;
            }

            if (active_terrain_type_ == TerrainType::FLY_SLOPE &&
                enable_fly_slope_pre_align_turn_in_place_ &&
                has_heading_ref &&
                !heading_aligned) {
                out_cmd.linear.x = 0.0;
                out_cmd.linear.y = 0.0;
                out_cmd.angular.z = std::clamp(
                    fly_slope_pre_align_heading_kp_ * heading_err,
                    -fly_slope_pre_align_max_angular_vel_,
                    fly_slope_pre_align_max_angular_vel_);
                decision = nav_core::TerrainControlDecision::OVERRIDE_CMD;
                break;
            }

            if (fsm_state_ == StairFsmState::PRE_ALIGN &&
                candidate.dist_to_feature_m <= contact_dist(active_terrain_type_) &&
                heading_aligned &&
                (!is_fly(active_terrain_type_) || fly_slope_lateral_aligned)) {
                transitionFsmState(StairFsmState::COMMIT_ASCENT,
                                   is_fly(active_terrain_type_)
                                       ? "contact + lateral + heading aligned"
                                       : "contact + heading aligned");
                precontact_miss_counter_ = 0;
                if (!computePathArcAtRobot(context, commit_arc_start_m_)) {
                    commit_arc_start_m_ = 0.0;
                }
                commit_progress_start_time_ = std::chrono::steady_clock::now();
            }

            decision = nav_core::TerrainControlDecision::PASS_THROUGH;
            break;
        }

        case StairFsmState::COMMIT_ASCENT: {
            bool do_raise = false;
            bool do_fix_vel = false;
            if (has_candidate &&
                active_feature_.feature_id == candidate.feature_id &&
                active_terrain_type_ == candidate.terrain_type) {
                do_raise = (candidate.dist_to_feature_m <= raise_dist(active_terrain_type_));
                do_fix_vel = (candidate.dist_to_feature_m <= fix_vel_dist(active_terrain_type_));
            } else {
                // 如果丢失候选（比如车体已经踩在台阶上导致盲区），默认保持动作生效
                do_raise = true;
                do_fix_vel = true;
            }
            publishStairMode(do_raise ? mode_of(active_terrain_type_) : 0, true);

            if (has_candidate) {
                if (candidate_in_cooldown) {
                    cooldown_stair_id_ = candidate.feature_id;
                    active_terrain_type_ = candidate.terrain_type;
                    cooldown_replan_pending_ = true;
                    transitionFsmState(StairFsmState::COOLDOWN_BLOCKED,
                                       "candidate entered cooldown in commit");
                    decision = nav_core::TerrainControlDecision::REQUEST_REPLAN;
                    break;
                }
                active_feature_ = candidate;
            }

            // 判据A：已触发抬腿且距离未到固定速度区间，若虚拟腿长连续为0(短腿)则判定失败
            if (do_raise && !do_fix_vel && virtual_leg_length_ == 0) {
                ++precontact_miss_counter_;
                if (precontact_miss_counter_ >= std::max(1, miss_cycles(active_terrain_type_))) {
                    enter_fail_backoff("criterion A: virtual short-leg in commit");
                }
            } else {
                precontact_miss_counter_ = 0;
            }

            if (fsm_state_ == StairFsmState::COMMIT_ASCENT) {
                double curr_arc = 0.0;
                if (computePathArcAtRobot(context, curr_arc)) {
                    const double arc_progress = curr_arc - commit_arc_start_m_;
                    if (arc_progress >= progress_min_arc(active_terrain_type_)) {
                        commit_arc_start_m_ = curr_arc;
                        commit_progress_start_time_ = std::chrono::steady_clock::now();
                    } else {
                        const double dt = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - commit_progress_start_time_).count();
                        if (dt > progress_timeout(active_terrain_type_)) {
                            enter_fail_backoff("criterion B: arc progress timeout");
                        }
                    }
                }
            }

            if (fsm_state_ == StairFsmState::COMMIT_ASCENT) {
                const Eigen::Vector2d robot_pos(
                    context.current_pose.pose.position.x,
                    context.current_pose.pose.position.y);
                const double signed_dist = (robot_pos - active_feature_.center).dot(active_feature_.normal);
                if (signed_dist >= commit_success_dist(active_terrain_type_)) {
                    transitionFsmState(StairFsmState::VERIFY_SUCCESS, "crossed centerline + success dist");
                }
            }

            if (fsm_state_ == StairFsmState::COMMIT_ASCENT) {
                if (do_fix_vel) {
                    if (fixed_vel_enable(active_terrain_type_)) {
                        out_cmd.linear.x = fixed_linear_vel(active_terrain_type_);
                    }
                    out_cmd.angular.z = 0.0;
                    double heading_ref = 0.0;
                    if (queryPathHeadingNearRobot(context, heading_ref)) {
                        tf2::Quaternion q;
                        tf2::fromMsg(context.current_pose.pose.orientation, q);
                        double roll, pitch, yaw;
                        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

                        double err = normalizeAngle(heading_ref - yaw);
                        if (std::abs(err) < fixed_heading_deadband(active_terrain_type_)) {
                            err = 0.0;
                        }
                        out_cmd.angular.z = std::clamp(
                            fixed_heading_kp(active_terrain_type_) * err,
                            -fixed_wz_max(active_terrain_type_),
                            fixed_wz_max(active_terrain_type_));
                    }
                    decision = nav_core::TerrainControlDecision::OVERRIDE_CMD;
                } else {
                    decision = nav_core::TerrainControlDecision::PASS_THROUGH;
                }
            }
            break;
        }

        case StairFsmState::VERIFY_SUCCESS: {
            publishStairMode(mode_of(active_terrain_type_), true);

            if (has_candidate) {
                active_feature_ = candidate;
            }

            const Eigen::Vector2d robot_pos(
                context.current_pose.pose.position.x,
                context.current_pose.pose.position.y);
            const double signed_dist = (robot_pos - active_feature_.center).dot(active_feature_.normal);
            if (signed_dist >= commit_success_dist(active_terrain_type_)) {
                if (active_terrain_type_ == TerrainType::FLY_SLOPE) {
                    onFlySlopeAttemptSucceeded(active_feature_.feature_id);
                } else {
                    onStairAttemptSucceeded(active_feature_.feature_id);
                }
                transitionFsmState(StairFsmState::NORMAL, "verify success");
                active_terrain_type_ = TerrainType::NONE;
                publishStairMode(0, true, true);
                decision = nav_core::TerrainControlDecision::PASS_THROUGH;
                break;
            }

            const double verify_dt = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - fsm_state_enter_time_).count();
            if (verify_dt > verify_timeout(active_terrain_type_)) {
                enter_fail_backoff("verify timeout");
            }

            if (fsm_state_ == StairFsmState::VERIFY_SUCCESS) {
                out_cmd.linear.x = 0.5 * fixed_linear_vel(active_terrain_type_);
                out_cmd.angular.z = 0.0;
                decision = nav_core::TerrainControlDecision::OVERRIDE_CMD;
            }
            break;
        }

        case StairFsmState::FAIL_RETRY_BACKOFF: {
            publishStairMode(0, true, true);

            if (active_feature_.normal.norm() < 1e-6 || active_feature_.tangent.norm() < 1e-6) {
                decision = nav_core::TerrainControlDecision::REQUEST_RECOVERY;
                break;
            }

            Eigen::Vector2d t = active_feature_.tangent.normalized();
            Eigen::Vector2d n = active_feature_.normal.normalized();
            const Eigen::Vector2d robot_pos(
                context.current_pose.pose.position.x,
                context.current_pose.pose.position.y);

            const double tangent_proj = (robot_pos - active_feature_.center).dot(t);
            const double max_tangent = active_feature_.half_length + backoff_tangent_half(active_terrain_type_);
            const double s_clamped = std::clamp(tangent_proj, -max_tangent, max_tangent);
            const Eigen::Vector2d centerline_point = active_feature_.center + s_clamped * t;
            const double signed_dist = (robot_pos - centerline_point).dot(n);
            const double target_signed = -backoff_distance(active_terrain_type_);

            if (signed_dist <= target_signed + backoff_pos_tol(active_terrain_type_)) {
                bool should_enter_cooldown = false;
                if (active_feature_.feature_id >= 0) {
                    if (active_terrain_type_ == TerrainType::FLY_SLOPE && enable_fly_slope_cooldown_) {
                        should_enter_cooldown = isFlySlopeInCooldown(active_feature_.feature_id, now_tp, nullptr);
                    } else if (active_terrain_type_ == TerrainType::STAIR && enable_stair_cooldown_) {
                        should_enter_cooldown = isStairInCooldown(active_feature_.feature_id, now_tp, nullptr);
                    }
                }

                if (should_enter_cooldown) {
                    cooldown_stair_id_ = active_feature_.feature_id;
                    cooldown_replan_pending_ = true;
                    transitionFsmState(StairFsmState::COOLDOWN_BLOCKED,
                                       "backoff done -> cooldown blocked");
                    decision = nav_core::TerrainControlDecision::REQUEST_REPLAN;
                } else if (active_attempt_count_ < std::max(1, retry_max(active_terrain_type_))) {
                    transitionFsmState(StairFsmState::PRE_ALIGN, "backoff done -> retry");
                    decision = nav_core::TerrainControlDecision::REQUEST_REPLAN; // 触发重规划以刷新实际距离
                } else {
                    if (recovery_on_max(active_terrain_type_)) {
                        decision = nav_core::TerrainControlDecision::REQUEST_RECOVERY;
                    } else {
                        decision = nav_core::TerrainControlDecision::REQUEST_REPLAN;
                    }
                    transitionFsmState(StairFsmState::NORMAL, "max retry reached");
                    active_terrain_type_ = TerrainType::NONE;
                }
                break;
            }

            const Eigen::Vector2d backoff_dir = -n;
            const double heading_ref = std::atan2(backoff_dir.y(), backoff_dir.x());
            tf2::Quaternion q;
            tf2::fromMsg(context.current_pose.pose.orientation, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            const double heading_err_backoff = normalizeAngle(heading_ref - yaw);
            out_cmd.angular.z = std::clamp(
                backoff_heading_kp(active_terrain_type_) * heading_err_backoff,
                -backoff_wz_max(active_terrain_type_),
                backoff_wz_max(active_terrain_type_));
            out_cmd.linear.x = backoff_linear_vel(active_terrain_type_) * std::max(0.0, std::cos(heading_err_backoff));
            decision = nav_core::TerrainControlDecision::OVERRIDE_CMD;
            break;
        }

        case StairFsmState::COOLDOWN_BLOCKED: {
            publishStairMode(0, true, true);

            if (cooldown_stair_id_ < 0) {
                transitionFsmState(StairFsmState::NORMAL, "cooldown stair invalid");
                cooldown_replan_pending_ = false;
                decision = nav_core::TerrainControlDecision::PASS_THROUGH;
                break;
            }

            double remain_sec = 0.0;
            const bool still_cooling = (active_terrain_type_ == TerrainType::FLY_SLOPE)
                ? isFlySlopeInCooldown(cooldown_stair_id_, now_tp, &remain_sec)
                : isStairInCooldown(cooldown_stair_id_, now_tp, &remain_sec);
            if (!still_cooling) {
                RCLCPP_INFO(node_->get_logger(),
                            "terrain_fsm cooldown cleared: type=%s id=%d",
                            terrainTypeName(active_terrain_type_),
                            cooldown_stair_id_);
                cooldown_stair_id_ = -1;
                cooldown_replan_pending_ = false;
                transitionFsmState(StairFsmState::NORMAL, "cooldown expired");
                active_terrain_type_ = TerrainType::NONE;
                decision = nav_core::TerrainControlDecision::PASS_THROUGH;
                break;
            }

            if (cooldown_replan_pending_) {
                cooldown_replan_pending_ = false;
                RCLCPP_WARN(node_->get_logger(),
                            "terrain_fsm cooldown blocked: type=%s id=%d, remain=%.2fs -> REQUEST_REPLAN",
                            terrainTypeName(active_terrain_type_), cooldown_stair_id_, remain_sec);
                decision = nav_core::TerrainControlDecision::REQUEST_REPLAN;
            } else {
                decision = nav_core::TerrainControlDecision::PASS_THROUGH;
            }
            break;
        }
    }

    applyStairModeOmegaLimit(out_cmd, context.control_rate_hz);
    syncRuntimeBlockedUphillStairs(now_tp);
    publishFsmDebug(now_tp);
    publishCooldownMarkers(now_tp);
    return decision;
}

double StairController::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void StairController::transitionFsmState(StairFsmState new_state, const char* reason) {
    if (fsm_state_ == new_state) {
        return;
    }
    const auto from_state = fsm_state_;
    last_transition_reason_ = (reason != nullptr) ? reason : "n/a";
    RCLCPP_INFO(node_->get_logger(), "stair_fsm: %s -> %s (%s)",
                fsmStateName(fsm_state_), fsmStateName(new_state), last_transition_reason_.c_str());

    if (stair_transition_debug_pub_) {
        std_msgs::msg::String msg;
        std::ostringstream oss;
        oss << "from=" << fsmStateName(from_state)
            << ",to=" << fsmStateName(new_state)
            << ",reason=" << last_transition_reason_
            << ",terrain=" << terrainTypeName(active_terrain_type_)
            << ",feature_id=" << active_feature_.feature_id
            << ",attempt=" << active_attempt_count_;
        msg.data = oss.str();
        stair_transition_debug_pub_->publish(msg);
    }

    fsm_state_ = new_state;
    fsm_state_enter_time_ = std::chrono::steady_clock::now();
}

const char* StairController::terrainTypeName(TerrainType terrain_type) {
    switch (terrain_type) {
        case TerrainType::NONE:
            return "NONE";
        case TerrainType::STAIR:
            return "STAIR";
        case TerrainType::FLY_SLOPE:
            return "FLY_SLOPE";
        case TerrainType::STAIR_LEVEL2:
            return "STAIR_LEVEL2";
    }
    return "UNKNOWN";
}

bool StairController::queryUpcomingTerrainCandidate(
    const nav_core::TerrainControlContext& context,
    TerrainType preferred_type,
    TerrainCandidateInfo& candidate) const {
    candidate = TerrainCandidateInfo{};

    TerrainCandidateInfo stair_candidate;
    TerrainCandidateInfo fly_candidate;
    const bool has_stair = enable_stair_mode_detection_ &&
        queryUpcomingStairCandidate(context, stair_candidate);
    const bool has_fly = enable_fly_slope_mode_detection_ &&
        queryUpcomingFlySlopeCandidate(context, fly_candidate);

    if ((preferred_type == TerrainType::STAIR ||
         preferred_type == TerrainType::STAIR_LEVEL2) && has_stair) {
        candidate = stair_candidate;
        return true;
    }
    if (preferred_type == TerrainType::FLY_SLOPE && has_fly) {
        candidate = fly_candidate;
        return true;
    }

    if (has_stair && has_fly) {
        candidate = (stair_candidate.dist_to_feature_m <= fly_candidate.dist_to_feature_m)
            ? stair_candidate
            : fly_candidate;
        return true;
    }
    if (has_stair) {
        candidate = stair_candidate;
        return true;
    }
    if (has_fly) {
        candidate = fly_candidate;
        return true;
    }
    return false;
}

bool StairController::queryUpcomingStairCandidate(
    const nav_core::TerrainControlContext& context,
    TerrainCandidateInfo& candidate) const {
    candidate = TerrainCandidateInfo{};
    if (!map_manager_ || context.current_path.poses.size() < 2) {
        return false;
    }

    const auto& poses = context.current_path.poses;
    const double sample_step = std::max(0.02, stair_mode_sample_step_m_);
    const double lookahead_dist = std::max(0.1, stair_mode_lookahead_dist_m_);

    size_t closest_idx = 0;
    double min_dist2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < poses.size(); ++i) {
        const auto& p = poses[i].pose.position;
        const double dx = context.current_pose.pose.position.x - p.x;
        const double dy = context.current_pose.pose.position.y - p.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < min_dist2) {
            min_dist2 = d2;
            closest_idx = i;
        }
    }

    double traveled = 0.0;
    for (size_t i = closest_idx; i + 1 < poses.size() && traveled <= lookahead_dist; ++i) {
        const auto& p0 = poses[i].pose.position;
        const auto& p1 = poses[i + 1].pose.position;
        const double seg_dx = p1.x - p0.x;
        const double seg_dy = p1.y - p0.y;
        const double seg_len = std::hypot(seg_dx, seg_dy);
        if (seg_len < 1e-6) {
            continue;
        }

        const double dir_x = seg_dx / seg_len;
        const double dir_y = seg_dy / seg_len;
        const int sample_count = std::max(1, static_cast<int>(std::ceil(seg_len / sample_step)));
        for (int s = 0; s <= sample_count; ++s) {
            const double t = static_cast<double>(s) / static_cast<double>(sample_count);
            const double local_dist = seg_len * t;
            const double path_dist = traveled + local_dist;
            if (path_dist > lookahead_dist) {
                break;
            }

            const double wx = p0.x + seg_dx * t;
            const double wy = p0.y + seg_dy * t;
            double nx = 0.0;
            double ny = 0.0;
            if (!map_manager_->getStairTraverseNormal(wx, wy, nx, ny)) {
                continue;
            }

            const double dot = dir_x * nx + dir_y * ny;
            if (dot < 0.0) {
                continue;
            }

            candidate.valid = true;
            candidate.terrain_type = TerrainType::STAIR;
            candidate.dist_to_feature_m = path_dist;
            candidate.normal = Eigen::Vector2d(nx, ny);
            if (candidate.normal.norm() > 1e-6) {
                candidate.normal.normalize();
            }
            candidate.tangent = Eigen::Vector2d(-candidate.normal.y(), candidate.normal.x());
            candidate.center = Eigen::Vector2d(wx, wy);
            candidate.half_length = 0.3;

            LayeredMapManager::StairPrimitive primitive;
            if (map_manager_->getStairPrimitiveAt(wx, wy, primitive)) {
                candidate.feature_id = primitive.stair_id;
                if (primitive.is_level2) {
                    candidate.terrain_type = TerrainType::STAIR_LEVEL2;
                }
                candidate.center = primitive.center;
                if (primitive.normal.norm() > 1e-6) {
                    candidate.normal = primitive.normal.normalized();
                    candidate.tangent = Eigen::Vector2d(-candidate.normal.y(), candidate.normal.x());
                }
                if (primitive.tangent.norm() > 1e-6) {
                    candidate.tangent = primitive.tangent.normalized();
                }
                candidate.half_length = std::max(0.05, primitive.half_length);
            }
            return true;
        }
        traveled += seg_len;
    }
    return false;
}

bool StairController::queryUpcomingFlySlopeCandidate(
    const nav_core::TerrainControlContext& context,
    TerrainCandidateInfo& candidate) const {
    candidate = TerrainCandidateInfo{};
    if (!map_manager_ || context.current_path.poses.size() < 2) {
        return false;
    }

    const auto& poses = context.current_path.poses;
    const double sample_step = std::max(0.02, fly_slope_mode_sample_step_m_);
    const double lookahead_dist = std::max(0.1, fly_slope_mode_lookahead_dist_m_);

    size_t closest_idx = 0;
    double min_dist2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < poses.size(); ++i) {
        const auto& p = poses[i].pose.position;
        const double dx = context.current_pose.pose.position.x - p.x;
        const double dy = context.current_pose.pose.position.y - p.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < min_dist2) {
            min_dist2 = d2;
            closest_idx = i;
        }
    }

    double traveled = 0.0;
    for (size_t i = closest_idx; i + 1 < poses.size() && traveled <= lookahead_dist; ++i) {
        const auto& p0 = poses[i].pose.position;
        const auto& p1 = poses[i + 1].pose.position;
        const double seg_dx = p1.x - p0.x;
        const double seg_dy = p1.y - p0.y;
        const double seg_len = std::hypot(seg_dx, seg_dy);
        if (seg_len < 1e-6) {
            continue;
        }

        const double dir_x = seg_dx / seg_len;
        const double dir_y = seg_dy / seg_len;
        const int sample_count = std::max(1, static_cast<int>(std::ceil(seg_len / sample_step)));
        for (int s = 0; s <= sample_count; ++s) {
            const double t = static_cast<double>(s) / static_cast<double>(sample_count);
            const double local_dist = seg_len * t;
            const double path_dist = traveled + local_dist;
            if (path_dist > lookahead_dist) {
                break;
            }

            const double wx = p0.x + seg_dx * t;
            const double wy = p0.y + seg_dy * t;
            double nx = 0.0;
            double ny = 0.0;
            if (!map_manager_->getFlySlopeTraverseNormal(wx, wy, nx, ny)) {
                continue;
            }

            const double dot = dir_x * nx + dir_y * ny;
            if (dot < 0.0) {
                continue;
            }

            candidate.valid = true;
            candidate.terrain_type = TerrainType::FLY_SLOPE;
            candidate.dist_to_feature_m = path_dist;
            candidate.normal = Eigen::Vector2d(nx, ny);
            if (candidate.normal.norm() > 1e-6) {
                candidate.normal.normalize();
            }
            candidate.tangent = Eigen::Vector2d(-candidate.normal.y(), candidate.normal.x());
            candidate.center = Eigen::Vector2d(wx, wy);
            candidate.half_length = 0.3;

            LayeredMapManager::FlySlopePrimitive primitive;
            if (map_manager_->getFlySlopePrimitiveAt(wx, wy, primitive)) {
                candidate.feature_id = primitive.fly_slope_id;
                candidate.center = primitive.center;
                if (primitive.normal.norm() > 1e-6) {
                    candidate.normal = primitive.normal.normalized();
                    candidate.tangent = Eigen::Vector2d(-candidate.normal.y(), candidate.normal.x());
                }
                if (primitive.tangent.norm() > 1e-6) {
                    candidate.tangent = primitive.tangent.normalized();
                }
                candidate.half_length = std::max(0.05, primitive.half_length);
            }
            return true;
        }
        traveled += seg_len;
    }
    return false;
}

bool StairController::computePathArcAtRobot(
    const nav_core::TerrainControlContext& context,
    double& arc_m) const {
    arc_m = 0.0;
    const auto& poses = context.current_path.poses;
    if (poses.size() < 2) {
        return false;
    }

    size_t best_seg = 0;
    double best_t = 0.0;
    double best_dist2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < poses.size(); ++i) {
        const Eigen::Vector2d a(poses[i].pose.position.x, poses[i].pose.position.y);
        const Eigen::Vector2d b(poses[i + 1].pose.position.x, poses[i + 1].pose.position.y);
        const Eigen::Vector2d p(context.current_pose.pose.position.x,
                                context.current_pose.pose.position.y);
        const Eigen::Vector2d ab = b - a;
        const double ab2 = ab.squaredNorm();
        if (ab2 < 1e-9) {
            continue;
        }
        const double t = std::clamp((p - a).dot(ab) / ab2, 0.0, 1.0);
        const Eigen::Vector2d proj = a + t * ab;
        const double d2 = (p - proj).squaredNorm();
        if (d2 < best_dist2) {
            best_dist2 = d2;
            best_seg = i;
            best_t = t;
        }
    }

    double acc = 0.0;
    for (size_t i = 0; i < best_seg; ++i) {
        const auto& p0 = poses[i].pose.position;
        const auto& p1 = poses[i + 1].pose.position;
        acc += std::hypot(p1.x - p0.x, p1.y - p0.y);
    }

    const auto& s0 = poses[best_seg].pose.position;
    const auto& s1 = poses[best_seg + 1].pose.position;
    acc += best_t * std::hypot(s1.x - s0.x, s1.y - s0.y);
    arc_m = acc;
    return true;
}

bool StairController::isStairInCooldown(
    int stair_id,
    const std::chrono::steady_clock::time_point& now,
    double* remain_sec) {
    if (!enable_stair_cooldown_ || stair_id < 0) {
        if (remain_sec) {
            *remain_sec = 0.0;
        }
        return false;
    }

    auto it = stair_cooldown_until_by_id_.find(stair_id);
    if (it == stair_cooldown_until_by_id_.end()) {
        if (remain_sec) {
            *remain_sec = 0.0;
        }
        return false;
    }

    if (now >= it->second) {
        stair_cooldown_until_by_id_.erase(it);
        if (remain_sec) {
            *remain_sec = 0.0;
        }
        return false;
    }

    if (remain_sec) {
        *remain_sec = std::chrono::duration<double>(it->second - now).count();
    }
    return true;
}

bool StairController::isFlySlopeInCooldown(
    int fly_slope_id,
    const std::chrono::steady_clock::time_point& now,
    double* remain_sec) {
    if (!enable_fly_slope_cooldown_ || fly_slope_id < 0) {
        if (remain_sec) {
            *remain_sec = 0.0;
        }
        return false;
    }

    auto it = fly_slope_cooldown_until_by_id_.find(fly_slope_id);
    if (it == fly_slope_cooldown_until_by_id_.end()) {
        if (remain_sec) {
            *remain_sec = 0.0;
        }
        return false;
    }

    if (now >= it->second) {
        fly_slope_cooldown_until_by_id_.erase(it);
        if (remain_sec) {
            *remain_sec = 0.0;
        }
        return false;
    }

    if (remain_sec) {
        *remain_sec = std::chrono::duration<double>(it->second - now).count();
    }
    return true;
}

void StairController::onStairAttemptFailed(
    int stair_id,
    const std::chrono::steady_clock::time_point& now) {
    if (stair_id < 0) {
        return;
    }

    const int fail_count = ++stair_fail_count_by_id_[stair_id];
    if (!enable_stair_cooldown_) {
        return;
    }

    if (fail_count >= std::max(1, stair_cooldown_fail_threshold_)) {
        const double dur_sec = std::max(0.0, stair_cooldown_duration_sec_);
        stair_cooldown_until_by_id_[stair_id] =
            now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                      std::chrono::duration<double>(dur_sec));
        RCLCPP_WARN(node_->get_logger(),
                    "stair_fsm cooldown set: stair_id=%d, fail_count=%d, duration=%.2fs",
                    stair_id, fail_count, dur_sec);
    }
}

void StairController::onFlySlopeAttemptFailed(
    int fly_slope_id,
    const std::chrono::steady_clock::time_point& now) {
    if (fly_slope_id < 0) {
        return;
    }

    const int fail_count = ++fly_slope_fail_count_by_id_[fly_slope_id];
    if (!enable_fly_slope_cooldown_) {
        return;
    }

    if (fail_count >= std::max(1, fly_slope_cooldown_fail_threshold_)) {
        const double dur_sec = std::max(0.0, fly_slope_cooldown_duration_sec_);
        fly_slope_cooldown_until_by_id_[fly_slope_id] =
            now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                      std::chrono::duration<double>(dur_sec));
        RCLCPP_WARN(node_->get_logger(),
                    "fly_slope_fsm cooldown set: id=%d, fail_count=%d, duration=%.2fs",
                    fly_slope_id, fail_count, dur_sec);
    }
}

void StairController::onStairAttemptSucceeded(int stair_id) {
    if (stair_id < 0) {
        return;
    }
    stair_fail_count_by_id_.erase(stair_id);
}

void StairController::onFlySlopeAttemptSucceeded(int fly_slope_id) {
    if (fly_slope_id < 0) {
        return;
    }
    fly_slope_fail_count_by_id_.erase(fly_slope_id);
}

bool StairController::queryPathHeadingNearRobot(
    const nav_core::TerrainControlContext& context,
    double& heading_rad) const {
    if (context.current_path.poses.size() < 2) {
        return false;
    }

    const auto& poses = context.current_path.poses;
    size_t closest_idx = 0;
    double min_dist2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < poses.size(); ++i) {
        const auto& p = poses[i].pose.position;
        const double dx = context.current_pose.pose.position.x - p.x;
        const double dy = context.current_pose.pose.position.y - p.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < min_dist2) {
            min_dist2 = d2;
            closest_idx = i;
        }
    }

    size_t next_idx = (closest_idx + 1 < poses.size()) ? (closest_idx + 1) : closest_idx;
    size_t prev_idx = (closest_idx > 0) ? (closest_idx - 1) : closest_idx;

    double dir_x = 0.0;
    double dir_y = 0.0;
    if (next_idx != closest_idx) {
        dir_x = poses[next_idx].pose.position.x - poses[closest_idx].pose.position.x;
        dir_y = poses[next_idx].pose.position.y - poses[closest_idx].pose.position.y;
    } else if (prev_idx != closest_idx) {
        dir_x = poses[closest_idx].pose.position.x - poses[prev_idx].pose.position.x;
        dir_y = poses[closest_idx].pose.position.y - poses[prev_idx].pose.position.y;
    }

    if (std::hypot(dir_x, dir_y) < 1e-6) {
        return false;
    }

    heading_rad = std::atan2(dir_y, dir_x);
    return true;
}

bool StairController::queryHeadingErrorToPathNearRobot(
    const nav_core::TerrainControlContext& context,
    double& heading_err_rad) const {
    double heading_ref = 0.0;
    if (!queryPathHeadingNearRobot(context, heading_ref)) {
        return false;
    }

    tf2::Quaternion q;
    tf2::fromMsg(context.current_pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    heading_err_rad = normalizeAngle(heading_ref - yaw);
    return true;
}

bool StairController::queryLateralErrorToPathNearRobot(
    const nav_core::TerrainControlContext& context,
    double& lateral_err_m) const {
    lateral_err_m = std::numeric_limits<double>::infinity();
    const auto& poses = context.current_path.poses;
    if (poses.size() < 2) {
        return false;
    }

    const Eigen::Vector2d robot_pos(context.current_pose.pose.position.x,
                                    context.current_pose.pose.position.y);
    double best_dist2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < poses.size(); ++i) {
        const Eigen::Vector2d a(poses[i].pose.position.x, poses[i].pose.position.y);
        const Eigen::Vector2d b(poses[i + 1].pose.position.x, poses[i + 1].pose.position.y);
        const Eigen::Vector2d ab = b - a;
        const double ab2 = ab.squaredNorm();
        if (ab2 < 1e-9) {
            continue;
        }

        const double t = std::clamp((robot_pos - a).dot(ab) / ab2, 0.0, 1.0);
        const Eigen::Vector2d projection = a + t * ab;
        const double dist2 = (robot_pos - projection).squaredNorm();
        if (dist2 < best_dist2) {
            best_dist2 = dist2;
        }
    }

    if (!std::isfinite(best_dist2)) {
        return false;
    }

    lateral_err_m = std::sqrt(best_dist2);
    return true;
}

bool StairController::detectUpcomingStairDistance(
    const nav_core::TerrainControlContext& context,
    double& dist_to_stair_m) const {
    dist_to_stair_m = std::numeric_limits<double>::infinity();

    if (!map_manager_ || context.current_path.poses.size() < 2) {
        return false;
    }

    const auto& poses = context.current_path.poses;
    const double sample_step = std::max(0.02, stair_mode_sample_step_m_);
    const double lookahead_dist = std::max(0.1, stair_mode_lookahead_dist_m_);

    size_t closest_idx = 0;
    double min_dist2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < poses.size(); ++i) {
        const auto& p = poses[i].pose.position;
        double dx = context.current_pose.pose.position.x - p.x;
        double dy = context.current_pose.pose.position.y - p.y;
        double d2 = dx * dx + dy * dy;
        if (d2 < min_dist2) {
            min_dist2 = d2;
            closest_idx = i;
        }
    }

    double traveled = 0.0;
    for (size_t i = closest_idx; i + 1 < poses.size() && traveled <= lookahead_dist; ++i) {
        const auto& p0 = poses[i].pose.position;
        const auto& p1 = poses[i + 1].pose.position;
        double seg_dx = p1.x - p0.x;
        double seg_dy = p1.y - p0.y;
        double seg_len = std::hypot(seg_dx, seg_dy);
        if (seg_len < 1e-6) {
            continue;
        }
        const double dir_x = seg_dx / seg_len;
        const double dir_y = seg_dy / seg_len;

        int sample_count = std::max(1, static_cast<int>(std::ceil(seg_len / sample_step)));
        for (int s = 0; s <= sample_count; ++s) {
            double t = static_cast<double>(s) / static_cast<double>(sample_count);
            double local_dist = seg_len * t;
            double path_dist = traveled + local_dist;
            if (path_dist > lookahead_dist) {
                break;
            }

            double wx = p0.x + seg_dx * t;
            double wy = p0.y + seg_dy * t;
            double nx = 0.0;
            double ny = 0.0;
            if (map_manager_->getStairTraverseNormal(wx, wy, nx, ny)) {
                const double dot = dir_x * nx + dir_y * ny;
                if (dot >= 0.0) {
                    dist_to_stair_m = path_dist;
                    return true;
                }
            }
        }

        traveled += seg_len;
    }

    return false;
}

void StairController::publishStairMode(uint8_t mode, bool force_publish, bool bypass_hold) {
    if (!stair_mode_pub_) {
        return;
    }

    uint8_t effective_mode = mode;
    const auto now_tp = std::chrono::steady_clock::now();
    if (effective_mode == 1 || effective_mode == 3) {
        const auto last_publish_time = (effective_mode == 3)
            ? stair_mode_last_mode3_publish_time_
            : stair_mode_last_mode1_publish_time_;
        if (stair_mode_reassert_block_sec_ > 0.0 &&
            last_publish_time != std::chrono::steady_clock::time_point{}) {
            const double elapsed_since_publish = std::chrono::duration<double>(
                now_tp - last_publish_time).count();
            if (elapsed_since_publish < stair_mode_reassert_block_sec_) {
                return;
            }
        }
        stair_mode_last_assert_time_ = now_tp;
    } else if (effective_mode == 2) {
        if (fly_slope_mode_reassert_block_sec_ > 0.0 &&
            stair_mode_last_mode2_publish_time_ != std::chrono::steady_clock::time_point{}) {
            const double elapsed_since_mode2_publish = std::chrono::duration<double>(
                now_tp - stair_mode_last_mode2_publish_time_).count();
            if (elapsed_since_mode2_publish < fly_slope_mode_reassert_block_sec_) {
                return;
            }
        }
        stair_mode_last_assert_time_ = now_tp;
    } else if (!bypass_hold &&
               (stair_mode_current_ == 1 ||
                stair_mode_current_ == 2 ||
                stair_mode_current_ == 3)) {
        const double min_hold_sec = (stair_mode_current_ == 2)
            ? fly_slope_mode_min_hold_sec_
            : stair_mode_min_hold_sec_;
        const double elapsed_since_assert = std::chrono::duration<double>(
            now_tp - stair_mode_last_assert_time_).count();
        if (elapsed_since_assert < min_hold_sec) {
            effective_mode = stair_mode_current_;
        }
    }

    if (!force_publish && effective_mode == stair_mode_current_) {
        return;
    }

    std_msgs::msg::UInt8 msg;
    msg.data = effective_mode;
    stair_mode_pub_->publish(msg);
    if (effective_mode == 1) {
        stair_mode_last_mode1_publish_time_ = now_tp;
    } else if (effective_mode == 2) {
        stair_mode_last_mode2_publish_time_ = now_tp;
    } else if (effective_mode == 3) {
        stair_mode_last_mode3_publish_time_ = now_tp;
    }

    if (effective_mode != stair_mode_current_) {
        RCLCPP_INFO(node_->get_logger(), "stair_mode -> %u", static_cast<unsigned>(effective_mode));
        if (effective_mode == 1 || effective_mode == 3) {
            stair_mode_enter_time_ = now_tp;
        } else {
            stair_mode_enter_time_ = std::chrono::steady_clock::time_point{};
        }
        stair_mode_current_ = effective_mode;
    }
}

void StairController::updateStairModeDetection(const nav_core::TerrainControlContext& context) {
    // 旧 stair_mode 检测逻辑已由 FSM 接管，保留兼容接口：仅持续发布当前模式。
    (void)context;
    publishStairMode(stair_mode_current_, true);
}

const char* StairController::fsmStateName(StairFsmState state) {
    switch (state) {
        case StairFsmState::NORMAL: return "NORMAL";
        case StairFsmState::PRE_ALIGN: return "PRE_ALIGN";
        case StairFsmState::COMMIT_ASCENT: return "COMMIT_ASCENT";
        case StairFsmState::VERIFY_SUCCESS: return "VERIFY_SUCCESS";
        case StairFsmState::FAIL_RETRY_BACKOFF: return "FAIL_RETRY_BACKOFF";
        case StairFsmState::COOLDOWN_BLOCKED: return "COOLDOWN_BLOCKED";
    }
    return "UNKNOWN";
}

void StairController::publishFsmDebug(const std::chrono::steady_clock::time_point& now) {
    if (!stair_fsm_state_pub_) {
        return;
    }

    double remain_sec = 0.0;
    bool cooling = false;
    if (cooldown_stair_id_ >= 0) {
        cooling = (active_terrain_type_ == TerrainType::FLY_SLOPE)
            ? isFlySlopeInCooldown(cooldown_stair_id_, now, &remain_sec)
            : isStairInCooldown(cooldown_stair_id_, now, &remain_sec);
    }

    std_msgs::msg::String msg;
    std::ostringstream oss;
    oss << "state=" << fsmStateName(fsm_state_)
        << ",reason=" << last_transition_reason_
        << ",active_terrain=" << terrainTypeName(active_terrain_type_)
        << ",active_feature_id=" << active_feature_.feature_id
        << ",attempt=" << active_attempt_count_
        << ",cooldown_feature_id=" << cooldown_stair_id_
        << ",cooldown_active=" << (cooling ? 1 : 0)
        << ",cooldown_remain_sec=" << remain_sec;
    msg.data = oss.str();
    stair_fsm_state_pub_->publish(msg);
}

void StairController::publishCooldownMarkers(const std::chrono::steady_clock::time_point& now) {
    if (!stair_cooldown_marker_pub_) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = node_->now();
    clear.ns = "stair_cooldown";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear);

    if (!map_manager_) {
        stair_cooldown_marker_pub_->publish(marker_array);
        return;
    }

    std::vector<int> expired_ids;
    for (const auto& kv : stair_cooldown_until_by_id_) {
        const int stair_id = kv.first;
        const auto until_tp = kv.second;
        if (now >= until_tp) {
            expired_ids.push_back(stair_id);
            continue;
        }

        LayeredMapManager::StairPrimitive primitive;
        if (!map_manager_->getStairPrimitiveById(stair_id, primitive)) {
            continue;
        }

        const double remain_sec = std::chrono::duration<double>(until_tp - now).count();

        visualization_msgs::msg::Marker txt;
        txt.header.frame_id = "map";
        txt.header.stamp = node_->now();
        txt.ns = "stair_cooldown_text";
        txt.id = stair_id * 2 + 1;
        txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        txt.action = visualization_msgs::msg::Marker::ADD;
        txt.pose.position.x = primitive.center.x();
        txt.pose.position.y = primitive.center.y();
        txt.pose.position.z = 0.35;
        txt.pose.orientation.w = 1.0;
        txt.scale.z = 0.18;
        txt.color.r = 1.0f;
        txt.color.g = 0.5f;
        txt.color.b = 0.1f;
        txt.color.a = 1.0f;
        txt.text = "cooldown id=" + std::to_string(stair_id) +
                   " t=" + std::to_string(remain_sec).substr(0, 4) + "s";
        marker_array.markers.push_back(txt);

        visualization_msgs::msg::Marker pt;
        pt.header = txt.header;
        pt.ns = "stair_cooldown_center";
        pt.id = stair_id * 2 + 2;
        pt.type = visualization_msgs::msg::Marker::SPHERE;
        pt.action = visualization_msgs::msg::Marker::ADD;
        pt.pose.position = txt.pose.position;
        pt.pose.position.z = 0.08;
        pt.pose.orientation.w = 1.0;
        pt.scale.x = 0.12;
        pt.scale.y = 0.12;
        pt.scale.z = 0.12;
        pt.color.r = 1.0f;
        pt.color.g = 0.1f;
        pt.color.b = 0.1f;
        pt.color.a = 0.9f;
        marker_array.markers.push_back(pt);
    }

    for (int sid : expired_ids) {
        stair_cooldown_until_by_id_.erase(sid);
    }

    stair_cooldown_marker_pub_->publish(marker_array);
}

void StairController::syncRuntimeBlockedUphillStairs(
    const std::chrono::steady_clock::time_point& now) {
    if (!map_manager_) {
        return;
    }

    std::unordered_set<int> active_blocked_ids;
    active_blocked_ids.reserve(stair_cooldown_until_by_id_.size());

    std::vector<int> expired_ids;
    expired_ids.reserve(stair_cooldown_until_by_id_.size());
    for (const auto& kv : stair_cooldown_until_by_id_) {
        if (now >= kv.second) {
            expired_ids.push_back(kv.first);
            continue;
        }
        active_blocked_ids.insert(kv.first);
    }

    for (int sid : expired_ids) {
        stair_cooldown_until_by_id_.erase(sid);
    }

    if (active_blocked_ids.empty()) {
        map_manager_->clearRuntimeBlockedStairUphillIds();
        return;
    }
    map_manager_->setRuntimeBlockedStairUphillIds(active_blocked_ids);
}

void StairController::applyStairModeOmegaLimit(geometry_msgs::msg::Twist& cmd,
                                               double control_rate_hz) {
    if (stair_mode_current_ != 1 && stair_mode_current_ != 2 && stair_mode_current_ != 3) {
        stair_mode_omega_limiter_initialized_ = false;
        return;
    }

    const bool fly_mode = (stair_mode_current_ == 2);
    const double omega_limit = fly_mode
        ? fly_slope_mode_omega_limit_rad_s_
        : stair_mode_omega_limit_rad_s_;
    const double omega_slew = fly_mode
        ? fly_slope_mode_omega_slew_rate_rad_s2_
        : stair_mode_omega_slew_rate_rad_s2_;

    const double raw_omega = cmd.angular.z;
    double limited_omega = raw_omega;

    if (omega_limit > 0.0) {
        limited_omega = std::clamp(
            limited_omega,
            -omega_limit,
            omega_limit);
    }

    if (omega_slew > 0.0) {
        const double dt = 1.0 / std::max(1.0, control_rate_hz);
        const double max_delta = omega_slew * dt;
        if (!stair_mode_omega_limiter_initialized_) {
            last_stair_mode_limited_omega_ = limited_omega;
            stair_mode_omega_limiter_initialized_ = true;
        } else {
            limited_omega = std::clamp(
                limited_omega,
                last_stair_mode_limited_omega_ - max_delta,
                last_stair_mode_limited_omega_ + max_delta);
            last_stair_mode_limited_omega_ = limited_omega;
        }
    }

    cmd.angular.z = limited_omega;

    if (std::abs(raw_omega - limited_omega) > 1e-4) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "stair_mode omega limit: raw=%.3f -> limited=%.3f (limit=%.3f, slew=%.3f)",
            raw_omega,
            limited_omega,
            omega_limit,
            omega_slew);
    }
}

}  // namespace nav_components
