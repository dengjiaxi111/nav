// nav_components/include/nav_components/stair_controller.hpp
// 台阶特殊地形控制器（阶段A：迁移 nav_server 既有 stair_mode 行为）

#pragma once

#include <chrono>
#include <unordered_map>

#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav_components/layered_map_manager.hpp"
#include "nav_core/special_terrain_controller.hpp"

namespace nav_components {

class StairController : public nav_core::SpecialTerrainController {
public:
    void initialize(rclcpp::Node* node) override;
    void setMap(nav_core::MapInterface::Ptr map) override;

    nav_core::TerrainControlDecision update(
        const nav_core::TerrainControlContext& context,
        const geometry_msgs::msg::Twist& base_cmd,
        geometry_msgs::msg::Twist& out_cmd) override;

    void onNavStateChanged(nav_core::NavState state) override;

private:
    enum class TerrainType : uint8_t {
        NONE = 0,
        STAIR = 1,
        FLY_SLOPE = 2,
    };

    enum class StairFsmState : uint8_t {
        NORMAL = 0,
        PRE_ALIGN = 1,
        COMMIT_ASCENT = 2,
        VERIFY_SUCCESS = 3,
        FAIL_RETRY_BACKOFF = 4,
        COOLDOWN_BLOCKED = 5,
    };

    struct TerrainCandidateInfo {
        bool valid{false};
        TerrainType terrain_type{TerrainType::NONE};
        double dist_to_feature_m{0.0};
        int feature_id{-1};
        Eigen::Vector2d center{Eigen::Vector2d::Zero()};
        Eigen::Vector2d normal{Eigen::Vector2d::Zero()};
        Eigen::Vector2d tangent{Eigen::Vector2d::Zero()};
        double half_length{0.0};
    };

    static double normalizeAngle(double angle);
    static const char* fsmStateName(StairFsmState state);
    static const char* terrainTypeName(TerrainType terrain_type);
    void transitionFsmState(StairFsmState new_state, const char* reason);
    bool queryUpcomingTerrainCandidate(const nav_core::TerrainControlContext& context,
                                       TerrainType preferred_type,
                                       TerrainCandidateInfo& candidate) const;
    bool queryUpcomingStairCandidate(const nav_core::TerrainControlContext& context,
                                     TerrainCandidateInfo& candidate) const;
    bool queryUpcomingFlySlopeCandidate(const nav_core::TerrainControlContext& context,
                                        TerrainCandidateInfo& candidate) const;
    bool queryPathHeadingNearRobot(const nav_core::TerrainControlContext& context,
                                   double& heading_rad) const;
    bool queryHeadingErrorToPathNearRobot(const nav_core::TerrainControlContext& context,
                                          double& heading_err_rad) const;
    bool queryLateralErrorToPathNearRobot(const nav_core::TerrainControlContext& context,
                                          double& lateral_err_m) const;
    bool computePathArcAtRobot(const nav_core::TerrainControlContext& context,
                               double& arc_m) const;
    bool isStairInCooldown(int stair_id,
                           const std::chrono::steady_clock::time_point& now,
                           double* remain_sec = nullptr);
    bool isFlySlopeInCooldown(int fly_slope_id,
                              const std::chrono::steady_clock::time_point& now,
                              double* remain_sec = nullptr);
    void onStairAttemptFailed(int stair_id,
                              const std::chrono::steady_clock::time_point& now);
    void onFlySlopeAttemptFailed(int fly_slope_id,
                                 const std::chrono::steady_clock::time_point& now);
    void onStairAttemptSucceeded(int stair_id);
    void onFlySlopeAttemptSucceeded(int fly_slope_id);
    bool detectUpcomingStairDistance(const nav_core::TerrainControlContext& context,
                                     double& dist_to_stair_m) const;
    void publishFsmDebug(const std::chrono::steady_clock::time_point& now);
    void publishCooldownMarkers(const std::chrono::steady_clock::time_point& now);
    void syncRuntimeBlockedUphillStairs(const std::chrono::steady_clock::time_point& now);

    void publishStairMode(uint8_t mode, bool force_publish, bool bypass_hold = false);
    void updateStairModeDetection(const nav_core::TerrainControlContext& context);
    void applyStairModeOmegaLimit(geometry_msgs::msg::Twist& cmd, double control_rate_hz);

    rclcpp::Node* node_{nullptr};
    std::shared_ptr<LayeredMapManager> map_manager_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr stair_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stair_fsm_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stair_transition_debug_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stair_cooldown_marker_pub_;

    bool enable_stair_mode_detection_{true};
    bool enable_fly_slope_mode_detection_{true};
    bool enable_stair_fsm_{true};
    double stair_mode_trigger_distance_m_{1.0};
    double stair_mode_lookahead_dist_m_{3.0};
    double stair_mode_sample_step_m_{0.10};
    double stair_mode_entry_heading_error_max_rad_{0.35};
    int stair_mode_release_grace_cycles_{5};
    double stair_mode_min_hold_sec_{0.35};
    double stair_mode_reassert_block_sec_{0.0};
    double stair_mode_force_release_distance_m_{2.5};
    double stair_mode_max_assert_sec_{6.0};
    double stair_mode_omega_limit_rad_s_{0.20};
    double stair_mode_omega_slew_rate_rad_s2_{1.2};
    bool enable_stair_fixed_velocity_strategy_{false};
    double stair_fixed_velocity_trigger_distance_m_{0.35};
    double stair_fixed_linear_vel_{0.35};
    double stair_fixed_heading_kp_{1.8};
    double stair_fixed_max_angular_vel_{0.8};
    double stair_fixed_heading_deadband_{0.05};

    // 飞坡参数（与台阶参数一一对应，默认初值一致）
    double fly_slope_mode_trigger_distance_m_{1.0};
    double fly_slope_mode_lookahead_dist_m_{3.0};
    double fly_slope_mode_sample_step_m_{0.10};
    double fly_slope_mode_entry_heading_error_max_rad_{0.35};
    int fly_slope_mode_release_grace_cycles_{5};
    double fly_slope_mode_min_hold_sec_{0.35};
    double fly_slope_mode_reassert_block_sec_{0.0};
    double fly_slope_mode_force_release_distance_m_{2.5};
    double fly_slope_mode_max_assert_sec_{6.0};
    double fly_slope_mode_omega_limit_rad_s_{0.20};
    double fly_slope_mode_omega_slew_rate_rad_s2_{1.2};
    double fly_slope_pre_align_lateral_error_max_m_{0.08};
    double fly_slope_pre_align_return_max_linear_vel_{0.35};
    double fly_slope_pre_align_return_max_angular_vel_{1.0};
    bool enable_fly_slope_pre_align_turn_in_place_{true};
    double fly_slope_pre_align_heading_kp_{2.0};
    double fly_slope_pre_align_max_angular_vel_{1.2};
    bool enable_fly_slope_fixed_velocity_strategy_{false};
    double fly_slope_fixed_velocity_trigger_distance_m_{0.35};
    double fly_slope_fixed_linear_vel_{0.35};
    double fly_slope_fixed_heading_kp_{1.8};
    double fly_slope_fixed_max_angular_vel_{0.8};
    double fly_slope_fixed_heading_deadband_{0.05};
    double fly_slope_raise_leg_distance_m_{0.40};
    double fly_slope_contact_distance_m_{0.25};
    double fly_slope_commit_success_dist_m_{0.18};
    double fly_slope_verify_timeout_sec_{1.2};
    double fly_slope_progress_timeout_sec_{1.2};
    double fly_slope_progress_min_arc_m_{0.18};
    int fly_slope_fail_a_precontact_miss_cycles_{3};
    double fly_slope_backoff_distance_m_{0.60};
    double fly_slope_backoff_tangent_search_half_width_m_{0.30};
    double fly_slope_backoff_linear_vel_{0.30};
    double fly_slope_backoff_heading_kp_{2.0};
    double fly_slope_backoff_max_angular_vel_{1.0};
    double fly_slope_backoff_pos_tolerance_m_{0.08};
    int fly_slope_retry_max_attempts_{3};
    bool fly_slope_request_recovery_on_max_attempts_{true};
    bool enable_fly_slope_cooldown_{true};
    int fly_slope_cooldown_fail_threshold_{3};
    double fly_slope_cooldown_duration_sec_{30.0};

    // 独立动作阈值（在 COMMIT_ASCENT 及以内生效）
    double stair_raise_leg_distance_m_{0.40};

    // 阶段B FSM 参数（不含冷却）
    double stair_contact_distance_m_{0.25};
    double stair_commit_success_dist_m_{0.18};
    double stair_verify_timeout_sec_{1.2};
    double stair_progress_timeout_sec_{1.2};
    double stair_progress_min_arc_m_{0.18};
    int stair_fail_a_precontact_miss_cycles_{3};
    double stair_backoff_distance_m_{0.60};
    double stair_backoff_tangent_search_half_width_m_{0.30};
    double stair_backoff_linear_vel_{0.30};
    double stair_backoff_heading_kp_{2.0};
    double stair_backoff_max_angular_vel_{1.0};
    double stair_backoff_pos_tolerance_m_{0.08};
    int stair_retry_max_attempts_{3};
    bool stair_request_recovery_on_max_attempts_{true};

    // 阶段C：同台阶重试与冷却
    bool enable_stair_cooldown_{true};
    int stair_cooldown_fail_threshold_{3};
    double stair_cooldown_duration_sec_{30.0};

    uint8_t stair_mode_current_{0};
    int virtual_leg_length_{1}; // 预留的虚拟腿长信号(1表示长腿，满足条件不触发失败，0表示短腿会触发失败)
    int stair_mode_release_counter_{0};
    std::chrono::steady_clock::time_point stair_mode_last_assert_time_{};
    std::chrono::steady_clock::time_point stair_mode_last_mode1_publish_time_{};
    std::chrono::steady_clock::time_point stair_mode_last_mode2_publish_time_{};
    std::chrono::steady_clock::time_point stair_mode_enter_time_{};
    bool stair_mode_omega_limiter_initialized_{false};
    double last_stair_mode_limited_omega_{0.0};

    StairFsmState fsm_state_{StairFsmState::NORMAL};
    std::chrono::steady_clock::time_point fsm_state_enter_time_{};
    TerrainType active_terrain_type_{TerrainType::NONE};
    TerrainCandidateInfo active_feature_{};
    int active_attempt_count_{0};
    int precontact_miss_counter_{0};
    double commit_arc_start_m_{0.0};
    std::chrono::steady_clock::time_point commit_progress_start_time_{};
    int cooldown_stair_id_{-1};
    bool cooldown_replan_pending_{false};
    std::string last_transition_reason_{"init"};
    std::unordered_map<int, int> stair_fail_count_by_id_;
    std::unordered_map<int, std::chrono::steady_clock::time_point> stair_cooldown_until_by_id_;
    std::unordered_map<int, int> fly_slope_fail_count_by_id_;
    std::unordered_map<int, std::chrono::steady_clock::time_point> fly_slope_cooldown_until_by_id_;
};

}  // namespace nav_components
