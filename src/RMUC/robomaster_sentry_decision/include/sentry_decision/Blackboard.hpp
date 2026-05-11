#ifndef BLACKBOARD_HPP
#define BLACKBOARD_HPP

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <geometry_msgs/msg/point.hpp>
#include "decision_messages/msg/our_robot_state.hpp"
#include "decision_messages/msg/enemy_robot_state.hpp"
#include "decision_messages/msg/game_state.hpp"
#include "sentry_decision/msg/sentry_control.hpp"
#include "GameConstants.hpp"

using OurRobotState = decision_messages::msg::OurRobotState;
using EnemyRobotState = decision_messages::msg::EnemyRobotState;
using GameState = decision_messages::msg::GameState;
using SentryControl = sentry_decision::msg::SentryControl;

enum class BehaviorState {
    IDLE,
    MOVING,
    EXECUTING,
    COMPLETED
};

enum class BehaviorType {
    NONE,
    INIT_MOVE,
    INIT_ATTACK,
    MOVE_TO_ATTACK_HERO,
    ATTACK_HERO,
    MOVE_TO_ATTACK_ROBOT,
    ATTACK_ROBOT,
    MOVE_TO_SUPPLY,
    SUPPLY,
    RESURRECTION_MOVE,
    RESURRECTING,
    MOVE_TO_BASE_DEFENSE,
    BASE_DEFENSE,
    MOVE_TO_GAIN_POINT,
    OCCUPY_GAIN_POINT,
    MOVE_TO_FORTRESS,
    OCCUPY_FORTRESS,
    RAMP_PROCESS
};

struct BehaviorInfo {
    BehaviorType type = BehaviorType::NONE;
    BehaviorState state = BehaviorState::IDLE;
    geometry_msgs::msg::Point target;
    double start_time = -1.0;
    double execution_start_time = -1.0;
    double execution_duration = 0.0;
    bool target_published = false;
    bool control_published = false;
    bool control_updated = false;
};

struct EnemyInfo {
    std::string id;
    std::string type;
    double x = 0.0, y = 0.0;
    double hp = 0.0;
    double allowance = 0.0;
    bool visible = false;
    double last_update_time = 0.0;
    EnemyInfo(const std::string& id_val, const std::string& type_val) : id(id_val), type(type_val) {}
};

struct GainPointStatus {
    std::string name;
    geometry_msgs::msg::Point position;
    double defense_gain = 0.0;
    bool occupied_by_us = false;
    bool occupied_by_enemy = false;
    bool neutral = true;
};

struct PriorityConfig {
    std::string type;
    double weight_hp;
    double weight_distance;
    double weight_ammo;
    double threshold;
};

class Blackboard {
public:
    Blackboard();

    bool loadConfigFromYAML(const std::string& filepath);

    // 坐标 getter
    geometry_msgs::msg::Point getAttackPoint() const;
    geometry_msgs::msg::Point getSupplyPoint() const;
    geometry_msgs::msg::Point getBaseGainPoint() const;
    geometry_msgs::msg::Point getFortressOccupyPoint() const;
    // ramp removed: getRampPoint() removed
    geometry_msgs::msg::Point getFortressGainPoint() const;
    geometry_msgs::msg::Point getCentralHighlandGain() const;
    geometry_msgs::msg::Point getTrapezoidHighlandGain() const;
    geometry_msgs::msg::Point getEnemyOutpostPoint() const;

    // 参数 getter
    double getArrivalWaitTime() const;
    double getDeviationThreshold() const;
    double getInitAttackDuration() const;
    double getAttackDuration() const;
    double getDefendDuration() const;
    double getSupplyThreshold() const;
    double getMaxHp() const;
    double getMaxAmmo() const;
    double getHalfMapX() const;

    double getHpWeight() const;
    double getAmmoWeight() const;
    double getBaseWeight() const;

    double getHeroAttackZThreshold() const;
    double getHeroAttackHThreshold() const;
    double getHeroHighPriorityZThreshold() const;

    double getTargetSelectionZHigh() const;
    double getTargetSelectionZMid() const;
    double getTargetSelectionThresholdHigh() const;
    double getTargetSelectionThresholdMid() const;
    double getTargetSelectionThresholdLow() const;

    double getGainPointZHigh() const;
    double getGainPointZMid() const;
    double getGainPointThresholdHigh() const;
    double getGainPointThresholdMid() const;
    double getGainPointThresholdLow() const;

    double getFortressOccupyZThreshold() const;
    double getFortressOccupyFThreshold() const;
    double getFortressOccupyHpRatio() const;

    const std::vector<PriorityConfig>& getPriorityTargets() const { return priority_targets_config_; }

    // 状态更新
    void updateOurState(const OurRobotState::SharedPtr msg);
    void updateEnemyState(const EnemyRobotState::SharedPtr msg);
    void updateGameState(const GameState::SharedPtr msg);
    void updatePositionFromTF(double x_m, double y_m);
    void resetForNewMatch();

    // 公共成员（状态机直接使用，符合省赛风格）
    double current_hp = 400.0;
    double allowance_17mm = 300.0;
    double x = 0.0, y = 0.0;
    double our_base_hp = 5000.0;
    double our_outpost_hp = 1500.0;
    uint8_t stage = 0;
    double stage_remaining_time = 420.0;
    double current_time = 0.0;

    bool resurrection_flag = false;
    bool initialization_complete = false;
    bool at_supply_point = false;
    bool hero_in_deploy_zone = false;

    bool at_current_target = false;
    double target_arrival_time = -1.0;
    double min_stay_duration = 3.0;

    BehaviorInfo current_behavior;

    std::vector<GainPointStatus> gain_points;

    EnemyInfo enemy_hero;
    EnemyInfo enemy_engineer;
    EnemyInfo enemy_infantry3;
    EnemyInfo enemy_infantry4;
    EnemyInfo enemy_sentry;

    bool base_gain_point_occupied = false;
    bool trapezoid_highland_occupied = false;
    bool fortress_gain_point_occupied_by_us = false;
    bool fortress_gain_point_occupied_by_enemy = false;
    bool central_highland_occupied_by_us = false;
    bool central_highland_occupied_by_enemy = false;
    bool outpost_gain_point_occupied_by_us = false;
    bool outpost_gain_point_occupied_by_enemy = false;

    bool isAtTarget(const geometry_msgs::msg::Point& target, double tolerance = 50.0) const;
    void setTargetReached(bool reached);
    bool shouldLeaveTarget() const;
    bool hasWaitedAtTarget(double wait_seconds) const;
    void updateControlMsg(uint8_t gimbal_mode, uint8_t spin_mode, uint8_t posture);
    std::shared_ptr<SentryControl> getControlMsg() const { return control_msg_; }

    void startBehavior(BehaviorType type, const geometry_msgs::msg::Point& target, double duration);
    void updateBehaviorState(BehaviorState state);
    void startExecutionTime();
    void completeCurrentBehavior();
    void resetCurrentBehavior();
    bool isBehaviorInProgress() const;
    double getExecutionElapsedTime() const;

    void setTargetPublished(bool published) { current_behavior.target_published = published; }
    bool isTargetPublished() const { return current_behavior.target_published; }
    void setControlPublished(bool published) { current_behavior.control_published = published; }
    bool isControlPublished() const { return current_behavior.control_published; }
    void setControlUpdated(bool updated) { current_behavior.control_updated = updated; }
    bool isControlUpdated() const { return current_behavior.control_updated; }
    void resetAllPublishStates();

    EnemyInfo* getEnemyById(const std::string& id);
    const EnemyInfo* getEnemyById(const std::string& id) const;

    // ramp removed: related fields and methods removed

private:
    struct Config {
        geometry_msgs::msg::Point red_attack;
        geometry_msgs::msg::Point blue_attack;
        geometry_msgs::msg::Point red_supply;
        geometry_msgs::msg::Point blue_supply;
        geometry_msgs::msg::Point red_base_gain;
        geometry_msgs::msg::Point blue_base_gain;
        geometry_msgs::msg::Point red_fortress_occupy;
        geometry_msgs::msg::Point blue_fortress_occupy;
        geometry_msgs::msg::Point red_ramp_point;
        geometry_msgs::msg::Point blue_ramp_point;
        geometry_msgs::msg::Point red_fortress_gain;
        geometry_msgs::msg::Point blue_fortress_gain;
        geometry_msgs::msg::Point central_highland_gain;
        geometry_msgs::msg::Point trapezoid_highland_gain;
        geometry_msgs::msg::Point red_enemy_outpost;
        geometry_msgs::msg::Point blue_enemy_outpost;

        double arrival_wait_time;
        double deviation_threshold;
        double init_attack_duration;
        double attack_duration;
        double defend_duration;

        double hp_weight;
        double ammo_weight;
        double base_weight;

        double supply_threshold;
        double max_hp;
        double max_ammo;

        double hero_attack_z_threshold;
        double hero_attack_h_threshold;
        double hero_high_priority_z_threshold;

        double target_selection_z_high;
        double target_selection_z_mid;
        double target_selection_threshold_high;
        double target_selection_threshold_mid;
        double target_selection_threshold_low;

        double gain_point_z_high;
        double gain_point_z_mid;
        double gain_point_threshold_high;
        double gain_point_threshold_mid;
        double gain_point_threshold_low;

        double fortress_occupy_z_threshold;
        double fortress_occupy_f_threshold;
        double fortress_occupy_hp_ratio;

        double half_map_x;
    } config_;

    std::shared_ptr<SentryControl> control_msg_;
    uint8_t robot_id_ = 0;
    uint8_t last_stage_ = 0;

    std::vector<PriorityConfig> priority_targets_config_;

    void initializeGainPoints();
    void updateGainPointStatus();
    void onRobotIdChanged(uint8_t old_id, uint8_t new_id);
};

#endif // BLACKBOARD_HPP
