#ifndef BLACKBOARD_HPP
#define BLACKBOARD_HPP

#include <memory>
#include <string>
#include <geometry_msgs/msg/point.hpp>
#include "decision_messages/msg/our_robot_state.hpp"
#include "decision_messages/msg/game_state.hpp"
#include "sentry_decision/msg/sentry_control.hpp"
#include "Constants.hpp"

using OurRobotState = decision_messages::msg::OurRobotState;
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
    MOVE_TO_ATTACK,
    MOVE_TO_SUPPLY,
    RESURRECTION,
    SUPPLY
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

class Blackboard {
public:
    Blackboard();

    // 从 YAML 文件加载配置
    bool loadConfigFromYAML(const std::string& filepath);

    // 配置 getter
    geometry_msgs::msg::Point getAttackPoint() const;   // 根据 robot_id 返回攻击点
    geometry_msgs::msg::Point getSupplyPoint() const;   // 根据 robot_id 返回补给点
    double getArrivalWaitTime() const { return config_.arrival_wait_time; }
    double getSupplyThreshold() const { return config_.supply_threshold; }
    double getDeviationThreshold() const { return config_.deviation_threshold; }
    double getHpWeight() const { return config_.hp_weight; }
    double getAmmoWeight() const { return config_.ammo_weight; }

    // 根据当前阶段返回云台模式 (0: 静止, 1: 打人)
    uint8_t getGimbalModeByStage() const;

    void updateOurState(const OurRobotState::SharedPtr msg);
    void updateGameState(const GameState::SharedPtr msg);
    void resetForNewMatch();

    void updatePositionFromTF(double x_m, double y_m);

    std::shared_ptr<SentryControl> getControlMsg() const { return control_msg_; }
    void updateControlMsg(uint8_t gimbal_mode, uint8_t spin_mode);

    uint8_t getRobotId() const { return robot_id_; }

    double current_hp = 400.0;
    double allowance_17mm = 300.0;
    double x = 0.0, y = 0.0;
    uint8_t stage = 0;
    double stage_remaining_time = 420.0;
    double current_time = 0.0;

    bool resurrection_flag = false;

    bool at_supply_point = false;
    bool at_current_target = false;
    double target_arrival_time = -1.0;
    double min_stay_duration = 3.0;   // 暂时保持，后续可配置

    BehaviorInfo current_behavior;

    bool isAtTarget(const geometry_msgs::msg::Point& target, double tolerance = 50.0) const;
    void setTargetReached(bool reached);
    bool shouldLeaveTarget() const;
    bool hasWaitedAtTarget(double wait_seconds) const;

    void startBehavior(BehaviorType type, const geometry_msgs::msg::Point& target, double duration = 0.0);
    void updateBehaviorState(BehaviorState state);
    void startExecutionTime();
    void completeCurrentBehavior();
    void resetCurrentBehavior();

    bool isBehaviorInProgress() const {
        return current_behavior.state != BehaviorState::IDLE &&
               current_behavior.state != BehaviorState::COMPLETED;
    }

    double getExecutionElapsedTime() const {
        if (current_behavior.execution_start_time < 0) return 0.0;
        return current_time - current_behavior.execution_start_time;
    }

    void setTargetPublished(bool published) { current_behavior.target_published = published; }
    bool isTargetPublished() const { return current_behavior.target_published; }
    void setControlPublished(bool published) { current_behavior.control_published = published; }
    bool isControlPublished() const { return current_behavior.control_published; }
    void setControlUpdated(bool updated) { current_behavior.control_updated = updated; }
    bool isControlUpdated() const { return current_behavior.control_updated; }
    void resetAllPublishStates() {
        setTargetPublished(false);
        setControlPublished(false);
        setControlUpdated(false);
    }

    void onRobotIdChanged(uint8_t old_id, uint8_t new_id);

private:
    struct Config {
        // 攻击点 (单位: cm)
        geometry_msgs::msg::Point red_attack;
        geometry_msgs::msg::Point blue_attack;
        // 补给点 (单位: cm)
        geometry_msgs::msg::Point red_supply;
        geometry_msgs::msg::Point blue_supply;
        double arrival_wait_time = SentryConstants::ARRIVAL_WAIT_TIME;
        double supply_threshold = SentryConstants::SUPPLY_THRESHOLD;
        double deviation_threshold = 0.5;   // 单位: 米
        double hp_weight = 1.0;
        double ammo_weight = 0.0;
    } config_;

    std::shared_ptr<SentryControl> control_msg_;
    uint8_t last_stage_ = 0;
    uint8_t robot_id_ = 0;
};

#endif // BLACKBOARD_HPP
