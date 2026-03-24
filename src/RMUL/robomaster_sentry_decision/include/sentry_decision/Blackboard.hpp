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

    void updateOurState(const OurRobotState::SharedPtr msg);
    void updateGameState(const GameState::SharedPtr msg);
    void resetForNewMatch();

    // 通过TF更新坐标（输入单位为米）
    void updatePositionFromTF(double x_m, double y_m);

    std::shared_ptr<SentryControl> getControlMsg() const { return control_msg_; }
    void updateControlMsg(uint8_t spin_mode);   // 仅控制小陀螺

    // 获取当前机器人ID和对应点
    uint8_t getRobotId() const { return robot_id_; }
    geometry_msgs::msg::Point getSupplyPoint() const;  // 根据robot_id返回补给点
    geometry_msgs::msg::Point getAttackPoint() const;  // 根据robot_id返回攻击点

    // 状态数据
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
    double min_stay_duration = 3.0;

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
    std::shared_ptr<SentryControl> control_msg_;
    uint8_t last_stage_ = 0;
    uint8_t robot_id_ = 0;
};

#endif // BLACKBOARD_HPP
