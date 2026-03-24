#include "sentry_decision/Blackboard.hpp"
#include <iostream>
#include <cmath>

using namespace SentryConstants;

Blackboard::Blackboard() {
    control_msg_ = std::make_shared<SentryControl>();
    control_msg_->spin_mode = SPIN_OFF;
    resetAllPublishStates();
}

void Blackboard::resetForNewMatch() {
    std::cout << "[SYSTEM] 重置比赛状态" << std::endl;
    resetCurrentBehavior();
    resurrection_flag = false;
    at_current_target = false;
    target_arrival_time = -1.0;
    updateControlMsg(SPIN_OFF);
    resetAllPublishStates();
}

void Blackboard::updateOurState(const OurRobotState::SharedPtr msg) {
    if (!msg) return;
    current_hp = static_cast<double>(msg->current_hp);
    allowance_17mm = static_cast<double>(msg->allowance_17mm);

    uint8_t old_id = robot_id_;
    robot_id_ = msg->robot_id;
    if (old_id != robot_id_) {
        onRobotIdChanged(old_id, robot_id_);
    }

    if (current_hp <= 0 && !resurrection_flag) {
        resurrection_flag = true;
        at_current_target = false;
        resetCurrentBehavior();
    }

    if (resurrection_flag && current_hp >= 400.0) {
        resurrection_flag = false;
        at_current_target = false;
    }

    if (at_current_target && shouldLeaveTarget()) {
        at_current_target = false;
        target_arrival_time = -1.0;
        if (current_behavior.state == BehaviorState::EXECUTING)
            completeCurrentBehavior();
    }
}

void Blackboard::updatePositionFromTF(double x_m, double y_m) {
    x = x_m * 100.0;
    y = y_m * 100.0;

    geometry_msgs::msg::Point supply_point = getSupplyPoint();
    double dx = x - supply_point.x;
    double dy = y - supply_point.y;
    at_supply_point = std::sqrt(dx*dx + dy*dy) <= 50.0;
}

void Blackboard::updateGameState(const GameState::SharedPtr msg) {
    if (!msg) return;

    if (last_stage_ != msg->stage) {
        if (msg->stage == STAGE_BATTLE && last_stage_ != STAGE_BATTLE)
            resetForNewMatch();
        else if (last_stage_ == STAGE_BATTLE && msg->stage != STAGE_BATTLE)
            resetCurrentBehavior();
        last_stage_ = msg->stage;
    }

    stage = msg->stage;
    stage_remaining_time = msg->stage_remaining_time;
    current_time = 420.0 - stage_remaining_time;
}

bool Blackboard::isAtTarget(const geometry_msgs::msg::Point& target, double tolerance) const {
    double dx = x - target.x;
    double dy = y - target.y;
    return std::sqrt(dx*dx + dy*dy) <= tolerance;
}

void Blackboard::updateControlMsg(uint8_t spin_mode) {
    if (!control_msg_) return;
    control_msg_->spin_mode = spin_mode;
    setControlUpdated(true);
}

void Blackboard::setTargetReached(bool reached) {
    if (reached && !at_current_target) {
        at_current_target = true;
        target_arrival_time = current_time;
    } else if (!reached) {
        at_current_target = false;
        target_arrival_time = -1.0;
    }
}

bool Blackboard::shouldLeaveTarget() const {
    if (!at_current_target) return true;
    if (target_arrival_time < 0) return false;

    if (current_behavior.type == BehaviorType::MOVE_TO_ATTACK &&
        current_behavior.state == BehaviorState::EXECUTING) {
        return false;
    }

    if (current_behavior.type == BehaviorType::SUPPLY ||
        current_behavior.type == BehaviorType::RESURRECTION) {
        return false;
    }

    double stay_duration = current_time - target_arrival_time;

    if (resurrection_flag) return true;

    if (current_behavior.state == BehaviorState::EXECUTING &&
        current_behavior.execution_start_time > 0) {
        double exec_time = current_time - current_behavior.execution_start_time;
        if (exec_time >= current_behavior.execution_duration && current_behavior.execution_duration > 0) {
            return true;
        }
    }

    return stay_duration >= min_stay_duration;
}

bool Blackboard::hasWaitedAtTarget(double wait_seconds) const {
    if (!at_current_target || target_arrival_time < 0) return false;
    return (current_time - target_arrival_time) >= wait_seconds;
}

geometry_msgs::msg::Point Blackboard::getSupplyPoint() const {
    if (robot_id_ == 1) { // 蓝方
        return BLUE_SUPPLY_POINT;
    } else { // 红方或默认
        return RED_SUPPLY_POINT;
    }
}

geometry_msgs::msg::Point Blackboard::getAttackPoint() const {
    if (robot_id_ == 1) { // 蓝方
        return BLUE_ATTACK_POINT;
    } else { // 红方或默认
        return RED_ATTACK_POINT;
    }
}

void Blackboard::onRobotIdChanged(uint8_t old_id, uint8_t new_id) {
    std::cout << "[SYSTEM] 机器人ID变化: " << (int)old_id << " -> " << (int)new_id << std::endl;
    setTargetPublished(false);

    geometry_msgs::msg::Point supply_point = getSupplyPoint();
    double dx = x - supply_point.x;
    double dy = y - supply_point.y;
    at_supply_point = std::sqrt(dx*dx + dy*dy) <= 50.0;
}

void Blackboard::startBehavior(BehaviorType type, const geometry_msgs::msg::Point& target, double duration) {
    std::cout << "[DECISION] 开始行为: ";
    switch(type) {
        case BehaviorType::MOVE_TO_ATTACK: std::cout << "前往攻击点"; break;
        case BehaviorType::MOVE_TO_SUPPLY: std::cout << "前往补给点"; break;
        case BehaviorType::SUPPLY:         std::cout << "补给中"; break;
        case BehaviorType::RESURRECTION:   std::cout << "复活"; break;
        default: std::cout << "未知"; break;
    }
    std::cout << std::endl;

    current_behavior.type = type;
    current_behavior.state = BehaviorState::MOVING;
    current_behavior.target = target;
    current_behavior.start_time = current_time;
    current_behavior.execution_start_time = -1.0;
    current_behavior.execution_duration = duration;
    at_current_target = false;
    target_arrival_time = -1.0;
    resetAllPublishStates();
}

void Blackboard::updateBehaviorState(BehaviorState state) {
    if (current_behavior.state != state) {
        current_behavior.state = state;
        if (state == BehaviorState::EXECUTING) {
            setControlPublished(false);
            setControlUpdated(false);
        }
    }
}

void Blackboard::startExecutionTime() {
    current_behavior.execution_start_time = current_time;
}

void Blackboard::completeCurrentBehavior() {
    current_behavior.state = BehaviorState::COMPLETED;
    resetAllPublishStates();
    at_current_target = false;
    target_arrival_time = -1.0;
}

void Blackboard::resetCurrentBehavior() {
    current_behavior.type = BehaviorType::NONE;
    current_behavior.state = BehaviorState::IDLE;
    current_behavior.start_time = -1.0;
    current_behavior.execution_start_time = -1.0;
    current_behavior.execution_duration = 0.0;
    at_current_target = false;
    target_arrival_time = -1.0;
    resetAllPublishStates();
}
