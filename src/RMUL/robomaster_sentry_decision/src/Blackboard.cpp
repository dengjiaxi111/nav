#include "sentry_decision/Blackboard.hpp"
#include <iostream>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace SentryConstants;

Blackboard::Blackboard() {
    control_msg_ = std::make_shared<SentryControl>();
    control_msg_->gimbal_mode = GIMBAL_ENEMY;
    control_msg_->spin_mode = SPIN_OFF;
    resetAllPublishStates();

    // 初始化配置为默认值（来自 Constants.hpp）
    config_.red_attack = RED_ATTACK_POINT;
    config_.blue_attack = BLUE_ATTACK_POINT;
    config_.red_supply = RED_SUPPLY_POINT;
    config_.blue_supply = BLUE_SUPPLY_POINT;
    config_.arrival_wait_time = ARRIVAL_WAIT_TIME;
    config_.supply_threshold = SUPPLY_THRESHOLD;
    config_.hp_weight = 1.0;
    config_.ammo_weight = 0.0;
}

bool Blackboard::loadConfigFromYAML(const std::string& filepath) {
    try {
        YAML::Node config = YAML::LoadFile(filepath);

        if (config["red_attack_x"] && config["red_attack_y"]) {
            config_.red_attack.x = config["red_attack_x"].as<double>();
            config_.red_attack.y = config["red_attack_y"].as<double>();
        }
        if (config["blue_attack_x"] && config["blue_attack_y"]) {
            config_.blue_attack.x = config["blue_attack_x"].as<double>();
            config_.blue_attack.y = config["blue_attack_y"].as<double>();
        }
        if (config["red_supply_x"] && config["red_supply_y"]) {
            config_.red_supply.x = config["red_supply_x"].as<double>();
            config_.red_supply.y = config["red_supply_y"].as<double>();
        }
        if (config["blue_supply_x"] && config["blue_supply_y"]) {
            config_.blue_supply.x = config["blue_supply_x"].as<double>();
            config_.blue_supply.y = config["blue_supply_y"].as<double>();
        }
        if (config["arrival_wait_time"])
            config_.arrival_wait_time = config["arrival_wait_time"].as<double>();
        if (config["supply_threshold"])
            config_.supply_threshold = config["supply_threshold"].as<double>();
        if (config["deviation_threshold"])
            config_.deviation_threshold = config["deviation_threshold"].as<double>();
        if (config["hp_weight"])
            config_.hp_weight = config["hp_weight"].as<double>();
        if (config["ammo_weight"])
            config_.ammo_weight = config["ammo_weight"].as<double>();

        std::cout << "[CONFIG] 成功加载配置文件: " << filepath << std::endl;
        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "[CONFIG] 加载 YAML 失败: " << e.what() << std::endl;
        return false;
    }
}

geometry_msgs::msg::Point Blackboard::getAttackPoint() const {
    if (robot_id_ == 1) {
        return config_.blue_attack;
    } else {
        return config_.red_attack;
    }
}

geometry_msgs::msg::Point Blackboard::getSupplyPoint() const {
    if (robot_id_ == 1) {
        return config_.blue_supply;
    } else {
        return config_.red_supply;
    }
}

uint8_t Blackboard::getGimbalModeByStage() const {
    // 阶段 3: 五秒倒计时，阶段 4: 比赛中，云台打人；其他阶段云台静止
    if (stage == 3 || stage == 4) {
        return 1;   // 打人模式
    } else {
        return 0;   // 静止
    }
}

void Blackboard::resetForNewMatch() {
    std::cout << "[SYSTEM] 重置比赛状态" << std::endl;
    resetCurrentBehavior();
    resurrection_flag = false;
    at_current_target = false;
    target_arrival_time = -1.0;
    // 根据当前阶段设置云台模式
    updateControlMsg(getGimbalModeByStage(), SPIN_OFF);
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
    at_supply_point = std::sqrt(dx*dx + dy*dy) <= 30.0;
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

void Blackboard::updateControlMsg(uint8_t gimbal_mode, uint8_t spin_mode) {
    if (!control_msg_) return;
    control_msg_->gimbal_mode = gimbal_mode;
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

void Blackboard::onRobotIdChanged(uint8_t old_id, uint8_t new_id) {
    std::cout << "[SYSTEM] 机器人ID变化: " << (int)old_id << " -> " << (int)new_id << std::endl;
    setTargetPublished(false);

    geometry_msgs::msg::Point supply_point = getSupplyPoint();
    double dx = x - supply_point.x;
    double dy = y - supply_point.y;
    at_supply_point = std::sqrt(dx*dx + dy*dy) <= 30.0;
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
