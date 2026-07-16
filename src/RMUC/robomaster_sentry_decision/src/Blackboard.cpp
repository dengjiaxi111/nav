#include "sentry_decision/Blackboard.hpp"
#include "sentry_decision/GameConstants.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <algorithm>
#include <yaml-cpp/yaml.h>

using namespace GameConstants;

Blackboard::Blackboard()
    : enemy_hero("hero", "hero"),
      enemy_engineer("engineer", "engineer"),
      enemy_infantry3("infantry3", "infantry"),
      enemy_infantry4("infantry4", "infantry"),
      enemy_sentry("sentry", "sentry")
{
    control_msg_ = std::make_shared<SentryControl>();
    control_msg_->gimbal_mode = GIMBAL_IDLE;
    control_msg_->spin_mode = SPIN_OFF;
    control_msg_->posture = POSTURE_MOVE;
    control_msg_->target_yaw_deg = 0.0;
    control_msg_->target_yaw_valid = false;
    resetAllPublishStates();
}

bool Blackboard::loadConfigFromYAML(const std::string& filepath) {
    try {
        YAML::Node config = YAML::LoadFile(filepath);

        config_.red_patrol_A.x = config["red_patrol_A_x"].as<double>();
        config_.red_patrol_A.y = config["red_patrol_A_y"].as<double>();
        config_.red_patrol_B.x = config["red_patrol_B_x"].as<double>();
        config_.red_patrol_B.y = config["red_patrol_B_y"].as<double>();
        config_.red_patrol_C.x = config["red_patrol_C_x"].as<double>();
        config_.red_patrol_C.y = config["red_patrol_C_y"].as<double>();
        config_.blue_patrol_A.x = config["blue_patrol_A_x"].as<double>();
        config_.blue_patrol_A.y = config["blue_patrol_A_y"].as<double>();
        config_.blue_patrol_B.x = config["blue_patrol_B_x"].as<double>();
        config_.blue_patrol_B.y = config["blue_patrol_B_y"].as<double>();
        config_.blue_patrol_C.x = config["blue_patrol_C_x"].as<double>();
        config_.blue_patrol_C.y = config["blue_patrol_C_y"].as<double>();

        config_.red_protect_hero.x = config["red_protect_hero_x"].as<double>();
        config_.red_protect_hero.y = config["red_protect_hero_y"].as<double>();
        config_.blue_protect_hero.x = config["blue_protect_hero_x"].as<double>();
        config_.blue_protect_hero.y = config["blue_protect_hero_y"].as<double>();

        config_.red_supply.x = config["red_supply_x"].as<double>();
        config_.red_supply.y = config["red_supply_y"].as<double>();
        config_.blue_supply.x = config["blue_supply_x"].as<double>();
        config_.blue_supply.y = config["blue_supply_y"].as<double>();
        config_.red_initial_outpost.x = config["red_initial_outpost_x"].as<double>();
        config_.red_initial_outpost.y = config["red_initial_outpost_y"].as<double>();
        config_.blue_initial_outpost.x = config["blue_initial_outpost_x"].as<double>();
        config_.blue_initial_outpost.y = config["blue_initial_outpost_y"].as<double>();

        config_.arrival_wait_time = config["arrival_wait_time"].as<double>();
        config_.deviation_threshold = config["deviation_threshold"].as<double>();
        config_.patrol_A_stay_duration = config["patrol_A_stay_duration"].as<double>();
        config_.patrol_B_stay_duration = config["patrol_B_stay_duration"].as<double>();
        config_.patrol_C_stay_duration = config["patrol_C_stay_duration"].as<double>();
        config_.attack_duration = config["attack_duration"].as<double>();
        config_.defend_duration = config["defend_duration"].as<double>();
        config_.initial_outpost_end_remaining_time = config["initial_outpost_end_remaining_time"].as<double>();
        config_.supply_threshold = config["supply_threshold"].as<double>();
        config_.max_hp = config["max_hp"].as<double>();
        config_.max_ammo = config["max_ammo"].as<double>();

        return true;
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("sentry_decision.blackboard"),
                     "[CONFIG] YAML error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("sentry_decision.blackboard"),
                     "[CONFIG] error: %s", e.what());
        return false;
    }
}

#define SELECT_RED_BLUE(red_field, blue_field) \
    (isBlueRobotId(robot_id_) ? config_.blue_field : config_.red_field)

geometry_msgs::msg::Point Blackboard::getPatrolPointA() const { return SELECT_RED_BLUE(red_patrol_A, blue_patrol_A); }
geometry_msgs::msg::Point Blackboard::getPatrolPointB() const { return SELECT_RED_BLUE(red_patrol_B, blue_patrol_B); }
geometry_msgs::msg::Point Blackboard::getPatrolPointC() const { return SELECT_RED_BLUE(red_patrol_C, blue_patrol_C); }
geometry_msgs::msg::Point Blackboard::getProtectHeroPoint() const { return SELECT_RED_BLUE(red_protect_hero, blue_protect_hero); }
geometry_msgs::msg::Point Blackboard::getSupplyPoint() const { return SELECT_RED_BLUE(red_supply, blue_supply); }
geometry_msgs::msg::Point Blackboard::getInitialOutpostPoint() const { return SELECT_RED_BLUE(red_initial_outpost, blue_initial_outpost); }

#undef SELECT_RED_BLUE

double Blackboard::getArrivalWaitTime() const { return config_.arrival_wait_time; }
double Blackboard::getDeviationThreshold() const { return config_.deviation_threshold; }
double Blackboard::getPatrolStayDurationA() const { return config_.patrol_A_stay_duration; }
double Blackboard::getPatrolStayDurationB() const { return config_.patrol_B_stay_duration; }
double Blackboard::getPatrolStayDurationC() const { return config_.patrol_C_stay_duration; }
double Blackboard::getAttackDuration() const { return config_.attack_duration; }
double Blackboard::getDefendDuration() const { return config_.defend_duration; }
double Blackboard::getInitialOutpostEndRemainingTime() const { return config_.initial_outpost_end_remaining_time; }
double Blackboard::getSupplyThreshold() const { return config_.supply_threshold; }
double Blackboard::getMaxHp() const { return config_.max_hp; }
double Blackboard::getMaxAmmo() const { return config_.max_ammo; }

void Blackboard::updateOurState(const OurRobotState::SharedPtr msg) {
    if (!msg) return;
    current_hp = static_cast<double>(msg->current_hp);
    allowance_17mm = static_cast<double>(msg->allowance_17mm);
    uint8_t old_id = robot_id_;
    robot_id_ = msg->robot_id;
    if (old_id != robot_id_) onRobotIdChanged(old_id, robot_id_);

    hp_history_.emplace_back(current_time, current_hp);
    while (!hp_history_.empty() && current_time - hp_history_.front().first > 5.0) {
        hp_history_.pop_front();
    }
    double max_recent_hp = current_hp;
    for (const auto& sample : hp_history_) {
        max_recent_hp = std::max(max_recent_hp, sample.second);
    }
    under_attack_ = (max_recent_hp - current_hp) > 10.0;

    // 更新我方英雄坐标（m -> cm）
    our_hero_x = msg->hero_x * 100.0;
    our_hero_y = msg->hero_y * 100.0;

    if (current_hp <= 0 && !resurrection_flag) {
        resurrection_flag = true;
        at_current_target = false;
    }
    if (resurrection_flag && current_hp >= config_.max_hp) {
        resurrection_flag = false;
        at_current_target = false;
    }

    // 如果离开目标点过远，认为丢失目标
    if (at_current_target && !isAtTarget(current_behavior.target, config_.deviation_threshold * 2)) {
        at_current_target = false;
        target_arrival_time = -1.0;
    }
}

void Blackboard::updateEnemyState(const EnemyRobotState::SharedPtr msg) {
    if (!msg) return;
    auto update = [this](EnemyInfo& info, double x, double y, double hp, double allowance) {
        info.x = x;
        info.y = y;
        info.hp = hp;
        info.allowance = allowance;
        info.last_update_time = current_time;
    };
    update(enemy_hero, msg->enemy_hero_x, msg->enemy_hero_y, msg->enemy_hero_hp, msg->enemy_hero_allowance);
    update(enemy_engineer, msg->enemy_engineer_x, msg->enemy_engineer_y, msg->enemy_engineer_hp, 0.0);
    update(enemy_infantry3, msg->enemy_infantry3_x, msg->enemy_infantry3_y, msg->enemy_infantry3_hp, msg->enemy_infantry3_allowance);
    update(enemy_infantry4, msg->enemy_infantry4_x, msg->enemy_infantry4_y, msg->enemy_infantry4_hp, msg->enemy_infantry4_allowance);
    update(enemy_sentry, msg->enemy_sentry_x, msg->enemy_sentry_y, msg->enemy_sentry_hp, msg->enemy_sentry_allowance);

    enemy_hero.visible = false;
    enemy_engineer.visible = false;
    enemy_infantry3.visible = false;
    enemy_infantry4.visible = false;
    enemy_sentry.visible = false;

    const bool has_visual_lock =
        msg->enemy_id > 0 &&
        msg->enemy_x != 0.0 &&
        msg->enemy_y != 0.0;

    if (!has_visual_lock) return;

    switch (msg->enemy_id) {
    case 1:
        enemy_hero.visible = (enemy_hero.hp > 0);
        break;
    case 2:
        enemy_engineer.visible = (enemy_engineer.hp > 0);
        break;
    case 3:
        enemy_infantry3.visible = (enemy_infantry3.hp > 0);
        break;
    case 4:
        enemy_infantry4.visible = (enemy_infantry4.hp > 0);
        break;
    case 7:
        enemy_sentry.visible = (enemy_sentry.hp > 0);
        break;
    default:
        break;
    }
}

void Blackboard::updateGameState(const GameState::SharedPtr msg) {
    if (!msg) return;
    if (last_stage_ != msg->stage) {
        if (msg->stage == STAGE_BATTLE && last_stage_ != STAGE_BATTLE)
            resetForNewMatch();
        last_stage_ = msg->stage;
    }
    stage = msg->stage;
    stage_remaining_time = msg->stage_remaining_time;
    current_time = 420.0 - stage_remaining_time;
    base_open = msg->baseopen;
    outpost_alive = msg->outpost_alive;
}

void Blackboard::updatePositionFromTF(double x_m, double y_m, double yaw_rad) {
    x = x_m * 100.0;
    y = y_m * 100.0;
    robot_yaw = yaw_rad;
}

void Blackboard::resetForNewMatch() {
    resurrection_flag = false;
    under_attack_ = false;
    hp_history_.clear();
    at_current_target = false;
    target_arrival_time = -1.0;
    updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE);
    resetAllPublishStates();
}

bool Blackboard::isAtTarget(const geometry_msgs::msg::Point& target, double tolerance) const {
    double dx = x - target.x, dy = y - target.y;
    return std::sqrt(dx*dx + dy*dy) <= tolerance;
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

bool Blackboard::hasWaitedAtTarget(double wait_seconds) const {
    if (!at_current_target || target_arrival_time < 0) return false;
    return (current_time - target_arrival_time) >= wait_seconds;
}

void Blackboard::updateControlMsg(uint8_t gimbal_mode, uint8_t spin_mode, uint8_t posture) {
    control_msg_->gimbal_mode = gimbal_mode;
    control_msg_->spin_mode = spin_mode;
    control_msg_->posture = posture;
    control_msg_->target_yaw_valid = false;
}

uint8_t Blackboard::getBattleSpinMode() const {
    return under_attack_ ? SPIN_HIGH : SPIN_LOW;
}

void Blackboard::resetAllPublishStates() {
    current_behavior.target_published = false;
    current_behavior.control_published = false;
}

EnemyInfo* Blackboard::getEnemyById(const std::string& id) {
    if (id == "hero") return &enemy_hero;
    if (id == "engineer") return &enemy_engineer;
    if (id == "infantry3") return &enemy_infantry3;
    if (id == "infantry4") return &enemy_infantry4;
    if (id == "sentry") return &enemy_sentry;
    return nullptr;
}

const EnemyInfo* Blackboard::getEnemyById(const std::string& id) const {
    if (id == "hero") return &enemy_hero;
    if (id == "engineer") return &enemy_engineer;
    if (id == "infantry3") return &enemy_infantry3;
    if (id == "infantry4") return &enemy_infantry4;
    if (id == "sentry") return &enemy_sentry;
    return nullptr;
}

void Blackboard::onRobotIdChanged(uint8_t old_id, uint8_t new_id) {
    (void)old_id;
    (void)new_id;
    resetAllPublishStates();
}
