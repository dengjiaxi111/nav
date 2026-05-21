#include "sentry_decision/Blackboard.hpp"
#include "sentry_decision/GameConstants.hpp"
#include <iostream>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

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

        config_.init_pre_attack.x = config["init_pre_attack_x"] ? config["init_pre_attack_x"].as<double>() : 850.0;
        config_.init_pre_attack.y = config["init_pre_attack_y"] ? config["init_pre_attack_y"].as<double>() : 96.0;
        config_.red_attack.x = config["red_attack_x"].as<double>();
        config_.red_attack.y = config["red_attack_y"].as<double>();
        config_.blue_attack.x = config["blue_attack_x"].as<double>();
        config_.blue_attack.y = config["blue_attack_y"].as<double>();
        config_.red_supply.x = config["red_supply_x"].as<double>();
        config_.red_supply.y = config["red_supply_y"].as<double>();
        config_.blue_supply.x = config["blue_supply_x"].as<double>();
        config_.blue_supply.y = config["blue_supply_y"].as<double>();
        config_.red_base_gain.x = config["red_base_gain_x"].as<double>();
        config_.red_base_gain.y = config["red_base_gain_y"].as<double>();
        config_.blue_base_gain.x = config["blue_base_gain_x"].as<double>();
        config_.blue_base_gain.y = config["blue_base_gain_y"].as<double>();
        config_.red_fortress_occupy.x = config["red_fortress_occupy_x"].as<double>();
        config_.red_fortress_occupy.y = config["red_fortress_occupy_y"].as<double>();
        config_.blue_fortress_occupy.x = config["blue_fortress_occupy_x"].as<double>();
        config_.blue_fortress_occupy.y = config["blue_fortress_occupy_y"].as<double>();
        config_.red_fortress_gain.x = config["red_fortress_gain_x"].as<double>();
        config_.red_fortress_gain.y = config["red_fortress_gain_y"].as<double>();
        config_.blue_fortress_gain.x = config["blue_fortress_gain_x"].as<double>();
        config_.blue_fortress_gain.y = config["blue_fortress_gain_y"].as<double>();
        // 中央高地增益点坐标已删除，不再读取
        config_.trapezoid_highland_gain.x = config["trapezoid_highland_gain_x"].as<double>();
        config_.trapezoid_highland_gain.y = config["trapezoid_highland_gain_y"].as<double>();
        config_.red_enemy_outpost.x = config["red_enemy_outpost_x"] ? config["red_enemy_outpost_x"].as<double>() : 0.0;
        config_.red_enemy_outpost.y = config["red_enemy_outpost_y"] ? config["red_enemy_outpost_y"].as<double>() : 0.0;
        config_.blue_enemy_outpost.x = config["blue_enemy_outpost_x"] ? config["blue_enemy_outpost_x"].as<double>() : 0.0;
        config_.blue_enemy_outpost.y = config["blue_enemy_outpost_y"] ? config["blue_enemy_outpost_y"].as<double>() : 0.0;

        config_.red_enemy_fortress.x = config["red_enemy_fortress_x"] ? config["red_enemy_fortress_x"].as<double>() : 2112.0;
        config_.red_enemy_fortress.y = config["red_enemy_fortress_y"] ? config["red_enemy_fortress_y"].as<double>() : 754.0;
        config_.blue_enemy_fortress.x = config["blue_enemy_fortress_x"] ? config["blue_enemy_fortress_x"].as<double>() : 694.0;
        config_.blue_enemy_fortress.y = config["blue_enemy_fortress_y"] ? config["blue_enemy_fortress_y"].as<double>() : 756.0;

        config_.red_main_decision.x = config["red_main_decision_x"] ? config["red_main_decision_x"].as<double>() : 1400.0;
        config_.red_main_decision.y = config["red_main_decision_y"] ? config["red_main_decision_y"].as<double>() : 750.0;
        config_.blue_main_decision.x = config["blue_main_decision_x"] ? config["blue_main_decision_x"].as<double>() : 1400.0;
        config_.blue_main_decision.y = config["blue_main_decision_y"] ? config["blue_main_decision_y"].as<double>() : 750.0;
        config_.decision_mode = config["decision_mode"] ? config["decision_mode"].as<int>() : 0;
        config_.red_simple_decision.x = config["red_simple_decision_x"] ? config["red_simple_decision_x"].as<double>() : config_.red_main_decision.x;
        config_.red_simple_decision.y = config["red_simple_decision_y"] ? config["red_simple_decision_y"].as<double>() : config_.red_main_decision.y;
        config_.blue_simple_decision.x = config["blue_simple_decision_x"] ? config["blue_simple_decision_x"].as<double>() : config_.blue_main_decision.x;
        config_.blue_simple_decision.y = config["blue_simple_decision_y"] ? config["blue_simple_decision_y"].as<double>() : config_.blue_main_decision.y;

        config_.arrival_wait_time = config["arrival_wait_time"].as<double>();
        config_.deviation_threshold = config["deviation_threshold"].as<double>();
        config_.enemy_chase_repath_threshold =
            config["enemy_chase_repath_threshold"] ? config["enemy_chase_repath_threshold"].as<double>() : 100.0;
        config_.init_attack_duration = config["init_attack_duration"].as<double>();
        config_.attack_duration = config["attack_duration"].as<double>();
        config_.defend_duration = config["defend_duration"].as<double>();

        config_.hp_weight = config["hp_weight"].as<double>();
        config_.ammo_weight = config["ammo_weight"].as<double>();
        config_.base_weight = config["base_weight"].as<double>();

        config_.supply_threshold = config["supply_threshold"].as<double>();
        config_.max_hp = config["max_hp"].as<double>();
        config_.max_ammo = config["max_ammo"].as<double>();

        config_.hero_attack_z_threshold = config["hero_attack_z_threshold"].as<double>();
        config_.hero_attack_h_threshold = config["hero_attack_h_threshold"].as<double>();
        config_.hero_high_priority_z_threshold = config["hero_high_priority_z_threshold"].as<double>();
        config_.target_selection_z_high = config["target_selection_z_high"].as<double>();
        config_.target_selection_z_mid = config["target_selection_z_mid"].as<double>();
        config_.target_selection_threshold_high = config["target_selection_threshold_high"].as<double>();
        config_.target_selection_threshold_mid = config["target_selection_threshold_mid"].as<double>();
        config_.target_selection_threshold_low = config["target_selection_threshold_low"].as<double>();
        config_.gain_point_z_high = config["gain_point_z_high"].as<double>();
        config_.gain_point_z_mid = config["gain_point_z_mid"].as<double>();
        config_.gain_point_threshold_high = config["gain_point_threshold_high"].as<double>();
        config_.gain_point_threshold_mid = config["gain_point_threshold_mid"].as<double>();
        config_.gain_point_threshold_low = config["gain_point_threshold_low"].as<double>();
        config_.fortress_occupy_z_threshold = config["fortress_occupy_z_threshold"].as<double>();
        config_.fortress_occupy_f_threshold = config["fortress_occupy_f_threshold"].as<double>();
        config_.fortress_occupy_hp_ratio = config["fortress_occupy_hp_ratio"].as<double>();
        config_.enemy_fortress_occupy_time = config["enemy_fortress_occupy_time"] ? config["enemy_fortress_occupy_time"].as<double>() : 180.0;
        config_.enemy_fortress_hp_threshold = config["enemy_fortress_hp_threshold"] ? config["enemy_fortress_hp_threshold"].as<double>() : 0.7;
        config_.enemy_fortress_ammo_threshold = config["enemy_fortress_ammo_threshold"] ? config["enemy_fortress_ammo_threshold"].as<double>() : 0.7;

        priority_targets_config_.clear();
        if (config["priority_targets"]) {
            for (const auto& node : config["priority_targets"]) {
                PriorityConfig pc;
                pc.type = node["type"].as<std::string>();
                pc.weight_hp = node["weight_hp"].as<double>();
                pc.weight_distance = node["weight_distance"].as<double>();
                pc.weight_ammo = node["weight_ammo"].as<double>();
                pc.threshold = node["threshold"].as<double>();
                priority_targets_config_.push_back(pc);
            }
        }

        config_.red_patrol.x = config["red_patrol_x"] ? config["red_patrol_x"].as<double>() : 1958.0;
        config_.red_patrol.y = config["red_patrol_y"] ? config["red_patrol_y"].as<double>() : 1304.0;
        config_.blue_patrol.x = config["blue_patrol_x"] ? config["blue_patrol_x"].as<double>() : 846.0;
        config_.blue_patrol.y = config["blue_patrol_y"] ? config["blue_patrol_y"].as<double>() : 200.0;
        config_.patrol_stay_duration = config["patrol_stay_duration"] ? config["patrol_stay_duration"].as<double>() : 10.0;

        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "[CONFIG] YAML loading failed: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "[CONFIG] Missing config: " << e.what() << std::endl;
        return false;
    }
}

geometry_msgs::msg::Point Blackboard::getInitPreAttackPoint() const { return config_.init_pre_attack; }
geometry_msgs::msg::Point Blackboard::getAttackPoint() const { return (robot_id_ == 1) ? config_.blue_attack : config_.red_attack; }
geometry_msgs::msg::Point Blackboard::getSupplyPoint() const { return (robot_id_ == 1) ? config_.blue_supply : config_.red_supply; }
geometry_msgs::msg::Point Blackboard::getBaseGainPoint() const { return (robot_id_ == 1) ? config_.blue_base_gain : config_.red_base_gain; }
geometry_msgs::msg::Point Blackboard::getFortressOccupyPoint() const { return (robot_id_ == 1) ? config_.blue_fortress_occupy : config_.red_fortress_occupy; }
geometry_msgs::msg::Point Blackboard::getEnemyFortressPoint() const { return (robot_id_ == 1) ? config_.blue_enemy_fortress : config_.red_enemy_fortress; }
geometry_msgs::msg::Point Blackboard::getFortressGainPoint() const { return (robot_id_ == 1) ? config_.blue_fortress_gain : config_.red_fortress_gain; }
// getCentralHighlandGain() 已删除
geometry_msgs::msg::Point Blackboard::getTrapezoidHighlandGain() const { return config_.trapezoid_highland_gain; }
geometry_msgs::msg::Point Blackboard::getEnemyOutpostPoint() const { return (robot_id_ == 1) ? config_.blue_enemy_outpost : config_.red_enemy_outpost; }
geometry_msgs::msg::Point Blackboard::getMainDecisionPoint() const { return (robot_id_ == 1) ? config_.blue_main_decision : config_.red_main_decision; }
geometry_msgs::msg::Point Blackboard::getSimpleDecisionPoint() const { return (robot_id_ == 1) ? config_.blue_simple_decision : config_.red_simple_decision; }

geometry_msgs::msg::Point Blackboard::getPatrolPoint() const {
    return (robot_id_ == 1) ? config_.blue_patrol : config_.red_patrol;
}

int Blackboard::getDecisionMode() const { return config_.decision_mode; }
double Blackboard::getArrivalWaitTime() const { return config_.arrival_wait_time; }
double Blackboard::getDeviationThreshold() const { return config_.deviation_threshold; }
double Blackboard::getEnemyChaseRepathThreshold() const { return config_.enemy_chase_repath_threshold; }
double Blackboard::getInitAttackDuration() const { return config_.init_attack_duration; }
double Blackboard::getAttackDuration() const { return config_.attack_duration; }
double Blackboard::getDefendDuration() const { return config_.defend_duration; }
double Blackboard::getSupplyThreshold() const { return config_.supply_threshold; }
double Blackboard::getMaxHp() const { return config_.max_hp; }
double Blackboard::getMaxAmmo() const { return config_.max_ammo; }
double Blackboard::getEnemyFortressOccupyTime() const { return config_.enemy_fortress_occupy_time; }
double Blackboard::getEnemyFortressHpThreshold() const { return config_.enemy_fortress_hp_threshold; }
double Blackboard::getEnemyFortressAmmoThreshold() const { return config_.enemy_fortress_ammo_threshold; }
double Blackboard::getHpWeight() const { return config_.hp_weight; }
double Blackboard::getAmmoWeight() const { return config_.ammo_weight; }
double Blackboard::getBaseWeight() const { return config_.base_weight; }
double Blackboard::getHeroAttackZThreshold() const { return config_.hero_attack_z_threshold; }
double Blackboard::getHeroAttackHThreshold() const { return config_.hero_attack_h_threshold; }
double Blackboard::getHeroHighPriorityZThreshold() const { return config_.hero_high_priority_z_threshold; }
double Blackboard::getTargetSelectionZHigh() const { return config_.target_selection_z_high; }
double Blackboard::getTargetSelectionZMid() const { return config_.target_selection_z_mid; }
double Blackboard::getTargetSelectionThresholdHigh() const { return config_.target_selection_threshold_high; }
double Blackboard::getTargetSelectionThresholdMid() const { return config_.target_selection_threshold_mid; }
double Blackboard::getTargetSelectionThresholdLow() const { return config_.target_selection_threshold_low; }
double Blackboard::getGainPointZHigh() const { return config_.gain_point_z_high; }
double Blackboard::getGainPointZMid() const { return config_.gain_point_z_mid; }
double Blackboard::getGainPointThresholdHigh() const { return config_.gain_point_threshold_high; }
double Blackboard::getGainPointThresholdMid() const { return config_.gain_point_threshold_mid; }
double Blackboard::getGainPointThresholdLow() const { return config_.gain_point_threshold_low; }
double Blackboard::getFortressOccupyZThreshold() const { return config_.fortress_occupy_z_threshold; }
double Blackboard::getFortressOccupyFThreshold() const { return config_.fortress_occupy_f_threshold; }
double Blackboard::getFortressOccupyHpRatio() const { return config_.fortress_occupy_hp_ratio; }
double Blackboard::getPatrolStayDuration() const { return config_.patrol_stay_duration; }

void Blackboard::updateOurState(const OurRobotState::SharedPtr msg) {
    if (!msg) return;
    if (current_behavior.type == BehaviorType::INIT_ATTACK &&
        current_behavior.state == BehaviorState::EXECUTING &&
        current_behavior.execution_start_time >= 0.0) {
        init_attack_elapsed_time = std::min(getInitAttackDuration(),
                                            init_attack_elapsed_time + getExecutionElapsedTime());
        current_behavior.execution_start_time = current_time;
    }

    current_hp = static_cast<double>(msg->current_hp);
    allowance_17mm = static_cast<double>(msg->allowance_17mm);
    our_base_hp = static_cast<double>(msg->base_hp);
    our_outpost_hp = static_cast<double>(msg->outpost_hp);

    uint8_t old_id = robot_id_;
    robot_id_ = msg->robot_id;
    if (old_id != robot_id_) onRobotIdChanged(old_id, robot_id_);

    if (current_hp <= 0 && !resurrection_flag) {
        resurrection_flag = true;
        at_current_target = false;
        resetCurrentBehavior();
    }

    if (resurrection_flag && current_hp >= config_.max_hp) {
        resurrection_flag = false;
        at_current_target = false;
    }

    geometry_msgs::msg::Point supply = getSupplyPoint();
    double dx = x - supply.x, dy = y - supply.y;
    at_supply_point = std::sqrt(dx*dx + dy*dy) <= 30.0;

    if (at_current_target && shouldLeaveTarget()) {
        at_current_target = false;
        target_arrival_time = -1.0;
        if (current_behavior.state == BehaviorState::EXECUTING)
            completeCurrentBehavior();
    }
}

void Blackboard::updateEnemyState(const EnemyRobotState::SharedPtr msg) {
    if (!msg) return;
    auto update = [this](EnemyInfo& info, double x, double y, double hp, double allowance) {
        info.x = x; info.y = y; info.hp = hp; info.allowance = allowance;
        info.visible = (x > 0 && y > 0);
        info.last_update_time = current_time;
    };
    update(enemy_hero, msg->enemy_hero_x, msg->enemy_hero_y, msg->enemy_hero_hp, msg->enemy_hero_allowance);
    update(enemy_engineer, msg->enemy_engineer_x, msg->enemy_engineer_y, msg->enemy_engineer_hp, 0.0);
    update(enemy_infantry3, msg->enemy_infantry3_x, msg->enemy_infantry3_y, msg->enemy_infantry3_hp, msg->enemy_infantry3_allowance);
    update(enemy_infantry4, msg->enemy_infantry4_x, msg->enemy_infantry4_y, msg->enemy_infantry4_hp, msg->enemy_infantry4_allowance);
    update(enemy_sentry, msg->enemy_sentry_x, msg->enemy_sentry_y, msg->enemy_sentry_hp, msg->enemy_sentry_allowance);
    enemy_fortress_gain_point_occupation = msg->enemy_fortress_gain_point_occupation;
    if (msg->enemy_fortress_gain_point_occupation == 2 ||
        msg->enemy_fortress_gain_point_occupation == 3) {
        enemy_fortress_gain_point_captured_by_us = true;
    }
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

    base_gain_point_occupied = (msg->base_gain_point_occupation == 1);
    trapezoid_highland_occupied = (msg->trapezoid_highland_occupation == 1);
    fortress_gain_point_occupied_by_us = (msg->fortress_gain_point_occupation == 1);
    fortress_gain_point_occupied_by_enemy = (msg->fortress_gain_point_occupation == 2);
    outpost_gain_point_occupied_by_us = (msg->outpost_gain_point_occupation == 1);
    outpost_gain_point_occupied_by_enemy = (msg->outpost_gain_point_occupation == 2);
    central_highland_occupied_by_us = (msg->central_highland_occupation == 1);
    central_highland_occupied_by_enemy = (msg->central_highland_occupation == 2);
    updateGainPointStatus();

    base_open = msg->baseopen;
    enemy_outpost_destroyed = (msg->outpoststate == 1);
}

void Blackboard::updatePositionFromTF(double x_m, double y_m, double yaw_rad) {
    x = x_m * 100.0;
    y = y_m * 100.0;
    robot_yaw = yaw_rad;
}

void Blackboard::resetForNewMatch() {
    resetCurrentBehavior();
    initialization_complete = false;
    init_attack_elapsed_time = 0.0;
    resurrection_flag = false;
    at_current_target = false;
    target_arrival_time = -1.0;
    must_occupy_enemy_fortress = false;
    enemy_fortress_gain_point_captured_by_us = false;
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

bool Blackboard::shouldLeaveTarget() const {
    if (!at_current_target) return true;
    if (target_arrival_time < 0) return false;
    if (current_behavior.type == BehaviorType::MOVE_TO_SUPPLY && current_behavior.state == BehaviorState::MOVING) return false;
    if (current_behavior.state == BehaviorState::EXECUTING && current_behavior.execution_start_time > 0) {
        double exec_time = current_time - current_behavior.execution_start_time;
        if (exec_time >= current_behavior.execution_duration && current_behavior.execution_duration > 0)
            return true;
    }
    double stay = current_time - target_arrival_time;
    return stay >= config_.arrival_wait_time;
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
    setControlUpdated(true);
}

void Blackboard::startBehavior(BehaviorType type, const geometry_msgs::msg::Point& target, double duration) {
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

bool Blackboard::isBehaviorInProgress() const {
    return current_behavior.state != BehaviorState::IDLE && current_behavior.state != BehaviorState::COMPLETED;
}

double Blackboard::getExecutionElapsedTime() const {
    if (current_behavior.execution_start_time < 0) return 0.0;
    return current_time - current_behavior.execution_start_time;
}

void Blackboard::resetAllPublishStates() {
    setTargetPublished(false);
    setControlPublished(false);
    setControlUpdated(false);
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

void Blackboard::initializeGainPoints() {
    gain_points.clear();
    auto add = [this](const std::string& name, const geometry_msgs::msg::Point& pos, double gain) {
        GainPointStatus gp;
        gp.name = name;
        gp.position = pos;
        gp.defense_gain = gain;
        gp.occupied_by_us = false;
        gp.occupied_by_enemy = false;
        gp.neutral = true;
        gain_points.push_back(gp);
    };
    add("base_gain", getBaseGainPoint(), 1.0);
    // add("trapezoid_highland_gain", getTrapezoidHighlandGain(), 1.0);
    // 中央高地增益点已删除，不再添加
    add("outpost_gain", getAttackPoint(), 0.5);
}

void Blackboard::updateGainPointStatus() {
    for (auto& gp : gain_points) {
        if (gp.name == "base_gain") {
            gp.occupied_by_us = base_gain_point_occupied;
            gp.neutral = !base_gain_point_occupied;
        } else if (gp.name == "trapezoid_highland_gain") {
            gp.occupied_by_us = trapezoid_highland_occupied;
            gp.neutral = !trapezoid_highland_occupied;
        } else if (gp.name == "fortress_gain") {
            gp.occupied_by_us = fortress_gain_point_occupied_by_us;
            gp.occupied_by_enemy = fortress_gain_point_occupied_by_enemy;
            gp.neutral = !(fortress_gain_point_occupied_by_us || fortress_gain_point_occupied_by_enemy);
        } else if (gp.name == "outpost_gain") {
            gp.occupied_by_us = outpost_gain_point_occupied_by_us;
            gp.occupied_by_enemy = outpost_gain_point_occupied_by_enemy;
            gp.neutral = !(outpost_gain_point_occupied_by_us || outpost_gain_point_occupied_by_enemy);
        }
        // 中央高地不再存在于 gain_points 中，因此无需处理
    }
}

void Blackboard::onRobotIdChanged(uint8_t old_id, uint8_t new_id) {
    (void)old_id;
    (void)new_id;
    setTargetPublished(false);
    initializeGainPoints();
}
