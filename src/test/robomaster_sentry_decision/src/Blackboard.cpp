#include "sentry_decision/Blackboard.hpp"
#include "sentry_decision/Constants.hpp"
#include "sentry_decision/Models.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace SentryConstants;

// 初始化静态映射表
std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> Blackboard::LOGICAL_TO_REAL_MAP;

Blackboard::Blackboard() 
    : enemy_hero("hero", "hero"),
      enemy_engineer("engineer", "engineer"),
      enemy_infantry3("infantry3", "infantry"),
      enemy_infantry4("infantry4", "infantry"),
      enemy_sentry("sentry", "sentry") {
    
    control_msg_ = std::make_shared<SentryControl>();
    control_msg_->gimbal_mode = SentryConstants::GIMBAL_IDLE;
    control_msg_->spin_mode = SentryConstants::SPIN_OFF;
    control_msg_->posture = SentryConstants::POSTURE_MOVE;
    control_msg_->ramp_mode = SentryConstants::RAMP_OFF;
    
    original_target_before_ramp.x = 0.0;
    original_target_before_ramp.y = 0.0;
    original_target_before_ramp.z = 0.0;
    
    initializeGainPoints();
    
    at_current_target = false;
    target_arrival_time = -1.0;
    
    current_behavior = BehaviorInfo();
    resetAllPublishStates();
    
    ramp_lock_state_ = RAMP_LOCK_INACTIVE;
    
   // ===== 初始化逻辑坐标到真实坐标的映射表（预留三个） =====
    geometry_msgs::msg::Point logical, real;
    
    // 映射1：逻辑(1.0,1.0) -> 真实(1152.0,992.0) cm
    logical.x = -580.0; logical.y = -130.0;
    real.x = 1152.0; real.y = 992.0;
    LOGICAL_TO_REAL_MAP.push_back({logical, real});
    
    // 映射2：逻辑(2.0,2.0) -> 真实(1642.0,1146.0) cm
    logical.x = -190.0; logical.y = -380.0;
    real.x = 1642.0; real.y = 1146.0;
    LOGICAL_TO_REAL_MAP.push_back({logical, real});
    
    // 映射3：逻辑(3.0,3.0) -> 真实(698,756) cm
    logical.x = -90.0; logical.y = -30.0;
    real.x = 698.0; real.y = 756.0;
    LOGICAL_TO_REAL_MAP.push_back({logical, real});
    
     
    logical.x = -480.0; logical.y = 120.0;
    real.x = 176.0; real.y = 262.0;
    LOGICAL_TO_REAL_MAP.push_back({logical, real});
}

// ===== 新增：逻辑坐标转换为真实坐标 =====
geometry_msgs::msg::Point Blackboard::convertToReal(const geometry_msgs::msg::Point& logical) const {
    // 遍历映射表，查找匹配的逻辑坐标
    for (const auto& entry : LOGICAL_TO_REAL_MAP) {
        double dx = entry.first.x - logical.x;
        double dy = entry.first.y - logical.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist < 0.001) {  // 允许微小浮点误差
            return entry.second;  // 返回对应的真实坐标
        }
    }
    // 未匹配，原样返回
    return logical;
}

void Blackboard::resetForNewMatch() {
    std::cout << "[SYSTEM] 重置比赛状态" << std::endl;
    resetCurrentBehavior();
    initialization_complete = false;
    energy_activated = false;  
    outpost_destroyed_init = false;
    resurrection_flag = false;
    at_current_target = false;
    target_arrival_time = -1.0;
    at_ramp_point = false;
    ramp_mode_active = false;
    ramp_in_process = false;
    original_target_before_ramp.x = 0.0;
    original_target_before_ramp.y = 0.0;
    ramp_lock_state_ = RAMP_LOCK_INACTIVE;
    
    enemy_hero.visible = false;
    enemy_engineer.visible = false;
    enemy_infantry3.visible = false;
    enemy_infantry4.visible = false;
    enemy_sentry.visible = false;
    
    updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    resetAllPublishStates();
    std::cout << "[SYSTEM] 比赛状态重置完成" << std::endl;
}

void Blackboard::updateOurState(const OurRobotState::SharedPtr msg) {
    if (!msg) return;
    
    current_hp = static_cast<double>(msg->current_hp);
    allowance_17mm = static_cast<double>(msg->allowance_17mm);
    x = msg->x * 100.0;
    y = msg->y * 100.0;
    rfid_status = msg->rfid_status;
    
    our_base_hp = static_cast<double>(msg->base_hp);
    our_outpost_hp = static_cast<double>(msg->outpost_hp);
    
    if (current_hp <= 0 && !resurrection_flag) {
        resurrection_flag = true;
        at_current_target = false;
        resetCurrentBehavior();
    }
    
    double dx = x - SentryConstants::RED_SUPPLY.x;
    double dy = y - SentryConstants::RED_SUPPLY.y;
    at_supply_point = std::sqrt(dx*dx + dy*dy) <= 50.0;
    
    bool rfid_bit19 = checkRFIDBit(19);
    bool rfid_bit20 = checkRFIDBit(20);
    supply_rfid_detected = rfid_bit19 || rfid_bit20;
    
    if (resurrection_flag && current_hp >= 400.0 && supply_rfid_detected) {
        resurrection_flag = false;
        supply_start_time = -1.0;
        at_current_target = false;
    }
    
    if (at_supply_point && supply_rfid_detected && current_hp < 400.0) {
        if (supply_start_time < 0) supply_start_time = current_time;
    } else {
        supply_start_time = -1.0;
    }
    
    if (at_current_target && shouldLeaveTarget()) {
        at_current_target = false;
        target_arrival_time = -1.0;
        if (current_behavior.state == BehaviorState::EXECUTING) {
            completeCurrentBehavior();
        }
    }
}

void Blackboard::updateEnemyState(const EnemyRobotState::SharedPtr msg) {
    if (!msg) return;
    
    updateEnemyInfo(enemy_hero, msg->enemy_hero_x , msg->enemy_hero_y ,
                   static_cast<double>(msg->enemy_hero_hp), static_cast<double>(msg->enemy_hero_allowance));
    updateEnemyInfo(enemy_engineer, msg->enemy_engineer_x , msg->enemy_engineer_y ,
                   static_cast<double>(msg->enemy_engineer_hp), 0.0);
    updateEnemyInfo(enemy_infantry3, msg->enemy_infantry3_x , msg->enemy_infantry3_y ,
                   static_cast<double>(msg->enemy_infantry3_hp), static_cast<double>(msg->enemy_infantry3_allowance));
    updateEnemyInfo(enemy_infantry4, msg->enemy_infantry4_x , msg->enemy_infantry4_y ,
                   static_cast<double>(msg->enemy_infantry4_hp), static_cast<double>(msg->enemy_infantry4_allowance));
    updateEnemyInfo(enemy_sentry, msg->enemy_sentry_x , msg->enemy_sentry_y ,
                   static_cast<double>(msg->enemy_sentry_hp), static_cast<double>(msg->enemy_sentry_allowance));
    
    if (current_time - hero_last_update_time > 0.1) {
        double dx = std::abs(enemy_hero.x - hero_last_x);
        double dy = std::abs(enemy_hero.y - hero_last_y);
        if (dx < 10.0 && dy < 10.0) {
            hero_static_start_time += (current_time - hero_last_update_time);
            if (hero_static_start_time > 1.0) {
                hero_in_deploy_zone = (enemy_hero.x >= 2194 && enemy_hero.x <= 2756 &&
                                      enemy_hero.y >= 48 && enemy_hero.y <= 942);
            }
        } else {
            hero_static_start_time = 0;
            hero_in_deploy_zone = false;
        }
        hero_last_x = enemy_hero.x;
        hero_last_y = enemy_hero.y;
        hero_last_update_time = current_time;
    }
}

void Blackboard::updateGameState(const GameState::SharedPtr msg) {
    if (!msg) return;
    
    if (last_stage_ != msg->stage) {
        if (msg->stage == STAGE_BATTLE && last_stage_ != STAGE_BATTLE) {
            resetForNewMatch();
        } else if (last_stage_ == STAGE_BATTLE && msg->stage != STAGE_BATTLE) {
            resetCurrentBehavior();
            at_current_target = false;
            target_arrival_time = -1.0;
        }
        last_stage_ = msg->stage;
    }
    
    stage = msg->stage;
    stage_remaining_time = msg->stage_remaining_time;
    current_time = 420.0 - stage_remaining_time;
    
    energy_mechanism_activatable = (msg->energy_mechanism_activatable == 1);
    large_energy_mechanism_activation = msg->large_energy_mechanism_activation;
    
    base_gain_point_occupied = (msg->base_gain_point_occupation == 1);
    trapezoid_highland_occupied = (msg->trapezoid_highland_occupation == 1);
    
    uint8_t fortress_occupation = msg->fortress_gain_point_occupation;
    fortress_gain_point_occupied_by_us = (fortress_occupation == 1);
    fortress_gain_point_occupied_by_enemy = (fortress_occupation == 2);
    
    uint8_t outpost_occupation = msg->outpost_gain_point_occupation;
    outpost_gain_point_occupied_by_us = (outpost_occupation == 1);
    outpost_gain_point_occupied_by_enemy = (outpost_occupation == 2);
    
    uint8_t central_occupation = msg->central_highland_occupation;
    central_highland_occupied_by_us = (central_occupation == 1);
    central_highland_occupied_by_enemy = (central_occupation == 2);
    
    updateGainPointStatus();
}

void Blackboard::updateEnemyInfo(EnemyInfo& info, double x, double y, double hp, double allowance) {
    info.x = x;
    info.y = y;
    info.hp = hp;
    info.allowance = allowance;
    info.visible = (x > 0 && y > 0);
    info.last_update_time = current_time;
}

std::vector<EnemyInfo*> Blackboard::getVisibleEnemies() {
    std::vector<EnemyInfo*> enemies;
    if (enemy_hero.visible && enemy_hero.hp > 0) enemies.push_back(&enemy_hero);
    if (enemy_engineer.visible && enemy_engineer.hp > 0) enemies.push_back(&enemy_engineer);
    if (enemy_infantry3.visible && enemy_infantry3.hp > 0) enemies.push_back(&enemy_infantry3);
    if (enemy_infantry4.visible && enemy_infantry4.hp > 0) enemies.push_back(&enemy_infantry4);
    if (enemy_sentry.visible && enemy_sentry.hp > 0) enemies.push_back(&enemy_sentry);
    return enemies;
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

bool Blackboard::checkRFIDBit(int bit_pos) const {
    if (bit_pos < 0 || bit_pos >= 32) return false;
    return ((rfid_status >> bit_pos) & 1) == 1;
}

// ===== 修改 isAtTarget：先转换目标点为真实坐标，再比较 =====
bool Blackboard::isAtTarget(const geometry_msgs::msg::Point& target, double tolerance) const {
    geometry_msgs::msg::Point real_target = convertToReal(target);
    double dx = x - real_target.x;
    double dy = y - real_target.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    return distance <= tolerance;
}

void Blackboard::updateControlMsg(uint8_t gimbal_mode, uint8_t spin_mode, 
                                 uint8_t posture, uint8_t ramp_mode) {
    if (!control_msg_) return;
    control_msg_->gimbal_mode = gimbal_mode;
    control_msg_->spin_mode = spin_mode;
    control_msg_->posture = posture;
    control_msg_->ramp_mode = ramp_mode;
    setControlUpdated(true);
}

void Blackboard::initializeGainPoints() {
    gain_points.clear();
    
    GainPointStatus base_gain;
    base_gain.name = "base_gain";
    base_gain.position = SentryConstants::RED_BASE_GAIN;
    base_gain.defense_gain = 1.0;
    base_gain.occupied_by_us = false;
    base_gain.occupied_by_enemy = false;
    base_gain.neutral = true;
    gain_points.push_back(base_gain);
    
    GainPointStatus trapezoid_gain;
    trapezoid_gain.name = "trapezoid_highland_gain";
    trapezoid_gain.position = SentryConstants::TRAPEZOID_HIGHLAND_GAIN;
    trapezoid_gain.defense_gain = 1.0;
    trapezoid_gain.occupied_by_us = false;
    trapezoid_gain.occupied_by_enemy = false;
    trapezoid_gain.neutral = true;
    gain_points.push_back(trapezoid_gain);
    
    GainPointStatus fortress_gain;
    fortress_gain.name = "fortress_gain";
    fortress_gain.position = SentryConstants::RED_FORTRESS;
    fortress_gain.defense_gain = 1.0;
    fortress_gain.occupied_by_us = false;
    fortress_gain.occupied_by_enemy = false;
    fortress_gain.neutral = true;
    gain_points.push_back(fortress_gain);
    
    GainPointStatus central_gain;
    central_gain.name = "central_highland_gain";
    central_gain.position = SentryConstants::CENTRAL_HIGHLAND_GAIN;
    central_gain.defense_gain = 0.5;
    central_gain.occupied_by_us = false;
    central_gain.occupied_by_enemy = false;
    central_gain.neutral = true;
    gain_points.push_back(central_gain);
    
    GainPointStatus outpost_gain;
    outpost_gain.name = "outpost_gain";
    outpost_gain.position = SentryConstants::RED_OUTPOST_GAIN;
    outpost_gain.defense_gain = 0.5;
    outpost_gain.occupied_by_us = false;
    outpost_gain.occupied_by_enemy = false;
    outpost_gain.neutral = true;
    gain_points.push_back(outpost_gain);
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
        } else if (gp.name == "central_highland_gain") {
            gp.occupied_by_us = central_highland_occupied_by_us;
            gp.occupied_by_enemy = central_highland_occupied_by_enemy;
            gp.neutral = !(central_highland_occupied_by_us || central_highland_occupied_by_enemy);
        } else if (gp.name == "outpost_gain") {
            gp.occupied_by_us = outpost_gain_point_occupied_by_us;
            gp.occupied_by_enemy = outpost_gain_point_occupied_by_enemy;
            gp.neutral = !(outpost_gain_point_occupied_by_us || outpost_gain_point_occupied_by_enemy);
        }
    }
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
    if (current_behavior.type == BehaviorType::RAMP_PROCESS) return false;

    // ===== 关键修改：补给和复活行为永不主动离开（与省赛一致） =====
    if (current_behavior.type == BehaviorType::SUPPLY ||
        current_behavior.type == BehaviorType::RESURRECTION) {
        return false;
    }
    // 初始化关键行为必须由 DecisionManager 的 checkBehaviorCompletion() 结束，
    // 否则 energy_activated / initialization_complete 等标志不会被置位。
    if (current_behavior.type == BehaviorType::ACTIVATE_ENERGY ||
        current_behavior.type == BehaviorType::ATTACK_OUTPOST) {
        return false;
    }
    // =============================================================

    if (target_arrival_time < 0) return false;
    double stay_duration = current_time - target_arrival_time;
    if (resurrection_flag) return true;
    if (our_base_hp < 1500) return true;
    if (current_hp < 100.0 || allowance_17mm < 50.0) return true;
    if (!current_target_id.empty()) {
        const EnemyInfo* target = getEnemyById(current_target_id);
        if (!target || target->hp <= 0) return true;
    }
    if (stay_duration < min_stay_duration) return false;
    if (current_behavior.state == BehaviorState::EXECUTING) {
        double execution_duration = current_time - current_behavior.execution_start_time;
        if (execution_duration >= current_behavior.execution_duration) return true;
    }
    return false;
}



// ===== 修改 startBehavior：保存原始目标，同时计算真实目标用于内部比较 =====
void Blackboard::startBehavior(BehaviorType type, const geometry_msgs::msg::Point& target, double duration) {
    // 计算真实坐标
    geometry_msgs::msg::Point real_target = convertToReal(target);
    
    // 检查是否与当前行为相同（基于真实坐标）
    if (type == current_behavior.type && 
        current_behavior.state != BehaviorState::COMPLETED) {
        double dx = current_behavior.real_target.x - real_target.x;
        double dy = current_behavior.real_target.y - real_target.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        if (distance < 10.0) {
            return;  // 相同行为且目标点相近，不重新启动
        }
    }
    
    // 输出开始新行为日志（简化）
    if (type != current_behavior.type || current_behavior.state == BehaviorState::COMPLETED) {
        std::cout << "[DECISION] 开始新行为: ";
        switch(type) {
            case BehaviorType::ACTIVATE_ENERGY: std::cout << "激活能量机关 (10秒)" << std::endl; break;
            case BehaviorType::ATTACK_OUTPOST: std::cout << "攻击前哨站 (10秒)" << std::endl; break;
            case BehaviorType::RESURRECTION: std::cout << "复活" << std::endl; break;
            case BehaviorType::SUPPLY: std::cout << "补给" << std::endl; break;
            case BehaviorType::DEFEND_BASE: std::cout << "基地防御 (10秒)" << std::endl; break;
            case BehaviorType::ATTACK_HERO: std::cout << "攻击英雄 (8秒)" << std::endl; break;
            case BehaviorType::DEFEND_GAIN_POINT: std::cout << "防守增益点 (10秒)" << std::endl; break;
            case BehaviorType::OCCUPY_FORTRESS: std::cout << "占领堡垒 (10秒)" << std::endl; break;
            case BehaviorType::MOVE_TO_FORTRESS: std::cout << "前往飞坡点" << std::endl; break;
            case BehaviorType::RAMP_PROCESS: std::cout << "飞坡流程" << std::endl; break;
            default: std::cout << "未知行为" << std::endl;
        }
    }
    
    // 保存行为信息
    current_behavior.type = type;
    current_behavior.state = BehaviorState::MOVING;
    current_behavior.target = target;                // 原始目标（用于发布）
    current_behavior.real_target = real_target;      // 真实目标（用于内部计算）
    current_behavior.start_time = current_time;
    current_behavior.execution_start_time = -1.0;
    current_behavior.execution_duration = duration;
    
    // 重置目标点到达状态
    at_current_target = false;
    target_arrival_time = -1.0;
    
    // 重置发布状态
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
    if (current_behavior.type == BehaviorType::RAMP_PROCESS) {
        deactivateRampLock();
    }
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
    current_behavior.target.x = 0.0;
    current_behavior.target.y = 0.0;
    current_behavior.real_target.x = 0.0;
    current_behavior.real_target.y = 0.0;
    
    at_current_target = false;
    target_arrival_time = -1.0;
    resetAllPublishStates();
}
