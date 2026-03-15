#include "sentry_decision/Blackboard.hpp"
#include "sentry_decision/Constants.hpp"
#include "sentry_decision/Models.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace SentryConstants;

Blackboard::Blackboard() 
    : enemy_hero("hero", "hero"),
      enemy_engineer("engineer", "engineer"),
      enemy_infantry3("infantry3", "infantry"),
      enemy_infantry4("infantry4", "infantry"),
      enemy_sentry("sentry", "sentry") {
    
    // 初始化控制消息
    control_msg_ = std::make_shared<SentryControl>();
    control_msg_->gimbal_mode = SentryConstants::GIMBAL_IDLE;
    control_msg_->spin_mode = SentryConstants::SPIN_OFF;
    control_msg_->posture = SentryConstants::POSTURE_MOVE;
    control_msg_->ramp_mode = SentryConstants::RAMP_OFF;
    
    // 初始化原始目标点
    original_target_before_ramp.x = 0.0;
    original_target_before_ramp.y = 0.0;
    original_target_before_ramp.z = 0.0;
    
    // 初始化增益点
    initializeGainPoints();
    
    // 初始化目标点状态
    at_current_target = false;
    target_arrival_time = -1.0;
    
    // 初始化行为状态
    current_behavior = BehaviorInfo();
    resetAllPublishStates();
    
    // 初始化飞坡锁状态
    ramp_lock_state_ = RAMP_LOCK_INACTIVE;
}

void Blackboard::resetForNewMatch() {
    std::cout << "[SYSTEM] 重置比赛状态" << std::endl;
    
    // 重置行为状态
    resetCurrentBehavior();
    
    // 重置初始化状态
    initialization_complete = false;
    energy_activated = false;  
    outpost_destroyed_init = false;
    
    // 重置复活状态
    resurrection_flag = false;
    
    // 重置目标点状态
    at_current_target = false;
    target_arrival_time = -1.0;
    
    // 重置飞坡相关状态
    at_ramp_point = false;
    ramp_mode_active = false;
    ramp_in_process = false;
    original_target_before_ramp.x = 0.0;
    original_target_before_ramp.y = 0.0;
    
    // 重置飞坡锁状态
    ramp_lock_state_ = RAMP_LOCK_INACTIVE;
    
    // 重置敌人可见性
    enemy_hero.visible = false;
    enemy_engineer.visible = false;
    enemy_infantry3.visible = false;
    enemy_infantry4.visible = false;
    enemy_sentry.visible = false;
    
    // 重置控制消息
    updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    
    // 重置所有发布状态
    resetAllPublishStates();
    
    std::cout << "[SYSTEM] 比赛状态重置完成" << std::endl;
}

void Blackboard::updateOurState(const OurRobotState::SharedPtr msg) {
    if (!msg) return;
    
    // 自身状态
    current_hp = static_cast<double>(msg->current_hp);
    allowance_17mm = static_cast<double>(msg->allowance_17mm);
    x = msg->x * 100.0;  // 转换为cm
    y = msg->y * 100.0;  // 转换为cm
    rfid_status = msg->rfid_status;
    
    // 基地和前哨站血量
    our_base_hp = static_cast<double>(msg->base_hp);
    our_outpost_hp = static_cast<double>(msg->outpost_hp);
    
    // 检查是否死亡
    if (current_hp <= 0 && !resurrection_flag) {
        resurrection_flag = true;
        at_current_target = false;  // 死亡重置目标状态
        // 重置当前行为
        resetCurrentBehavior();
    }
    
    // 检查是否在补给点
    double dx = x - SentryConstants::RED_SUPPLY.x;
    double dy = y - SentryConstants::RED_SUPPLY.y;
    at_supply_point = std::sqrt(dx*dx + dy*dy) <= 50.0;
    
    // 检查RFID状态
    bool rfid_bit19 = checkRFIDBit(19);
    bool rfid_bit20 = checkRFIDBit(20);
    supply_rfid_detected = rfid_bit19 || rfid_bit20;
    
    // 检查是否复活完成
    if (resurrection_flag && current_hp >= 400.0 && supply_rfid_detected) {
        resurrection_flag = false;
        supply_start_time = -1.0;
        at_current_target = false;  // 复活重置目标状态
    }
    
    // 如果正在补给，更新补给开始时间
    if (at_supply_point && supply_rfid_detected && current_hp < 400.0) {
        if (supply_start_time < 0) {
            supply_start_time = current_time;
        }
    } else {
        supply_start_time = -1.0;
    }
    
    // 检查是否到达当前目标点
    if (at_current_target) {
        // 如果已经到达目标点，检查是否需要离开
        if (shouldLeaveTarget()) {
            at_current_target = false;
            target_arrival_time = -1.0;
            // 行为完成后重置
            if (current_behavior.state == BehaviorState::EXECUTING) {
                completeCurrentBehavior();
            }
        }
    }
}

void Blackboard::updateEnemyState(const EnemyRobotState::SharedPtr msg) {
    if (!msg) return;
    
    // 更新英雄机器人
    updateEnemyInfo(enemy_hero, 
                   msg->enemy_hero_x ,  
                   msg->enemy_hero_y ,
                   static_cast<double>(msg->enemy_hero_hp),
                   static_cast<double>(msg->enemy_hero_allowance));
    
    // 更新工程机器人
    updateEnemyInfo(enemy_engineer,
                   msg->enemy_engineer_x ,
                   msg->enemy_engineer_y ,
                   static_cast<double>(msg->enemy_engineer_hp), 0.0);
    
    // 更新步兵3机器人
    updateEnemyInfo(enemy_infantry3,
                   msg->enemy_infantry3_x ,
                   msg->enemy_infantry3_y ,
                   static_cast<double>(msg->enemy_infantry3_hp),
                   static_cast<double>(msg->enemy_infantry3_allowance));
    
    // 更新步兵4机器人
    updateEnemyInfo(enemy_infantry4,
                   msg->enemy_infantry4_x ,
                   msg->enemy_infantry4_y ,
                   static_cast<double>(msg->enemy_infantry4_hp),
                   static_cast<double>(msg->enemy_infantry4_allowance));
    
    // 更新哨兵机器人
    updateEnemyInfo(enemy_sentry,
                   msg->enemy_sentry_x ,
                   msg->enemy_sentry_y ,
                   static_cast<double>(msg->enemy_sentry_hp),
                   static_cast<double>(msg->enemy_sentry_allowance));
    
    // 跟踪英雄部署状态
    if (current_time - hero_last_update_time > 0.1) {  // 100ms
        double dx = std::abs(enemy_hero.x - hero_last_x);
        double dy = std::abs(enemy_hero.y - hero_last_y);
        
        if (dx < 10.0 && dy < 10.0) {  // 移动小于10cm
            hero_static_start_time += (current_time - hero_last_update_time);
            if (hero_static_start_time > 1.0) {  // 静止超过1秒
                // 检查是否在部署区域 (2752,942) (2756,48) (2194,48) (2194,942)
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
    
    // 检查比赛是否重新开始 - 恢复到原始代码，但添加额外的检查
    if (last_stage_ != msg->stage) {
        // 只有当从非比赛阶段进入比赛阶段时才重置状态
        // 避免在比赛过程中重复重置
        if (msg->stage == STAGE_BATTLE && last_stage_ != STAGE_BATTLE) {
            // 比赛从其他阶段进入比赛阶段，重置状态
            resetForNewMatch();
        }
        // 从比赛阶段退出时，也要重置一些状态
        else if (last_stage_ == STAGE_BATTLE && msg->stage != STAGE_BATTLE) {
            // 比赛结束，停止所有行为
            resetCurrentBehavior();
            // 重置目标点状态
            at_current_target = false;
            target_arrival_time = -1.0;
        }
        last_stage_ = msg->stage;
    }
    
    stage = msg->stage;
    stage_remaining_time = msg->stage_remaining_time;
    current_time = 420.0 - stage_remaining_time;  // 比赛开始后经过的时间
    
    // 能量机关状态
    energy_mechanism_activatable = (msg->energy_mechanism_activatable == 1);
    large_energy_mechanism_activation = msg->large_energy_mechanism_activation;
    
    // 增益点占领状态
    base_gain_point_occupied = (msg->base_gain_point_occupation == 1);
    trapezoid_highland_occupied = (msg->trapezoid_highland_occupation == 1);
    
    // 堡垒增益点
    uint8_t fortress_occupation = msg->fortress_gain_point_occupation;
    fortress_gain_point_occupied_by_us = (fortress_occupation == 1);
    fortress_gain_point_occupied_by_enemy = (fortress_occupation == 2);
    
    // 前哨站增益点
    uint8_t outpost_occupation = msg->outpost_gain_point_occupation;
    outpost_gain_point_occupied_by_us = (outpost_occupation == 1);
    outpost_gain_point_occupied_by_enemy = (outpost_occupation == 2);
    
    // 中央高地
    uint8_t central_occupation = msg->central_highland_occupation;
    central_highland_occupied_by_us = (central_occupation == 1);
    central_highland_occupied_by_enemy = (central_occupation == 2);
    
    // 更新增益点状态
    updateGainPointStatus();
}

void Blackboard::updateEnemyInfo(EnemyInfo& info, double x, double y, double hp, double allowance) {
    info.x = x;
    info.y = y;
    info.hp = hp;
    info.allowance = allowance;
    info.visible = (x > 0 && y > 0);  // 简单可见性判断
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

bool Blackboard::isAtTarget(const geometry_msgs::msg::Point& target, double tolerance) const {
    double dx = x - target.x;
    double dy = y - target.y;
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
    
    // 标记控制消息已更新
    setControlUpdated(true);
}

void Blackboard::initializeGainPoints() {
    gain_points.clear();
    
    // 基地增益点 (50%防御增益)
    GainPointStatus base_gain;
    base_gain.name = "base_gain";
    base_gain.position = SentryConstants::RED_BASE_GAIN;
    base_gain.defense_gain = 1.0;  // 对应50%防御增益
    base_gain.occupied_by_us = false;
    base_gain.occupied_by_enemy = false;
    base_gain.neutral = true;
    gain_points.push_back(base_gain);
    
    // 梯形高地增益点 (50%防御增益)
    GainPointStatus trapezoid_gain;
    trapezoid_gain.name = "trapezoid_highland_gain";
    trapezoid_gain.position = SentryConstants::TRAPEZOID_HIGHLAND_GAIN;
    trapezoid_gain.defense_gain = 1.0;  // 对应50%防御增益
    trapezoid_gain.occupied_by_us = false;
    trapezoid_gain.occupied_by_enemy = false;
    trapezoid_gain.neutral = true;
    gain_points.push_back(trapezoid_gain);
    
    // 己方堡垒增益点 (50%防御增益)
    GainPointStatus fortress_gain;
    fortress_gain.name = "fortress_gain";
    fortress_gain.position = SentryConstants::RED_FORTRESS;
    fortress_gain.defense_gain = 1.0;
    fortress_gain.occupied_by_us = false;
    fortress_gain.occupied_by_enemy = false;
    fortress_gain.neutral = true;
    gain_points.push_back(fortress_gain);
    
    // 中央高地增益点 (25%防御增益)
    GainPointStatus central_gain;
    central_gain.name = "central_highland_gain";
    central_gain.position = SentryConstants::CENTRAL_HIGHLAND_GAIN;
    central_gain.defense_gain = 0.5;  // 对应25%防御增益
    central_gain.occupied_by_us = false;
    central_gain.occupied_by_enemy = false;
    central_gain.neutral = true;
    gain_points.push_back(central_gain);
    
    // 前哨站增益点 (25%防御增益)
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

// 修改 Blackboard::shouldLeaveTarget() 函数，添加飞坡流程的特殊处理
bool Blackboard::shouldLeaveTarget() const {
    if (!at_current_target) return true;
    
    // 关键修改：飞坡流程不应该被提前离开
    if (current_behavior.type == BehaviorType::RAMP_PROCESS) {
        return false;
    }
    
    // 如果已经到达目标点，检查是否应该离开
    if (target_arrival_time < 0) return false;
    
    double stay_duration = current_time - target_arrival_time;
    
    // 检查复活状态
    if (resurrection_flag) return true;
    
    // 检查关键事件
    if (our_base_hp < 1500) return true;
    
    // 检查补给需求
    if (current_hp < 100.0 || allowance_17mm < 50.0) return true;
    
    // 检查攻击目标是否死亡
    if (!current_target_id.empty()) {
        const EnemyInfo* target = getEnemyById(current_target_id);
        if (!target || target->hp <= 0) return true;
    }
    
    // 最少停留时间
    if (stay_duration < min_stay_duration) return false;
    
    // 根据行为类型决定停留时间
    if (current_behavior.state == BehaviorState::EXECUTING) {
        double execution_duration = current_time - current_behavior.execution_start_time;
        if (execution_duration >= current_behavior.execution_duration) {
            return true;  // 行为执行时间达到
        }
    }
    
    return false;
}

void Blackboard::startBehavior(BehaviorType type, const geometry_msgs::msg::Point& target, double duration) {
    // 关键修复：即使类型相同，如果目标点不同或者行为已完成，也要重新启动
    if (type == current_behavior.type && 
        current_behavior.state != BehaviorState::COMPLETED) {
        // 检查目标点是否相同（允许10cm的误差）
        double dx = current_behavior.target.x - target.x;
        double dy = current_behavior.target.y - target.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < 10.0) {
            // 目标和类型都相同，且行为未完成，不重新启动
            return;
        }
    }
    
    // 精简输出：只显示开始新行为
    if (type != current_behavior.type || current_behavior.state == BehaviorState::COMPLETED) {
        std::cout << "[DECISION] 开始新行为: ";
        switch(type) {
            case BehaviorType::ACTIVATE_ENERGY:
                std::cout << "激活能量机关 (10秒)" << std::endl;
                break;
            case BehaviorType::ATTACK_OUTPOST:
                std::cout << "攻击前哨站 (10秒)" << std::endl;
                break;
            case BehaviorType::RESURRECTION:
                std::cout << "复活" << std::endl;
                break;
            case BehaviorType::SUPPLY:
                std::cout << "补给" << std::endl;
                break;
            case BehaviorType::DEFEND_BASE:
                std::cout << "基地防御 (10秒)" << std::endl;
                break;
            case BehaviorType::ATTACK_HERO:
                std::cout << "攻击英雄 (8秒)" << std::endl;
                break;
            case BehaviorType::DEFEND_GAIN_POINT:
                std::cout << "防守增益点 (10秒)" << std::endl;
                break;
            case BehaviorType::OCCUPY_FORTRESS:
                std::cout << "占领堡垒 (10秒)" << std::endl;
                break;
            case BehaviorType::MOVE_TO_FORTRESS:
                std::cout << "前往飞坡点" << std::endl;
                break;
            case BehaviorType::RAMP_PROCESS:
                std::cout << "飞坡流程" << std::endl;
                break;
            default:
                std::cout << "未知行为" << std::endl;
        }
    }
    
    // 重置行为状态
    current_behavior.type = type;
    current_behavior.state = BehaviorState::MOVING;
    current_behavior.target = target;
    current_behavior.start_time = current_time;
    current_behavior.execution_start_time = -1.0;  // 重置执行开始时间
    current_behavior.execution_duration = duration;
    
    // 重置目标点状态
    at_current_target = false;
    target_arrival_time = -1.0;
    
    // 重置发布状态
    resetAllPublishStates();
}

void Blackboard::updateBehaviorState(BehaviorState state) {
    if (current_behavior.state != state) {
        current_behavior.state = state;
        
        // 状态改变时重置控制发布状态，确保能发布新的控制消息
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
    // 如果是飞坡流程完成，解除飞坡锁
    if (current_behavior.type == BehaviorType::RAMP_PROCESS) {
        deactivateRampLock();
    }
    
    current_behavior.state = BehaviorState::COMPLETED;
    resetAllPublishStates();
    
    // 确保at_current_target被重置
    at_current_target = false;
    target_arrival_time = -1.0;
}

void Blackboard::resetCurrentBehavior() {
    // 重置行为状态
    current_behavior.type = BehaviorType::NONE;
    current_behavior.state = BehaviorState::IDLE;
    current_behavior.start_time = -1.0;
    current_behavior.execution_start_time = -1.0;
    current_behavior.execution_duration = 0.0;
    
    // 重置目标点状态
    at_current_target = false;
    target_arrival_time = -1.0;
    
    // 重置发布状态
    resetAllPublishStates();
}
