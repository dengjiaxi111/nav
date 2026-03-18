#include "sentry_decision/bt/BTAction.hpp"
#include "sentry_decision/Models.hpp"
#include <iostream>

using namespace SentryConstants;

// SetEnergyActivationTarget 实现
BTStatus SetEnergyActivationTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                           std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    if (blackboard->current_behavior.type != BehaviorType::ACTIVATE_ENERGY) {
        blackboard->startBehavior(BehaviorType::ACTIVATE_ENERGY, RED_ENERGY_POINT, 10.0);
    }
    
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    return BTStatus::RUNNING;
}

// SetOutpostAttackTarget 实现
BTStatus SetOutpostAttackTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                        std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    if (!blackboard->energy_activated || blackboard->initialization_complete) {
        return BTStatus::FAILURE;
    }
    
    if (blackboard->current_behavior.type != BehaviorType::ATTACK_OUTPOST) {
        std::cout << "[INIT] 开始前往前哨站攻击点: (" 
                  << SentryConstants::ENEMY_OUTPOST_ATTACK.x/100.0 << ", " 
                  << SentryConstants::ENEMY_OUTPOST_ATTACK.y/100.0 << ")" << std::endl;
        blackboard->startBehavior(BehaviorType::ATTACK_OUTPOST, SentryConstants::ENEMY_OUTPOST_ATTACK, 10.0);
    }
    
    updateControlMsg(blackboard, SentryConstants::GIMBAL_IDLE, SentryConstants::SPIN_OFF, 
                     SentryConstants::POSTURE_MOVE, SentryConstants::RAMP_OFF);
    return BTStatus::RUNNING;
}

// SetBaseDefenseTarget 实现
BTStatus SetBaseDefenseTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                      std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    if (blackboard->current_behavior.type != BehaviorType::DEFEND_BASE) {
        blackboard->startBehavior(BehaviorType::DEFEND_BASE, RED_BASE_GAIN, 10.0);
    }
    
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    return BTStatus::RUNNING;
}

// ===== 修改点：设置补给行为持续时间为 10 秒 =====
BTStatus SetSupplyTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                 std::shared_ptr<RegionManager> /*region_manager*/) {
    if (!blackboard) return BTStatus::FAILURE;
    if (blackboard->current_behavior.type != BehaviorType::MOVE_TO_SUPPLY &&
        blackboard->current_behavior.type != BehaviorType::SUPPLY) {
        blackboard->startBehavior(BehaviorType::MOVE_TO_SUPPLY, RED_SUPPLY, 10.0);
    }
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    return BTStatus::RUNNING;
}

// ===== 修改点：设置复活行为持续时间为 10 秒 =====
BTStatus SetResurrectionTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                        std::shared_ptr<RegionManager> /*region_manager*/) {
    if (!blackboard) return BTStatus::FAILURE;
    if (blackboard->current_behavior.type != BehaviorType::RESURRECTION &&
        blackboard->current_behavior.type != BehaviorType::SUPPLY) {
        blackboard->startBehavior(BehaviorType::RESURRECTION, RED_SUPPLY, 10.0);
    }
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    return BTStatus::RUNNING;
}

// SetHeroAttackTarget::execute 函数
BTStatus SetHeroAttackTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                     std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard || !region_manager) return BTStatus::FAILURE;
    
    if (!blackboard->enemy_hero.visible || blackboard->enemy_hero.hp <= 0) {
        std::cout << "[HERO_ATTACK] 英雄不可见或已死亡" << std::endl;
        return BTStatus::FAILURE;
    }
    
    if (blackboard->isRampLockActive() || blackboard->isRampLockPending()) {
        std::cout << "[HERO_ATTACK] 飞坡锁激活，跳过英雄攻击判定" << std::endl;
        return BTStatus::FAILURE;
    }
    
    geometry_msgs::msg::Point attack_point = region_manager->findSameRegionHexPoint(
        blackboard->enemy_hero.x, blackboard->enemy_hero.y,
        blackboard->x, blackboard->y);
    
    bool need_ramp = (blackboard->x < SentryConstants::HALF_MAP_X && 
                     attack_point.x >= SentryConstants::HALF_MAP_X);
    
    if (need_ramp && blackboard->isRampLockInactive()) {
        std::cout << "[HERO_ATTACK] 需要飞坡，保存原始目标点: (" 
                  << attack_point.x << ", " << attack_point.y << ")" << std::endl;
        blackboard->original_target_before_ramp = attack_point;
        blackboard->activateRampLock();
        blackboard->ramp_in_process = true;
        blackboard->startBehavior(BehaviorType::RAMP_PROCESS, SentryConstants::RED_RAMP_POINT, 0.0);
        updateControlMsg(blackboard, SentryConstants::GIMBAL_IDLE, SentryConstants::SPIN_OFF, 
                         SentryConstants::POSTURE_MOVE, SentryConstants::RAMP_OFF);
        return BTStatus::RUNNING;
    } else {
        if (blackboard->current_behavior.type != BehaviorType::ATTACK_HERO ||
            blackboard->current_behavior.state == BehaviorState::COMPLETED) {
            std::cout << "[HERO_ATTACK] 开始/继续攻击英雄，英雄位置: (" 
                      << blackboard->enemy_hero.x << ", " << blackboard->enemy_hero.y << ")"
                      << "，目标点: (" << attack_point.x << ", " << attack_point.y << ")" 
                      << "，区域: " << region_manager->getRegionName(attack_point.x, attack_point.y) << std::endl;
            blackboard->startBehavior(BehaviorType::ATTACK_HERO, attack_point, 8.0);
        } else {
            double dx = blackboard->current_behavior.target.x - attack_point.x;
            double dy = blackboard->current_behavior.target.y - attack_point.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            if (distance > 50.0) {
                std::cout << "[HERO_ATTACK] 英雄移动，更新攻击目标点" << std::endl;
                blackboard->startBehavior(BehaviorType::ATTACK_HERO, attack_point, 8.0);
            }
        }
        
        bool in_deploy_zone = region_manager->isInHeroDeployZone(blackboard->enemy_hero.x, blackboard->enemy_hero.y);
        if (in_deploy_zone) {
            std::cout << "[HERO_ATTACK] 英雄在部署区，采用打部署区姿态" << std::endl;
            updateControlMsg(blackboard, SentryConstants::GIMBAL_ENEMY, SentryConstants::SPIN_VARIABLE, 
                             SentryConstants::POSTURE_ATTACK, SentryConstants::RAMP_OFF);
        } else {
            updateControlMsg(blackboard, SentryConstants::GIMBAL_ENEMY, SentryConstants::SPIN_VARIABLE, 
                             SentryConstants::POSTURE_MOVE, SentryConstants::RAMP_OFF);
        }
    }
    return BTStatus::RUNNING;
}

void SetHeroAttackTarget::reset() {
    // 空实现
}

// SetRobotAttackTarget 实现
BTStatus SetRobotAttackTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                      std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard || !region_manager) return BTStatus::FAILURE;
    
    EnemyInfo* enemy = blackboard->getEnemyById(target_id_);
    if (!enemy || !enemy->visible || enemy->hp <= 0) {
        return BTStatus::FAILURE;
    }
    
    if (blackboard->current_behavior.type != BehaviorType::ATTACK_HERO) {
        geometry_msgs::msg::Point attack_point = region_manager->findSameRegionHexPoint(
            enemy->x, enemy->y,
            blackboard->x, blackboard->y);
        blackboard->startBehavior(BehaviorType::ATTACK_HERO, attack_point, 8.0);
    }
    
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    return BTStatus::RUNNING;
}

// SetRampPointTarget 实现
BTStatus SetRampPointTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                    std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    if (blackboard->current_behavior.type != BehaviorType::MOVE_TO_FORTRESS) {
        blackboard->startBehavior(BehaviorType::MOVE_TO_FORTRESS, RED_RAMP_POINT, 1.0);
    }
    
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    return BTStatus::RUNNING;
}

// SetGainPointTarget 实现
BTStatus SetGainPointTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                    std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    if (blackboard->current_behavior.type != BehaviorType::DEFEND_GAIN_POINT) {
        blackboard->startBehavior(BehaviorType::DEFEND_GAIN_POINT, position_, 10.0);
    }
    
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    return BTStatus::RUNNING;
}

// SetFortressOccupyTarget 实现
BTStatus SetFortressOccupyTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                         std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    if (blackboard->current_behavior.type != BehaviorType::OCCUPY_FORTRESS) {
        blackboard->startBehavior(BehaviorType::OCCUPY_FORTRESS, ENEMY_FORTRESS_OCCUPY, 10.0);
    }
    
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    return BTStatus::RUNNING;
}
