#include "sentry_decision/bt/BTAction.hpp"
#include "sentry_decision/Models.hpp"
#include <iostream>

using namespace SentryConstants;

// SetResurrectionTarget 实现 - 保持不变
BTStatus SetResurrectionTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                       std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    // 防止重复启动相同行为
    if (blackboard->current_behavior.type != BehaviorType::RESURRECTION) {
        blackboard->startBehavior(BehaviorType::RESURRECTION, RED_SUPPLY, 0.0);
    }
    
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    
    return BTStatus::RUNNING;
}

// SetEnergyActivationTarget 实现 - 保持不变
BTStatus SetEnergyActivationTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                           std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    // 防止重复启动相同行为
    if (blackboard->current_behavior.type != BehaviorType::ACTIVATE_ENERGY) {
        blackboard->startBehavior(BehaviorType::ACTIVATE_ENERGY, RED_ENERGY_POINT, 10.0);
    }
    
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    
    return BTStatus::RUNNING;
}

// SetOutpostAttackTarget 实现 - 保持不变
BTStatus SetOutpostAttackTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                        std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    // 只有在初始化阶段才执行这个节点
    if (!blackboard->energy_activated || blackboard->initialization_complete) {
        return BTStatus::FAILURE;
    }
    
    // 防止重复启动相同行为
    if (blackboard->current_behavior.type != BehaviorType::ATTACK_OUTPOST) {
        std::cout << "[INIT] 开始前往前哨站攻击点: (" 
                  << SentryConstants::ENEMY_OUTPOST_ATTACK.x/100.0 << ", " 
                  << SentryConstants::ENEMY_OUTPOST_ATTACK.y/100.0 << ")" << std::endl;
        blackboard->startBehavior(BehaviorType::ATTACK_OUTPOST, SentryConstants::ENEMY_OUTPOST_ATTACK, 10.0);
    }
    
    // 关键修改：设置移动姿态
    updateControlMsg(blackboard, 
                    SentryConstants::GIMBAL_IDLE, 
                    SentryConstants::SPIN_OFF, 
                    SentryConstants::POSTURE_MOVE,  // 移动姿态
                    SentryConstants::RAMP_OFF);
    
    return BTStatus::RUNNING;
}

// SetBaseDefenseTarget 实现 - 保持不变
BTStatus SetBaseDefenseTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                      std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    // 防止重复启动相同行为
    if (blackboard->current_behavior.type != BehaviorType::DEFEND_BASE) {
        blackboard->startBehavior(BehaviorType::DEFEND_BASE, RED_BASE_GAIN, 10.0);
    }
    
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    
    return BTStatus::RUNNING;
}

// SetSupplyTarget 实现 - 保持不变
BTStatus SetSupplyTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                 std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    // 防止重复启动相同行为
    if (blackboard->current_behavior.type != BehaviorType::SUPPLY) {
        blackboard->startBehavior(BehaviorType::SUPPLY, RED_SUPPLY, 0.0);
    }
    
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    
    return BTStatus::RUNNING;
}

// 修改 SetHeroAttackTarget::execute 函数
BTStatus SetHeroAttackTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                     std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard || !region_manager) return BTStatus::FAILURE;
    
    // 检查英雄是否可见
    if (!blackboard->enemy_hero.visible || blackboard->enemy_hero.hp <= 0) {
        std::cout << "[HERO_ATTACK] 英雄不可见或已死亡" << std::endl;
        return BTStatus::FAILURE;
    }
    
    // 关键修改：检查飞坡锁状态，如果处于激活状态，直接返回失败
    // 这样可以防止在飞坡过程中重复触发攻击英雄
    if (blackboard->isRampLockActive() || blackboard->isRampLockPending()) {
        std::cout << "[HERO_ATTACK] 飞坡锁激活，跳过英雄攻击判定" << std::endl;
        return BTStatus::FAILURE;
    }
    
    // 计算攻击点
    geometry_msgs::msg::Point attack_point = region_manager->findSameRegionHexPoint(
        blackboard->enemy_hero.x, blackboard->enemy_hero.y,
        blackboard->x, blackboard->y);
    
    // 关键修复：检查是否需要飞坡（针对所有目标点，不只是英雄）
    bool need_ramp = (blackboard->x < SentryConstants::HALF_MAP_X && 
                     attack_point.x >= SentryConstants::HALF_MAP_X);
    
    if (need_ramp && blackboard->isRampLockInactive()) {
        std::cout << "[HERO_ATTACK] 需要飞坡，保存原始目标点: (" 
                  << attack_point.x << ", " << attack_point.y << ")" << std::endl;
        
        // 保存原始目标点
        blackboard->original_target_before_ramp = attack_point;
        
        // 激活飞坡锁（PENDING状态）
        blackboard->activateRampLock();
        
        // 标记需要飞坡流程
        blackboard->ramp_in_process = true;
        
        // 启动飞坡流程（RAMP_PROCESS类型）
        blackboard->startBehavior(BehaviorType::RAMP_PROCESS, 
                                 SentryConstants::RED_RAMP_POINT, 0.0);
        
        // 设置前往飞坡点的控制消息
        updateControlMsg(blackboard, 
                       SentryConstants::GIMBAL_IDLE, 
                       SentryConstants::SPIN_OFF, 
                       SentryConstants::POSTURE_MOVE,  // 移动姿态
                       SentryConstants::RAMP_OFF);  // 到达飞坡点后才开启飞坡模式
        
        return BTStatus::RUNNING;
    } else {
        // 不需要飞坡，直接攻击
        
        // 关键修复2：每次都要启动行为，即使已经是攻击英雄类型
        // 这样确保目标点会更新并重新发布
        if (blackboard->current_behavior.type != BehaviorType::ATTACK_HERO ||
            blackboard->current_behavior.state == BehaviorState::COMPLETED) {
            
            std::cout << "[HERO_ATTACK] 开始/继续攻击英雄，英雄位置: (" 
                      << blackboard->enemy_hero.x << ", " << blackboard->enemy_hero.y << ")"
                      << "，目标点: (" << attack_point.x << ", " << attack_point.y << ")" 
                      << "，区域: " << region_manager->getRegionName(attack_point.x, attack_point.y) << std::endl;
            
            blackboard->startBehavior(BehaviorType::ATTACK_HERO, attack_point, 8.0);
        } else {
            // 如果已经在攻击英雄，但目标点可能已经改变，更新目标点
            double dx = blackboard->current_behavior.target.x - attack_point.x;
            double dy = blackboard->current_behavior.target.y - attack_point.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance > 50.0) {  // 如果目标点移动超过50cm，更新目标
                std::cout << "[HERO_ATTACK] 英雄移动，更新攻击目标点" << std::endl;
                blackboard->startBehavior(BehaviorType::ATTACK_HERO, attack_point, 8.0);
            }
        }
        
        // 检查是否在英雄部署区
        bool in_deploy_zone = region_manager->isInHeroDeployZone(blackboard->enemy_hero.x, blackboard->enemy_hero.y);
        
        if (in_deploy_zone) {
            std::cout << "[HERO_ATTACK] 英雄在部署区，采用打部署区姿态" << std::endl;
            updateControlMsg(blackboard, 
                           SentryConstants::GIMBAL_ENEMY,  // 打人模式
                           SentryConstants::SPIN_VARIABLE,  // 变速转
                           SentryConstants::POSTURE_ATTACK,  // 进攻姿态
                           SentryConstants::RAMP_OFF);
        } else {
            updateControlMsg(blackboard, 
                           SentryConstants::GIMBAL_ENEMY,  // 打人模式
                           SentryConstants::SPIN_VARIABLE,  // 变速转
                           SentryConstants::POSTURE_MOVE,  // 移动姿态前往目标
                           SentryConstants::RAMP_OFF);
        }
    }
    
    return BTStatus::RUNNING;
}

void SetHeroAttackTarget::reset() {
    // 重置攻击状态 - 空实现
}

// SetRobotAttackTarget 实现 - 保持不变
BTStatus SetRobotAttackTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                      std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard || !region_manager) return BTStatus::FAILURE;
    
    EnemyInfo* enemy = blackboard->getEnemyById(target_id_);
    if (!enemy || !enemy->visible || enemy->hp <= 0) {
        return BTStatus::FAILURE;
    }
    
    // 防止重复启动相同行为
    if (blackboard->current_behavior.type != BehaviorType::ATTACK_HERO) {
        // 计算攻击点
        geometry_msgs::msg::Point attack_point = region_manager->findSameRegionHexPoint(
            enemy->x, enemy->y,
            blackboard->x, blackboard->y);
        
        // 使用ATTACK_HERO类型，因为行为逻辑类似
        blackboard->startBehavior(BehaviorType::ATTACK_HERO, attack_point, 8.0);
    }
    
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    
    return BTStatus::RUNNING;
}

// SetRampPointTarget 实现 - 关键修改
BTStatus SetRampPointTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                    std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    // 防止重复启动相同行为
    if (blackboard->current_behavior.type != BehaviorType::MOVE_TO_FORTRESS) {
        blackboard->startBehavior(BehaviorType::MOVE_TO_FORTRESS, RED_RAMP_POINT, 1.0);
    }
    
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    
    return BTStatus::RUNNING;
}

// SetGainPointTarget 实现 - 保持不变
BTStatus SetGainPointTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                    std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    // 防止重复启动相同行为
    if (blackboard->current_behavior.type != BehaviorType::DEFEND_GAIN_POINT) {
        blackboard->startBehavior(BehaviorType::DEFEND_GAIN_POINT, position_, 10.0);
    }
    
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    
    return BTStatus::RUNNING;
}

// SetFortressOccupyTarget 实现 - 保持不变
BTStatus SetFortressOccupyTarget::execute(std::shared_ptr<Blackboard> blackboard,
                                         std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    // 防止重复启动相同行为
    if (blackboard->current_behavior.type != BehaviorType::OCCUPY_FORTRESS) {
        blackboard->startBehavior(BehaviorType::OCCUPY_FORTRESS, ENEMY_FORTRESS_OCCUPY, 10.0);
    }
    
    updateControlMsg(blackboard, GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
    
    return BTStatus::RUNNING;
}
