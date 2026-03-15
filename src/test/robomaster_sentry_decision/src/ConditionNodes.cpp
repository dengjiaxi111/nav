#include "sentry_decision/bt/BTCondition.hpp"
#include "sentry_decision/Models.hpp"
#include <iostream>

// CheckGameStarted 实现 - 保持不变
BTStatus CheckGameStarted::execute(std::shared_ptr<Blackboard> blackboard,
                                  std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    if (blackboard->stage == 4) {  // 比赛进行中
        return BTStatus::SUCCESS;
    }
    return BTStatus::FAILURE;
}

// CheckResurrection 实现 - 保持不变
BTStatus CheckResurrection::execute(std::shared_ptr<Blackboard> blackboard,
                                   std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    if (blackboard->resurrection_flag) {
        return BTStatus::SUCCESS;
    }
    return BTStatus::FAILURE;
}

// CheckBaseCritical 实现 - 保持不变
BTStatus CheckBaseCritical::execute(std::shared_ptr<Blackboard> blackboard,
                                   std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    if (blackboard->our_base_hp < 1500) {
        return BTStatus::SUCCESS;
    }
    return BTStatus::FAILURE;
}

// CheckOutpostDestroyed 实现 - 保持不变
BTStatus CheckOutpostDestroyed::execute(std::shared_ptr<Blackboard> blackboard,
                                       std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    if (blackboard->our_outpost_hp == 0) {
        return BTStatus::SUCCESS;
    }
    return BTStatus::FAILURE;
}

// CheckSupplyRFID 实现 - 保持不变
BTStatus CheckSupplyRFID::execute(std::shared_ptr<Blackboard> blackboard,
                                 std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    bool rfid_bit19 = (blackboard->rfid_status >> 19) & 1;
    bool rfid_bit20 = (blackboard->rfid_status >> 20) & 1;
    
    if ((rfid_bit19 || rfid_bit20) && blackboard->current_hp >= 400.0) {
        return BTStatus::SUCCESS;
    }
    return BTStatus::FAILURE;
}

// CheckAtRampPoint 实现 - 关键修复版本
BTStatus CheckAtRampPoint::execute(std::shared_ptr<Blackboard> blackboard,
                                  std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    double dx = blackboard->x - SentryConstants::RED_RAMP_POINT.x;
    double dy = blackboard->y - SentryConstants::RED_RAMP_POINT.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    if (distance <= 20.0) {  // 到达飞坡点
        std::cout << "[RAMP] 到达飞坡点，准备飞坡" << std::endl;
        
        // 标记到达飞坡点
        blackboard->at_ramp_point = true;
        
        return BTStatus::SUCCESS;
    }
    
    // 如果不在飞坡点，重置飞坡标志
    if (blackboard->at_ramp_point) {
        blackboard->at_ramp_point = false;
    }
    
    return BTStatus::FAILURE;
}

// CheckAttackDuration 实现 - 保持不变
BTStatus CheckAttackDuration::execute(std::shared_ptr<Blackboard> blackboard,
                                     std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    if (!blackboard->current_target_id.empty()) {
        double attack_duration = blackboard->current_time - blackboard->attack_start_time;
        if (attack_duration >= 10.0) {
            // 攻击持续时间超过10秒，结束攻击
            blackboard->current_target_id = "";
            blackboard->setTargetReached(false);
            return BTStatus::SUCCESS;
        }
        
        // 检查目标是否死亡
        EnemyInfo* target = blackboard->getEnemyById(blackboard->current_target_id);
        
        if (target && target->hp <= 0) {
            // 目标死亡，结束攻击
            blackboard->current_target_id = "";
            blackboard->setTargetReached(false);
            return BTStatus::SUCCESS;
        }
        
        return BTStatus::RUNNING;
    }
    return BTStatus::FAILURE;
}

// CheckSupplyComplete 实现 - 保持不变
BTStatus CheckSupplyComplete::execute(std::shared_ptr<Blackboard> blackboard,
                                     std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    if (!blackboard->at_supply_point) {
        return BTStatus::FAILURE;
    }
    
    bool rfid_bit19 = (blackboard->rfid_status >> 19) & 1;
    bool rfid_bit20 = (blackboard->rfid_status >> 20) & 1;
    if (!rfid_bit19 && !rfid_bit20) {
        return BTStatus::FAILURE;
    }
    
    if (blackboard->current_hp < 400.0) {
        return BTStatus::FAILURE;
    }
    
    return BTStatus::SUCCESS;
}

// EvaluateEnergyActivation 实现 - 保持不变
BTStatus EvaluateEnergyActivation::execute(std::shared_ptr<Blackboard> blackboard,
                                          std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    // 如果已经到达目标点且不应离开，则继续执行
    if (blackboard->at_current_target && !blackboard->shouldLeaveTarget()) {
        return BTStatus::SUCCESS;
    }
    
    double Z = Models::calculateSituationZ(*blackboard);
    double distance_to_energy = Models::calculateDistance(
        blackboard->x, blackboard->y,
        SentryConstants::RED_ENERGY_POINT.x,
        SentryConstants::RED_ENERGY_POINT.y);
    double E = Models::calculateEnergyValue(*blackboard, distance_to_energy);
    
    bool should_activate = false;
    if (Z > 0.6 && E > 0.7) {
        should_activate = true;
    } else if (Z >= 0.4 && Z <= 0.6 && E > 0.6) {
        should_activate = true;
    }
    
    if (should_activate) {
        return BTStatus::SUCCESS;
    }
    return BTStatus::FAILURE;
}

// EvaluateSupplyNeed 实现 - 保持不变
BTStatus EvaluateSupplyNeed::execute(std::shared_ptr<Blackboard> blackboard,
                                    std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    // 如果已经到达目标点且不应离开，则继续执行
    if (blackboard->at_current_target && !blackboard->shouldLeaveTarget()) {
        return BTStatus::SUCCESS;
    }
    
    double Z = Models::calculateSituationZ(*blackboard);
    double distance_to_supply = Models::calculateDistance(
        blackboard->x, blackboard->y,
        SentryConstants::RED_SUPPLY.x,
        SentryConstants::RED_SUPPLY.y);
    double S = Models::calculateSupplyScore(*blackboard, distance_to_supply);
    
    double hp_ratio = blackboard->current_hp / 400.0;
    double ammo_ratio = blackboard->allowance_17mm / 300.0;
    
    bool should_supply = false;
    
    if (hp_ratio < 0.2 || ammo_ratio < 0.15) {
        should_supply = true;
    } else if (Z > 0.6 && S > 0.6) {
        double nearest_enemy = Models::getNearestEnemyDistance(*blackboard);
        if (nearest_enemy > 500.0) {
            should_supply = true;
        }
    } else if (Z >= 0.4 && Z <= 0.6 && S > 0.5) {
        should_supply = true;
    } else if (Z < 0.4 && S > 0.4) {
        should_supply = true;
    }
    
    if (should_supply) {
        return BTStatus::SUCCESS;
    }
    return BTStatus::FAILURE;
}

// EvaluateHeroAttack 实现 - 修改：添加调试输出
BTStatus EvaluateHeroAttack::execute(std::shared_ptr<Blackboard> blackboard,
                                    std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    // 如果已经到达目标点且不应离开，则继续执行
    if (blackboard->at_current_target && !blackboard->shouldLeaveTarget()) {
        return BTStatus::SUCCESS;
    }
    
    if (!blackboard->enemy_hero.visible || blackboard->enemy_hero.hp <= 0) {
        std::cout << "[EVAL_HERO] 英雄不可见或已死亡" << std::endl;
        return BTStatus::FAILURE;
    }
    
    double Z = Models::calculateSituationZ(*blackboard);
    double distance_to_hero = Models::calculateDistance(
        blackboard->x, blackboard->y,
        blackboard->enemy_hero.x, blackboard->enemy_hero.y);
    double H = Models::calculateHeroAttackValue(*blackboard, distance_to_hero, 
                                               blackboard->hero_in_deploy_zone);
    
    // 打印评估值
    std::cout << "[EVAL_HERO] Z=" << Z << ", H=" << H 
              << ", 距离=" << distance_to_hero 
              << ", 部署区=" << (blackboard->hero_in_deploy_zone ? "是" : "否") << std::endl;
    
    // 降低英雄攻击阈值，提高优先级
    bool should_attack = false;
    if (Z > 0.4 && H > 0.5) {  // 降低阈值
        should_attack = true;
    } else if (Z >= 0.3 && Z <= 0.4 && H > 0.4) {
        should_attack = true;
    } else if (Z < 0.3 && H > 0.6) {
        should_attack = true;
    }
    
    if (should_attack) {
        std::cout << "[EVAL_HERO] 应该攻击英雄!" << std::endl;
        return BTStatus::SUCCESS;
    }
    
    std::cout << "[EVAL_HERO] 不满足攻击条件" << std::endl;
    return BTStatus::FAILURE;
}

// 修改 CheckHeroAttackCondition::execute 函数
BTStatus CheckHeroAttackCondition::execute(std::shared_ptr<Blackboard> blackboard,
                                         std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    // 关键修改：如果飞坡锁已激活或挂起，直接返回失败
    if (blackboard->isRampLockActive() || blackboard->isRampLockPending()) {
        std::cout << "[CHECK_HERO] 飞坡锁激活，跳过英雄攻击检查" << std::endl;
        return BTStatus::FAILURE;
    }
    
    // 检查英雄是否可见且存活
    if (!blackboard->enemy_hero.visible || blackboard->enemy_hero.hp <= 0) {
        std::cout << "[CHECK_HERO] 英雄不可见或已死亡" << std::endl;
        return BTStatus::FAILURE;
    }
    
    // 检查英雄是否在部署区超过1秒
    if (!blackboard->hero_in_deploy_zone) {
        std::cout << "[CHECK_HERO] 英雄不在部署区" << std::endl;
        return BTStatus::FAILURE;
    }
    
    // 计算自身状态评分（Z值）
    double Z = Models::calculateSituationZ(*blackboard);
    
    std::cout << "[CHECK_HERO] Z=" << Z << ", 英雄在部署区，触发高优先级攻击" << std::endl;
    
    // 高优先级条件：状态良好（Z>0.3）且英雄在部署区
    if (Z > 0.3) {
        return BTStatus::SUCCESS;
    }
    
    return BTStatus::FAILURE;
}

// 修改 CheckNeedRamp::execute 函数，添加更严格的检查
BTStatus CheckNeedRamp::execute(std::shared_ptr<Blackboard> blackboard,
                               std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;
    
    // 关键修改：如果飞坡锁已激活或挂起，直接返回失败
    if (blackboard->isRampLockActive() || blackboard->isRampLockPending()) {
        return BTStatus::FAILURE;
    }
    
    // 如果已经在飞坡流程中，不再检查飞坡需求
    if (blackboard->ramp_in_process) {
        return BTStatus::FAILURE;
    }
    
    // 检查是否在飞坡点
    double dx_to_ramp = blackboard->x - SentryConstants::RED_RAMP_POINT.x;
    double dy_to_ramp = blackboard->y - SentryConstants::RED_RAMP_POINT.y;
    double distance_to_ramp = std::sqrt(dx_to_ramp*dx_to_ramp + dy_to_ramp*dy_to_ramp);
    
    // 如果已经在飞坡点，不触发飞坡需求
    if (distance_to_ramp <= 20.0) {
        return BTStatus::FAILURE;
    }
    
    // 检查当前行为是否需要飞坡
    // 条件：目标点在敌方半区（x>=1400）且自身在己方半区（x<1400）
    // 并且当前没有其他高优先级行为在执行
    if (blackboard->current_behavior.target.x >= SentryConstants::HALF_MAP_X && 
        blackboard->x < SentryConstants::HALF_MAP_X &&
        blackboard->isRampLockInactive()) {  // 确保飞坡锁未激活
        std::cout << "[RAMP] 需要飞坡：目标在敌方半区，自身在己方半区，且不在飞坡点" << std::endl;
        return BTStatus::SUCCESS;
    }
    
    return BTStatus::FAILURE;
}


// EvaluateTargetSelection 实现 - 保持不变
BTStatus EvaluateTargetSelection::execute(std::shared_ptr<Blackboard> blackboard,
                                         std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    // 如果已经到达目标点且不应离开，则继续执行
    if (blackboard->at_current_target && !blackboard->shouldLeaveTarget()) {
        return BTStatus::SUCCESS;
    }
    
    double Z = Models::calculateSituationZ(*blackboard);
    auto target_scores = Models::calculateAllTargetScores(*blackboard);
    
    std::vector<Models::TargetScore> valid_targets;
    
    double threshold = 0.0;
    if (Z > 0.6) {
        threshold = 0.6;
    } else if (Z >= 0.4 && Z <= 0.6) {
        threshold = 0.5;
    } else {
        threshold = 0.7;
    }
    
    for (const auto& target : target_scores) {
        if (target.score > threshold) {
            valid_targets.push_back(target);
        }
    }
    
    if (!valid_targets.empty()) {
        selected_target_ = valid_targets[0].id;
        return BTStatus::SUCCESS;
    }
    
    return BTStatus::FAILURE;
}

// EvaluateRampNeed 实现 - 保持不变
BTStatus EvaluateRampNeed::execute(std::shared_ptr<Blackboard> blackboard,
                                  std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    if (blackboard->target_position.x >= SentryConstants::HALF_MAP_X && 
        blackboard->x < SentryConstants::HALF_MAP_X) {
        return BTStatus::SUCCESS;
    }
    return BTStatus::FAILURE;
}

// EvaluateGainPoint 实现 - 保持不变
BTStatus EvaluateGainPoint::execute(std::shared_ptr<Blackboard> blackboard,
                                   std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    // 如果已经到达目标点且不应离开，则继续执行
    if (blackboard->at_current_target && !blackboard->shouldLeaveTarget()) {
        return BTStatus::SUCCESS;
    }
    
    double Z = Models::calculateSituationZ(*blackboard);
    auto gain_point_scores = Models::calculateGainPointScores(*blackboard);
    
    std::vector<Models::GainPointScore> valid_gain_points;
    
    double threshold = 0.0;
    if (Z > 0.6) {
        threshold = 0.5;
    } else if (Z >= 0.4 && Z <= 0.6) {
        threshold = 0.4;
    } else {
        threshold = 0.6;
    }
    
    for (const auto& gain_point : gain_point_scores) {
        bool occupied = false;
        if (gain_point.name == "base_gain" && blackboard->base_gain_point_occupied) occupied = true;
        else if (gain_point.name == "trapezoid_highland_gain" && blackboard->trapezoid_highland_occupied) occupied = true;
        else if (gain_point.name == "fortress_gain" && 
                (blackboard->fortress_gain_point_occupied_by_us || blackboard->fortress_gain_point_occupied_by_enemy)) occupied = true;
        else if (gain_point.name == "central_highland_gain" && 
                (blackboard->central_highland_occupied_by_us || blackboard->central_highland_occupied_by_enemy)) occupied = true;
        else if (gain_point.name == "outpost_gain" && 
                (blackboard->outpost_gain_point_occupied_by_us || blackboard->outpost_gain_point_occupied_by_enemy)) occupied = true;
        
        if (!occupied && gain_point.score > threshold) {
            valid_gain_points.push_back(gain_point);
        }
    }
    
    if (!valid_gain_points.empty()) {
        selected_gain_point_ = valid_gain_points[0];
        return BTStatus::SUCCESS;
    }
    
    return BTStatus::FAILURE;
}

// CheckFortressOccupyCondition 实现 - 保持不变
BTStatus CheckFortressOccupyCondition::execute(std::shared_ptr<Blackboard> blackboard,
                                              std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    double Z = Models::calculateSituationZ(*blackboard);
    double distance_to_fortress = Models::calculateDistance(
        blackboard->x, blackboard->y,
        SentryConstants::ENEMY_FORTRESS_OCCUPY.x,
        SentryConstants::ENEMY_FORTRESS_OCCUPY.y);
    double F = Models::calculateFortressValue(*blackboard, distance_to_fortress);
    double hp_ratio = blackboard->current_hp / 400.0;
    
    if (Z > 0.6 && F > 0.7 && hp_ratio > 0.7) {
        return BTStatus::SUCCESS;
    }
    return BTStatus::FAILURE;
}

// CheckInitializationComplete 实现 - 保持不变
BTStatus CheckInitializationComplete::execute(std::shared_ptr<Blackboard> blackboard,
                                             std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    if (!blackboard->initialization_complete) {
        return BTStatus::SUCCESS;
    }
    return BTStatus::FAILURE;
}

// CheckAtEnergyPoint 实现 - 保持不变
BTStatus CheckAtEnergyPoint::execute(std::shared_ptr<Blackboard> blackboard,
                                    std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    double dx = blackboard->x - SentryConstants::RED_ENERGY_POINT.x;
    double dy = blackboard->y - SentryConstants::RED_ENERGY_POINT.y;
    if (std::sqrt(dx*dx + dy*dy) <= 50.0) {  // 改为50cm容差
        return BTStatus::SUCCESS;
    }
    return BTStatus::FAILURE;
}

// CheckEnergyActivated 实现 - 保持不变
BTStatus CheckEnergyActivated::execute(std::shared_ptr<Blackboard> blackboard,
                                      std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    // 检查能量机关是否已激活
    if (blackboard->energy_activated) {
        return BTStatus::SUCCESS;
    }
    return BTStatus::FAILURE;
}

// CheckOutpostStatus 实现 - 保持不变
BTStatus CheckOutpostStatus::execute(std::shared_ptr<Blackboard> blackboard,
                                    std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    (void)region_manager;

    // 这个节点不再使用，逻辑已集成到DecisionManager中
    return BTStatus::FAILURE;
}
