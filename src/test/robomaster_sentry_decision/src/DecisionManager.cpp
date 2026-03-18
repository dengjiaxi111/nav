#include "sentry_decision/DecisionManager.hpp"
#include "sentry_decision/Constants.hpp"
#include "sentry_decision/Models.hpp"
#include <iostream>

using namespace SentryConstants;

std::string DecisionManager::getBehaviorStateName(BehaviorState state) const {
    switch (state) {
        case BehaviorState::IDLE: return "IDLE";
        case BehaviorState::MOVING: return "MOVING";
        case BehaviorState::EXECUTING: return "EXECUTING";
        case BehaviorState::COMPLETED: return "COMPLETED";
        default: return "UNKNOWN";
    }
}

std::string DecisionManager::getBehaviorTypeName(BehaviorType type) const {
    switch (type) {
        case BehaviorType::NONE: return "NONE";
        case BehaviorType::MOVE_TO_ENERGY: return "MOVE_TO_ENERGY";
        case BehaviorType::ACTIVATE_ENERGY: return "ACTIVATE_ENERGY";
        case BehaviorType::MOVE_TO_OUTPOST: return "MOVE_TO_OUTPOST";
        case BehaviorType::ATTACK_OUTPOST: return "ATTACK_OUTPOST";
        case BehaviorType::MOVE_TO_SUPPLY: return "MOVE_TO_SUPPLY";
        case BehaviorType::SUPPLY: return "SUPPLY";
        case BehaviorType::MOVE_TO_GAIN_POINT: return "MOVE_TO_GAIN_POINT";
        case BehaviorType::DEFEND_GAIN_POINT: return "DEFEND_GAIN_POINT";
        case BehaviorType::MOVE_TO_HERO: return "MOVE_TO_HERO";
        case BehaviorType::ATTACK_HERO: return "ATTACK_HERO";
        case BehaviorType::MOVE_TO_BASE: return "MOVE_TO_BASE";
        case BehaviorType::DEFEND_BASE: return "DEFEND_BASE";
        case BehaviorType::MOVE_TO_FORTRESS: return "MOVE_TO_FORTRESS";
        case BehaviorType::OCCUPY_FORTRESS: return "OCCUPY_FORTRESS";
        case BehaviorType::RESURRECTION: return "RESURRECTION";
        case BehaviorType::RAMP_PROCESS: return "RAMP_PROCESS";
        default: return "UNKNOWN";
    }
}

GameStateProcessor::CriticalEvent GameStateProcessor::checkCriticalEvents() const {
    if (!blackboard_) return NONE;
    if (blackboard_->our_base_hp < 1500) return OUR_BASE_CRITICAL;
    if (blackboard_->our_outpost_hp == 0) return OUTPOST_DESTROYED;
    return NONE;
}

void GameStateProcessor::processGameStage() {
    if (!blackboard_) return;
    switch (blackboard_->stage) {
        case 0: case 1: case 2: case 3:
            blackboard_->initialization_complete = false;
            blackboard_->energy_activated = false;
            blackboard_->outpost_destroyed_init = false;
            init_state_ = INIT_WAIT_FOR_START;
            break;
        case 4: case 5: break;
    }
}

DecisionManager::DecisionManager()
    : blackboard_(std::make_shared<Blackboard>()),
      region_manager_(std::make_shared<RegionManager>()),
      game_state_processor_(std::make_shared<GameStateProcessor>(blackboard_)),
      last_decision_time_(0.0),
      init_energy_started_(false),
    init_outpost_started_(false),
    post_central_outpost_started_(false) {
    behavior_tree_ = TreeBuilder::buildMainDecisionTree();
    initialization_tree_ = TreeBuilder::buildInitializationTree();
}

void DecisionManager::updateOurState(const OurRobotState::SharedPtr msg) {
    if (blackboard_) blackboard_->updateOurState(msg);
}

void DecisionManager::updateEnemyState(const EnemyRobotState::SharedPtr msg) {
    if (blackboard_) blackboard_->updateEnemyState(msg);
}

void DecisionManager::updateGameState(const GameState::SharedPtr msg) {
    if (blackboard_) {
        blackboard_->updateGameState(msg);
        if (game_state_processor_) game_state_processor_->processGameStage();
    }
    static uint8_t last_stage = 0;
    if (msg->stage == STAGE_BATTLE && last_stage != STAGE_BATTLE) {
        init_energy_started_ = false;
        init_outpost_started_ = false;
        post_central_outpost_started_ = false;
    }
    last_stage = msg->stage;
}

bool DecisionManager::executeInitialization() {
    if (!blackboard_ || !initialization_tree_) return false;
    if (!blackboard_->initialization_complete) {
        BTStatus status = initialization_tree_->execute(blackboard_, region_manager_);
        if (status == BTStatus::RUNNING) {
            last_decision_reason_ = "INITIALIZATION_IN_PROGRESS";
            return true;
        } else if (status == BTStatus::SUCCESS) {
            last_decision_reason_ = "INITIALIZATION_COMPLETE";
            return false;
        }
    }
    return false;
}

// ===== 修改点：补给和复活行为仅根据时间完成，忽略血量/弹量 =====
bool DecisionManager::checkBehaviorCompletion() {
    if (!blackboard_->isBehaviorInProgress()) return false;
    
    BehaviorInfo& behavior = blackboard_->current_behavior;
    
    switch (behavior.type) {
        case BehaviorType::ACTIVATE_ENERGY:
            if (behavior.state == BehaviorState::EXECUTING) {
                double elapsed_time = blackboard_->getExecutionElapsedTime();
                if (elapsed_time >= behavior.execution_duration) {
                    std::cout << "[DECISION] 行为完成: " << getBehaviorTypeName(behavior.type)
                              << ", 持续时间: " << elapsed_time << "秒" << std::endl;
                    blackboard_->energy_activated = true;
                    std::cout << "[INIT] 打符完成，设置energy_activated为true" << std::endl;
                    blackboard_->completeCurrentBehavior();
                    init_energy_started_ = false;
                    return true;
                }
            }
            break;
            
        case BehaviorType::ATTACK_OUTPOST:
            if (behavior.state == BehaviorState::EXECUTING) {
                double elapsed_time = blackboard_->getExecutionElapsedTime();
                if (elapsed_time >= behavior.execution_duration) {
                    std::cout << "[DECISION] 行为完成: " << getBehaviorTypeName(behavior.type)
                              << ", 持续时间: " << elapsed_time << "秒" << std::endl;
                    blackboard_->initialization_complete = true;
                    blackboard_->outpost_destroyed_init = true;
                    std::cout << "[INIT] 初始化流程完成，进入主决策循环" << std::endl;
                    blackboard_->completeCurrentBehavior();
                    init_outpost_started_ = false;
                    return true;
                }
            }
            break;
            
        case BehaviorType::DEFEND_BASE:
        case BehaviorType::OCCUPY_FORTRESS:
        case BehaviorType::DEFEND_GAIN_POINT:
            if (behavior.state == BehaviorState::EXECUTING) {
                double elapsed_time = blackboard_->getExecutionElapsedTime();
                if (elapsed_time >= behavior.execution_duration) {
                    std::cout << "[DECISION] 行为完成: " << getBehaviorTypeName(behavior.type)
                              << ", 持续时间: " << elapsed_time << "秒" << std::endl;
                    blackboard_->completeCurrentBehavior();
                    return true;
                }
            }
            break;
            
        case BehaviorType::ATTACK_HERO:
            if (behavior.state == BehaviorState::EXECUTING) {
                double elapsed_time = blackboard_->getExecutionElapsedTime();
                if (!blackboard_->enemy_hero.visible || blackboard_->enemy_hero.hp <= 0) {
                    std::cout << "[DECISION] 英雄不可见或死亡，结束攻击" << std::endl;
                    blackboard_->completeCurrentBehavior();
                    return true;
                }
                if (elapsed_time >= behavior.execution_duration) {
                    std::cout << "[DECISION] 行为完成: " << getBehaviorTypeName(behavior.type)
                              << ", 持续时间: " << elapsed_time << "秒" << std::endl;
                    blackboard_->completeCurrentBehavior();
                    return true;
                }
            }
            break;
            
        // ===== 复活行为：仅时间到即完成 =====
        case BehaviorType::RESURRECTION:
            if (behavior.state == BehaviorState::EXECUTING) {
                double elapsed_time = blackboard_->getExecutionElapsedTime();
                if (elapsed_time >= behavior.execution_duration) {
                    std::cout << "[DECISION] 复活时间到，完成" << std::endl;
                    blackboard_->completeCurrentBehavior();
                    return true;
                }
            }
            break;
            
        // ===== 补给行为：仅时间到即完成 =====
        case BehaviorType::SUPPLY:
            if (behavior.state == BehaviorState::EXECUTING) {
                double elapsed_time = blackboard_->getExecutionElapsedTime();
                if (elapsed_time >= behavior.execution_duration) {
                    std::cout << "[DECISION] 补给时间到，完成" << std::endl;
                    blackboard_->completeCurrentBehavior();
                    return true;
                }
            }
            break;
            
        default:
            break;
    }
    return false;
}

bool DecisionManager::startNewBehaviorDecision() {
    if (!behavior_tree_) {
        std::cout << "[ERROR] behavior_tree_ is null!" << std::endl;
        return false;
    }
    if (blackboard_->isRampLockActive() || blackboard_->isRampLockPending()) {
        std::cout << "[DECISION] 飞坡锁激活，跳过主决策树" << std::endl;
        return false;
    }
    if (blackboard_->ramp_in_process) {
        std::cout << "[DECISION] 飞坡流程中，跳过主决策树" << std::endl;
        return false;
    }
    
    BTStatus status = behavior_tree_->execute(blackboard_, region_manager_);
    std::cout << "[DECISION] 行为树执行状态: " << static_cast<int>(status) << std::endl;
    
    bool has_behavior_now = blackboard_->isBehaviorInProgress();
    
    if (status == BTStatus::RUNNING || status == BTStatus::SUCCESS) {
        if (has_behavior_now) {
            std::cout << "[DECISION] 行为树启动了新行为: " 
                      << getBehaviorTypeName(blackboard_->current_behavior.type) << std::endl;
            return true;
        } else if (status == BTStatus::RUNNING) {
            std::cout << "[DECISION] 行为树在执行中" << std::endl;
            return true;
        }
    }
    std::cout << "[DECISION] 行为树没有启动新行为" << std::endl;
    return false;
}

DecisionOutput DecisionManager::executeDecision() {
    DecisionOutput output;
    output.target_needs_publishing = false;
    output.control_needs_publishing = false;
    
    if (blackboard_->stage != STAGE_BATTLE) {
        blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
        output.control_msg = *blackboard_->getControlMsg();
        output.decision_reason = "WAITING_FOR_START";
        output.control_needs_publishing = true;
        return output;
    }
    
    // 1. 检查行为是否完成
    if (blackboard_->isBehaviorInProgress()) {
        checkBehaviorCompletion();
    }
    
    // 2. 飞坡流程（最高优先级）
    if (blackboard_->ramp_in_process) {
        if (!blackboard_->isBehaviorInProgress() || blackboard_->current_behavior.type != BehaviorType::RAMP_PROCESS) {
            blackboard_->startBehavior(BehaviorType::RAMP_PROCESS, RED_RAMP_POINT, 0.0);
        }
        
        BehaviorInfo& behavior = blackboard_->current_behavior;
        switch (behavior.state) {
            case BehaviorState::MOVING: {
                bool at_ramp_point = blackboard_->isAtTarget(behavior.target, 50.0);
                if (at_ramp_point && !blackboard_->at_current_target) {
                    std::cout << "[RAMP_PROCESS] 到达飞坡点，开始飞坡" << std::endl;
                    blackboard_->setRampLockActive();
                    blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
                    blackboard_->setTargetReached(true);
                    blackboard_->startExecutionTime();
                    blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_ON);
                    blackboard_->ramp_mode_active = true;
                    if (blackboard_->original_target_before_ramp.x != 0 || 
                        blackboard_->original_target_before_ramp.y != 0) {
                        output.target_position = blackboard_->original_target_before_ramp;
                        output.target_needs_publishing = true;
                        blackboard_->setTargetPublished(true);
                        std::cout << "[RAMP_PROCESS] 发布原始目标点: (" 
                                  << output.target_position.x/100.0 << ", " 
                                  << output.target_position.y/100.0 << ")" << std::endl;
                    }
                    output.decision_reason = "RAMP_PROCESS_START_FLYING";
                    output.control_needs_publishing = true;
                } else if (!at_ramp_point && !blackboard_->isTargetPublished()) {
                    output.target_position = behavior.target;
                    output.decision_reason = "RAMP_PROCESS_MOVING_TO_RAMP_POINT";
                    output.target_needs_publishing = true;
                    blackboard_->setTargetPublished(true);
                    blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
                    output.control_needs_publishing = true;
                    std::cout << "[RAMP_PROCESS] 发布飞坡点: (" 
                              << output.target_position.x/100.0 << ", " 
                              << output.target_position.y/100.0 << ")" << std::endl;
                } else if (!at_ramp_point) {
                    output.decision_reason = "RAMP_PROCESS_MOVING_IN_PROGRESS";
                }
                break;
            }
            case BehaviorState::EXECUTING: {
                bool at_original_target = false;
                if (blackboard_->original_target_before_ramp.x != 0 || 
                    blackboard_->original_target_before_ramp.y != 0) {
                    at_original_target = blackboard_->isAtTarget(blackboard_->original_target_before_ramp, 50.0);
                }
                if (at_original_target) {
                    std::cout << "[RAMP_PROCESS] 到达原始目标点，飞坡流程完成" << std::endl;
                    blackboard_->completeCurrentBehavior();
                    blackboard_->ramp_in_process = false;
                    blackboard_->ramp_mode_active = false;
                    blackboard_->original_target_before_ramp.x = 0;
                    blackboard_->original_target_before_ramp.y = 0;
                    output.decision_reason = "RAMP_PROCESS_COMPLETE";
                } else {
                    output.decision_reason = "RAMP_PROCESS_FLYING";
                    if (!blackboard_->isControlPublished()) {
                        blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_ON);
                        output.control_needs_publishing = true;
                    }
                }
                break;
            }
            default:
                output.decision_reason = "RAMP_PROCESS_UNKNOWN_STATE";
                break;
        }
        
        output.control_msg = *blackboard_->getControlMsg();
        if (blackboard_->isControlUpdated()) {
            output.control_needs_publishing = true;
            blackboard_->setControlUpdated(false);
        }
        return output;
    }
    
    // 强制中断逻辑（仅对可中断的行为）
    bool force_redecide = false;
    if (blackboard_->isBehaviorInProgress()) {
        auto type = blackboard_->current_behavior.type;
        bool is_interruptible = (type == BehaviorType::DEFEND_BASE ||
                     type == BehaviorType::ATTACK_HERO ||
                                 type == BehaviorType::DEFEND_GAIN_POINT ||
                                 type == BehaviorType::OCCUPY_FORTRESS);
        if (is_interruptible) {
            double hp_ratio = blackboard_->current_hp / 400.0;
            double ammo_ratio = blackboard_->allowance_17mm / 300.0;
            if (hp_ratio < 0.2 || ammo_ratio < 0.15 || blackboard_->resurrection_flag) {
                std::cout << "[DECISION] 状态不佳，中断当前行为前往补给/复活" << std::endl;
                blackboard_->resetCurrentBehavior();
                force_redecide = true;
            }
        }
    }
    
    // 3. 如果有行为在进行且未被中断，继续处理
    if (blackboard_->isBehaviorInProgress() && !force_redecide) {
        BehaviorInfo& behavior = blackboard_->current_behavior;
        
        if (behavior.type == BehaviorType::RAMP_PROCESS) {
            output.decision_reason = "RAMP_PROCESS_SKIPPED";
            output.control_msg = *blackboard_->getControlMsg();
            return output;
        }
        
        switch (behavior.state) {
            case BehaviorState::MOVING: {
                bool at_target = blackboard_->isAtTarget(behavior.target, 50.0);
                if (at_target && !blackboard_->at_current_target) {
                    if (behavior.type == BehaviorType::MOVE_TO_SUPPLY ||
                        behavior.type == BehaviorType::RESURRECTION) {
                        behavior.type = BehaviorType::SUPPLY;
                        behavior.execution_start_time = blackboard_->current_time;
                    }
                    
                    std::cout << "[DECISION] 到达目标点，切换姿态" << std::endl;
                    blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
                    blackboard_->setTargetReached(true);
                    blackboard_->startExecutionTime();
                    
                    switch (behavior.type) {
                        case BehaviorType::ACTIVATE_ENERGY:
                            blackboard_->updateControlMsg(GIMBAL_ENERGY, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
                            output.decision_reason = "ARRIVED_AT_ENERGY_ACTIVATION";
                            break;
                        case BehaviorType::ATTACK_OUTPOST:
                            blackboard_->updateControlMsg(GIMBAL_OUTPOST, SPIN_VARIABLE, POSTURE_ATTACK, RAMP_OFF);
                            output.decision_reason = "ARRIVED_AT_OUTPOST_ATTACK";
                            break;
                        case BehaviorType::DEFEND_BASE:
                            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
                            output.decision_reason = "ARRIVED_AT_BASE_DEFENSE";
                            break;
                        case BehaviorType::SUPPLY:
                            blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
                            output.decision_reason = "ARRIVED_AT_SUPPLY";
                            break;
                        case BehaviorType::ATTACK_HERO:
                            if (region_manager_->isInHeroDeployZone(behavior.target.x, behavior.target.y)) {
                                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_VARIABLE, POSTURE_ATTACK, RAMP_OFF);
                                std::cout << "[DECISION] 英雄在部署区，使用进攻姿态" << std::endl;
                            } else {
                                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_VARIABLE, POSTURE_ATTACK, RAMP_OFF);
                            }
                            output.decision_reason = "ARRIVED_AT_HERO_ATTACK";
                            break;
                        case BehaviorType::OCCUPY_FORTRESS:
                        case BehaviorType::DEFEND_GAIN_POINT:
                            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
                            output.decision_reason = (behavior.type == BehaviorType::OCCUPY_FORTRESS) ?
                                                     "ARRIVED_AT_FORTRESS_OCCUPY" : "ARRIVED_AT_GAIN_POINT";
                            break;
                        default:
                            blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_DEFENSE, RAMP_OFF);
                            output.decision_reason = "ARRIVED_AT_TARGET";
                    }
                    output.control_needs_publishing = true;
                    
                } else if (!at_target && !blackboard_->isTargetPublished()) {
                    output.target_position = behavior.target;
                    output.decision_reason = "MOVING_TO_TARGET";
                    output.target_needs_publishing = true;
                    blackboard_->setTargetPublished(true);
                    if (!blackboard_->ramp_mode_active) {
                        blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
                        output.control_needs_publishing = true;
                    }
                } else if (!at_target) {
                    output.decision_reason = "MOVING_IN_PROGRESS";
                }
                break;
            }
            case BehaviorState::EXECUTING: {
                output.decision_reason = "EXECUTING_BEHAVIOR";
                if (!blackboard_->isControlPublished()) {
                    output.control_needs_publishing = true;
                }
                break;
            }
            case BehaviorState::COMPLETED: {
                output.decision_reason = "BEHAVIOR_COMPLETE";
                blackboard_->resetCurrentBehavior();
                blackboard_->at_ramp_point = false;
                blackboard_->ramp_mode_active = false;
                break;
            }
            default:
                output.decision_reason = "UNKNOWN_BEHAVIOR_STATE";
                break;
        }
        
        output.control_msg = *blackboard_->getControlMsg();
        if (blackboard_->isControlUpdated()) {
            output.control_needs_publishing = true;
            blackboard_->setControlUpdated(false);
        }
        return output;
    }
    
    // 4. 没有行为在执行，开始新的决策
    if (blackboard_->resurrection_flag) {
        // 设置复活持续时间为10秒
        blackboard_->startBehavior(BehaviorType::RESURRECTION, RED_SUPPLY, 10.0);
        output.target_position = RED_SUPPLY;
        output.decision_reason = "START_RESURRECTION";
        output.target_needs_publishing = true;
        output.control_msg = *blackboard_->getControlMsg();
        return output;
    }

    // 资源不足时，强制优先补给（高于关键事件/主行为树）
    bool low_hp_or_ammo = (blackboard_->current_hp < 100.0 || blackboard_->allowance_17mm < 50.0);
    if (!blackboard_->isBehaviorInProgress() && low_hp_or_ammo) {
        std::cout << "[MAIN] 血量/弹量不足，强制优先前往补给区" << std::endl;
        blackboard_->startBehavior(BehaviorType::MOVE_TO_SUPPLY, RED_SUPPLY, 10.0);
        blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);

        output.target_position = RED_SUPPLY;
        output.decision_reason = "FORCE_SUPPLY_BY_LOW_HP_OR_AMMO";
        output.target_needs_publishing = true;
        output.control_needs_publishing = true;
        output.control_msg = *blackboard_->getControlMsg();
        return output;
    }
    
    auto critical_event = game_state_processor_->checkCriticalEvents();
    if (critical_event != GameStateProcessor::NONE) {
        handleCriticalEvent(critical_event);
        if (blackboard_->isBehaviorInProgress()) {
            output.target_position = blackboard_->current_behavior.target;
            output.decision_reason = "CRITICAL_EVENT_RESPONSE";
            output.target_needs_publishing = !blackboard_->isTargetPublished();
            output.control_msg = *blackboard_->getControlMsg();
            return output;
        }
    }
    
    if (blackboard_->initialization_complete) {
        // 新增：占领中央高地后，优先转前哨站攻击点（单次触发）
        if (!blackboard_->isBehaviorInProgress() &&
            blackboard_->central_highland_occupied_by_us &&
            !blackboard_->outpost_gain_point_occupied_by_us &&
            !post_central_outpost_started_) {
            std::cout << "[MAIN] 已占领中央高地，转前哨站攻击点" << std::endl;
            blackboard_->startBehavior(BehaviorType::ATTACK_OUTPOST, ENEMY_OUTPOST_ATTACK, 10.0);
            blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
            post_central_outpost_started_ = true;

            output.target_position = ENEMY_OUTPOST_ATTACK;
            output.decision_reason = "START_OUTPOST_AFTER_CENTRAL_HIGHLAND";
            output.target_needs_publishing = true;
            output.control_needs_publishing = true;
            output.control_msg = *blackboard_->getControlMsg();
            return output;
        }

        std::cout << "[MAIN] 进入主决策循环" << std::endl;
        if (startNewBehaviorDecision()) {
            std::cout << "[MAIN] 行为树启动新行为: " 
                      << getBehaviorTypeName(blackboard_->current_behavior.type) << std::endl;
            BehaviorInfo& behavior = blackboard_->current_behavior;
            output.decision_reason = "START_NEW_BEHAVIOR_FROM_MAIN_TREE";
            if (behavior.state == BehaviorState::MOVING && !blackboard_->isTargetPublished()) {
                output.target_position = behavior.target;
                output.target_needs_publishing = true;
                blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
                output.control_needs_publishing = true;
            }
            output.control_msg = *blackboard_->getControlMsg();
            return output;
        }

        if (!blackboard_->isBehaviorInProgress()) {
            std::cout << "[MAIN] 无其他行为，开始默认基地防御" << std::endl;
            blackboard_->startBehavior(BehaviorType::DEFEND_BASE, RED_BASE_GAIN, 10.0);
            output.target_position = RED_BASE_GAIN;
            output.decision_reason = "START_DEFAULT_BEHAVIOR";
            output.target_needs_publishing = true;
            output.control_msg = *blackboard_->getControlMsg();
            return output;
        }
        output.control_msg = *blackboard_->getControlMsg();
        return output;
    }
    
    // 初始化阶段
    if (!blackboard_->energy_activated && !init_energy_started_ && !blackboard_->isBehaviorInProgress()) {
        std::cout << "[INIT] 开始打符" << std::endl;
        blackboard_->startBehavior(BehaviorType::ACTIVATE_ENERGY, RED_ENERGY_POINT, 10.0);
        init_energy_started_ = true;
    }
    else if (blackboard_->energy_activated && !blackboard_->initialization_complete && 
             !init_outpost_started_ && !blackboard_->isBehaviorInProgress()) {
        std::cout << "[INIT] 打符完成，前往前哨站攻击点" << std::endl;
        blackboard_->startBehavior(BehaviorType::ATTACK_OUTPOST, ENEMY_OUTPOST_ATTACK, 10.0);
        init_outpost_started_ = true;
        blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
        output.control_needs_publishing = true;
    }
    
    if (blackboard_->isBehaviorInProgress()) {
        BehaviorInfo& behavior = blackboard_->current_behavior;
        if (behavior.state == BehaviorState::MOVING && !blackboard_->isTargetPublished()) {
            output.target_position = behavior.target;
            output.target_needs_publishing = true;
            if (behavior.type == BehaviorType::ATTACK_OUTPOST || 
                behavior.type == BehaviorType::ACTIVATE_ENERGY) {
                blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
                output.control_needs_publishing = true;
            }
            output.decision_reason = "MOVING_TO_TARGET_IN_INIT";
        }
        output.control_msg = *blackboard_->getControlMsg();
        return output;
    }
    
    output.control_msg = *blackboard_->getControlMsg();
    return output;
}

void DecisionManager::handleCriticalEvent(GameStateProcessor::CriticalEvent event) {
    // 资源不足时不处理关键事件抢占，避免与补给决策打架
    if (blackboard_->current_hp < 100.0 ||
        blackboard_->allowance_17mm < 50.0 ||
        blackboard_->resurrection_flag) {
        return;
    }

    switch (event) {
        case GameStateProcessor::OUR_BASE_CRITICAL:
            if (!blackboard_->isBehaviorInProgress()) {
                blackboard_->startBehavior(BehaviorType::DEFEND_BASE, RED_BASE_GAIN, 10.0);
                blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
            }
            break;
        case GameStateProcessor::OUTPOST_DESTROYED:
            {
                double Z = Models::calculateSituationZ(*blackboard_);
                double distance_to_fortress = Models::calculateDistance(
                    blackboard_->x, blackboard_->y,
                    ENEMY_FORTRESS_OCCUPY.x,
                    ENEMY_FORTRESS_OCCUPY.y);
                double F = Models::calculateFortressValue(*blackboard_, distance_to_fortress);
                double hp_ratio = blackboard_->current_hp / 400.0;
                if (Z > 0.6 && F > 0.7 && hp_ratio > 0.7) {
                    if (!blackboard_->isBehaviorInProgress()) {
                        blackboard_->startBehavior(BehaviorType::OCCUPY_FORTRESS, ENEMY_FORTRESS_OCCUPY, 10.0);
                        blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
                    }
                } else {
                    if (!blackboard_->isBehaviorInProgress()) {
                        blackboard_->startBehavior(BehaviorType::DEFEND_BASE, RED_BASE_GAIN, 10.0);
                        blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
                    }
                }
            }
            break;
        default:
            break;
    }
}
