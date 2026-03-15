#include "sentry_decision/DecisionManager.hpp"
#include "sentry_decision/Constants.hpp"
#include "sentry_decision/Models.hpp"
#include <iostream>

using namespace SentryConstants;

// 实现 getBehaviorStateName
std::string DecisionManager::getBehaviorStateName(BehaviorState state) const {
    switch (state) {
        case BehaviorState::IDLE: return "IDLE";
        case BehaviorState::MOVING: return "MOVING";
        case BehaviorState::EXECUTING: return "EXECUTING";
        case BehaviorState::COMPLETED: return "COMPLETED";
        default: return "UNKNOWN";
    }
}

// 实现 getBehaviorTypeName
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
        case BehaviorType::RAMP_PROCESS: return "RAMP_PROCESS";  // 新增
        default: return "UNKNOWN";
    }
}

// GameStateProcessor 方法实现
GameStateProcessor::CriticalEvent GameStateProcessor::checkCriticalEvents() const {
    if (!blackboard_) return NONE;
    
    // 检查我方基地危急
    if (blackboard_->our_base_hp < 1500) {
        return OUR_BASE_CRITICAL;
    }
    
    // 检查前哨站被毁
    if (blackboard_->our_outpost_hp == 0) {
        return OUTPOST_DESTROYED;
    }
    
    return NONE;
}

void GameStateProcessor::processGameStage() {
    if (!blackboard_) return;
    
    // 处理比赛阶段
    switch (blackboard_->stage) {
        case 0: // 未开始比赛
        case 1: // 准备阶段
        case 2: // 十五秒裁判系统自检阶段
        case 3: // 五秒倒计时
            // 这些阶段都不能动
            blackboard_->initialization_complete = false;
            blackboard_->energy_activated = false;
            blackboard_->outpost_destroyed_init = false;
            init_state_ = INIT_WAIT_FOR_START;
            break;
            
        case 4: // 比赛中
            // 如果初始化未完成，继续初始化流程
            break;
            
        case 5: // 比赛结算中
            // 比赛结束，不要设置initialization_complete = true
            break;
    }
}

// DecisionManager 构造函数
DecisionManager::DecisionManager()
    : blackboard_(std::make_shared<Blackboard>()),
      region_manager_(std::make_shared<RegionManager>()),
      game_state_processor_(std::make_shared<GameStateProcessor>(blackboard_)),
      last_decision_time_(0.0),
      init_energy_started_(false),
      init_outpost_started_(false) {
    
    // 构建行为树
    behavior_tree_ = TreeBuilder::buildMainDecisionTree();
    initialization_tree_ = TreeBuilder::buildInitializationTree();
}

// 状态更新函数
void DecisionManager::updateOurState(const OurRobotState::SharedPtr msg) {
    if (blackboard_) {
        blackboard_->updateOurState(msg);
    }
}

void DecisionManager::updateEnemyState(const EnemyRobotState::SharedPtr msg) {
    if (blackboard_) {
        blackboard_->updateEnemyState(msg);
    }
}

void DecisionManager::updateGameState(const GameState::SharedPtr msg) {
    if (blackboard_) {
        blackboard_->updateGameState(msg);
        if (game_state_processor_) {
            game_state_processor_->processGameStage();
        }
    }
    
    // 比赛开始时重置初始化标志
    static uint8_t last_stage = 0;
    if (msg->stage == STAGE_BATTLE && last_stage != STAGE_BATTLE) {
        // 从非比赛阶段进入比赛阶段，重置初始化标志
        init_energy_started_ = false;
        init_outpost_started_ = false;
    }
    last_stage = msg->stage;
}

// 初始化流程
bool DecisionManager::executeInitialization() {
    if (!blackboard_ || !initialization_tree_) return false;
    
    // 只在初始化未完成时才执行初始化树
    if (!blackboard_->initialization_complete) {
        BTStatus status = initialization_tree_->execute(blackboard_, region_manager_);
        
        if (status == BTStatus::RUNNING) {
            last_decision_reason_ = "INITIALIZATION_IN_PROGRESS";
            return true;
        }
        else if (status == BTStatus::SUCCESS) {
            last_decision_reason_ = "INITIALIZATION_COMPLETE";
            return false; // 初始化完成，不再需要执行
        }
    }
    
    return false;
}

// DecisionManager.cpp中的checkBehaviorCompletion函数修改 - 修复重复的case
bool DecisionManager::checkBehaviorCompletion() {
    if (!blackboard_->isBehaviorInProgress()) return false;
    
    BehaviorInfo& behavior = blackboard_->current_behavior;
    
    // 检查行为是否完成
    switch (behavior.type) {
        case BehaviorType::ACTIVATE_ENERGY:
            if (behavior.state == BehaviorState::EXECUTING) {
                double elapsed_time = blackboard_->getExecutionElapsedTime();
                if (elapsed_time >= behavior.execution_duration) {
                    std::cout << "[DECISION] 行为完成: " << getBehaviorTypeName(behavior.type)
                              << ", 持续时间: " << elapsed_time << "秒" << std::endl;
                    
                    // 打符完成后设置energy_activated为true
                    blackboard_->energy_activated = true;
                    std::cout << "[INIT] 打符完成，设置energy_activated为true" << std::endl;
                    
                    blackboard_->completeCurrentBehavior();
                    
                    // 重置打符启动标志，准备开始前哨站攻击
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
                    
                    // 前哨站攻击完成，设置初始化完成
                    blackboard_->initialization_complete = true;
                    blackboard_->outpost_destroyed_init = true;
                    std::cout << "[INIT] 初始化流程完成，进入主决策循环" << std::endl;
                    
                    blackboard_->completeCurrentBehavior();
                    
                    // 重置前哨站启动标志
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
                
                // 关键修复：检查英雄是否仍然可见和存活
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
            
        case BehaviorType::RESURRECTION:
            if (behavior.state == BehaviorState::EXECUTING && 
                !blackboard_->resurrection_flag && blackboard_->current_hp >= 400.0) {
                std::cout << "[DECISION] 复活完成" << std::endl;
                blackboard_->completeCurrentBehavior();
                return true;
            }
            break;
            
        case BehaviorType::SUPPLY:
            if (behavior.state == BehaviorState::EXECUTING &&
                blackboard_->current_hp >= 400.0 && blackboard_->allowance_17mm >= 300.0) {
                std::cout << "[DECISION] 补给完成" << std::endl;
                blackboard_->completeCurrentBehavior();
                return true;
            }
            break;
            
        default:
            break;
    }
    
    return false;
}

// 开始新的行为决策 - 修复版本
bool DecisionManager::startNewBehaviorDecision() {
    if (!behavior_tree_) {
        std::cout << "[ERROR] behavior_tree_ is null!" << std::endl;
        return false;
    }
    
    // 关键修改：如果飞坡锁已激活或挂起，跳过主决策树
    if (blackboard_->isRampLockActive() || blackboard_->isRampLockPending()) {
        std::cout << "[DECISION] 飞坡锁激活，跳过主决策树" << std::endl;
        return false;
    }
    
    // 如果已经在飞坡流程中，不执行主决策树
    if (blackboard_->ramp_in_process) {
        std::cout << "[DECISION] 飞坡流程中，跳过主决策树" << std::endl;
        return false;
    }
    
    // 执行行为树决策
    BTStatus status = behavior_tree_->execute(blackboard_, region_manager_);
    std::cout << "[DECISION] 行为树执行状态: " << static_cast<int>(status) << std::endl;
    
    // 检查是否有新行为启动
    bool has_behavior_now = blackboard_->isBehaviorInProgress();
    
    if (status == BTStatus::RUNNING || status == BTStatus::SUCCESS) {
        // 如果有新行为启动，或者行为树指示有行为在进行
        if (has_behavior_now) {
            std::cout << "[DECISION] 行为树启动了新行为: " 
                      << getBehaviorTypeName(blackboard_->current_behavior.type) << std::endl;
            return true;
        }
        // 即使没有立即启动行为，RUNNING状态也表示行为树在执行某个流程
        else if (status == BTStatus::RUNNING) {
            std::cout << "[DECISION] 行为树在执行中" << std::endl;
            return true;
        }
    }
    
    std::cout << "[DECISION] 行为树没有启动新行为" << std::endl;
    return false;
}

// DecisionManager.cpp中的executeDecision函数 - 关键修改
DecisionOutput DecisionManager::executeDecision() {
    DecisionOutput output;
    output.target_needs_publishing = false;
    output.control_needs_publishing = false;
    
    // 比赛开始前（stage != 4）不发布任何目标点
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
    
    // 2. 检查飞坡流程（最高优先级）
    if (blackboard_->ramp_in_process) {
        // 确保当前行为是RAMP_PROCESS
        if (!blackboard_->isBehaviorInProgress() || blackboard_->current_behavior.type != BehaviorType::RAMP_PROCESS) {
            blackboard_->startBehavior(BehaviorType::RAMP_PROCESS, RED_RAMP_POINT, 0.0);
        }
        
        BehaviorInfo& behavior = blackboard_->current_behavior;
        
        switch (behavior.state) {
            case BehaviorState::MOVING: {
                // 检查是否到达飞坡点
                bool at_ramp_point = blackboard_->isAtTarget(behavior.target, 50.0);
                
                if (at_ramp_point && !blackboard_->at_current_target) {
                    // 到达飞坡点，切换姿态为飞坡模式
                    std::cout << "[RAMP_PROCESS] 到达飞坡点，开始飞坡" << std::endl;
                    
                    // 关键修改：设置飞坡锁为ACTIVE状态
                    blackboard_->setRampLockActive();
                    
                    blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
                    blackboard_->setTargetReached(true);
                    blackboard_->startExecutionTime();  // 开始执行计时
                    
                    // 关键：切换到飞坡姿态
                    blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_ON);
                    blackboard_->ramp_mode_active = true;
                    
                    // 如果保存了原始目标点，发布它
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
                    // 第一次开始移动，需要发布目标点（飞坡点）
                    output.target_position = behavior.target;
                    output.decision_reason = "RAMP_PROCESS_MOVING_TO_RAMP_POINT";
                    output.target_needs_publishing = true;
                    blackboard_->setTargetPublished(true);
                    
                    // 在前往飞坡点的过程中，设置移动姿态
                    blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
                    output.control_needs_publishing = true;
                    
                    std::cout << "[RAMP_PROCESS] 发布飞坡点: (" 
                              << output.target_position.x/100.0 << ", " 
                              << output.target_position.y/100.0 << ")" << std::endl;
                } else if (!at_ramp_point) {
                    // 还在移动中，不需要发布任何东西
                    output.decision_reason = "RAMP_PROCESS_MOVING_IN_PROGRESS";
                }
                break;
            }
                
            case BehaviorState::EXECUTING: {
                // 执行中：检查是否到达原始目标点
                bool at_original_target = false;
                if (blackboard_->original_target_before_ramp.x != 0 || 
                    blackboard_->original_target_before_ramp.y != 0) {
                    at_original_target = blackboard_->isAtTarget(blackboard_->original_target_before_ramp, 50.0);
                }
                
                if (at_original_target) {
                    // 到达原始目标点，完成飞坡流程
                    std::cout << "[RAMP_PROCESS] 到达原始目标点，飞坡流程完成" << std::endl;
                    
                    // 完成当前行为，这会自动解除飞坡锁
                    blackboard_->completeCurrentBehavior();
                    blackboard_->ramp_in_process = false;
                    blackboard_->ramp_mode_active = false;
                    blackboard_->original_target_before_ramp.x = 0;
                    blackboard_->original_target_before_ramp.y = 0;
                    output.decision_reason = "RAMP_PROCESS_COMPLETE";
                    
                } else {
                    // 还在飞坡中，保持飞坡控制消息
                    output.decision_reason = "RAMP_PROCESS_FLYING";
                    
                    // 确保控制消息是正确的飞坡模式
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
        
        // 如果控制消息已更新，需要发布
        if (blackboard_->isControlUpdated()) {
            output.control_needs_publishing = true;
            blackboard_->setControlUpdated(false);
        }
        
        return output;
    }
    
    // 3. 如果有行为在执行，继续执行当前行为
    if (blackboard_->isBehaviorInProgress()) {
        BehaviorInfo& behavior = blackboard_->current_behavior;
        
        // 跳过RAMP_PROCESS行为，它已经被上面的逻辑处理
        if (behavior.type == BehaviorType::RAMP_PROCESS) {
            // 不应该到达这里，但为了安全起见
            output.decision_reason = "RAMP_PROCESS_SKIPPED";
            output.control_msg = *blackboard_->getControlMsg();
            return output;
        }
        
        switch (behavior.state) {
            case BehaviorState::MOVING: {
                // 检查是否到达目标点
                bool at_target = blackboard_->isAtTarget(behavior.target, 50.0);
                
                if (at_target && !blackboard_->at_current_target) {
                    // 第一次到达目标点，切换姿态
                    std::cout << "[DECISION] 到达目标点，切换姿态" << std::endl;
                    blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
                    blackboard_->setTargetReached(true);
                    blackboard_->startExecutionTime();  // 开始执行计时
                    
                    // 根据行为类型设置控制消息
                    switch (behavior.type) {
                        case BehaviorType::ACTIVATE_ENERGY:
                            blackboard_->updateControlMsg(GIMBAL_ENERGY, SPIN_LOW, POSTURE_ATTACK, RAMP_OFF);
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
                        case BehaviorType::RESURRECTION:
                        case BehaviorType::SUPPLY:
                            blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
                            output.decision_reason = (behavior.type == BehaviorType::RESURRECTION) ? 
                                                     "ARRIVED_AT_RESURRECTION" : "ARRIVED_AT_SUPPLY";
                            break;
                        case BehaviorType::ATTACK_HERO:
                            // 检查是否在英雄部署区
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
                    
                    // 需要发布控制消息
                    output.control_needs_publishing = true;
                    
                } else if (!at_target && !blackboard_->isTargetPublished()) {
                    // 第一次开始移动，需要发布目标点
                    output.target_position = behavior.target;
                    output.decision_reason = "MOVING_TO_TARGET";
                    output.target_needs_publishing = true;
                    blackboard_->setTargetPublished(true);
                    
                    // 在前往目标点的过程中，设置移动姿态（除非正在飞坡）
                    if (!blackboard_->ramp_mode_active) {
                        blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
                        output.control_needs_publishing = true;
                    }
                    
                } else if (!at_target) {
                    // 还在移动中，不需要发布任何东西
                    output.decision_reason = "MOVING_IN_PROGRESS";
                }
                break;
            }
                
            case BehaviorState::EXECUTING: {
                // 执行中：保持控制消息，检查是否完成
                output.decision_reason = "EXECUTING_BEHAVIOR";
                
                // 确保控制消息是正确的（只发布一次）
                if (!blackboard_->isControlPublished()) {
                    output.control_needs_publishing = true;
                }
                break;
            }
                
            case BehaviorState::COMPLETED: {
                // 行为完成，重置状态，可以开始新的决策
                output.decision_reason = "BEHAVIOR_COMPLETE";
                blackboard_->resetCurrentBehavior();
                // 重置飞坡标志
                blackboard_->at_ramp_point = false;
                blackboard_->ramp_mode_active = false;
                break;
            }
                
            default:
                output.decision_reason = "UNKNOWN_BEHAVIOR_STATE";
                break;
        }
        
        output.control_msg = *blackboard_->getControlMsg();
        
        // 如果控制消息已更新，需要发布
        if (blackboard_->isControlUpdated()) {
            output.control_needs_publishing = true;
            blackboard_->setControlUpdated(false);
        }
        
        return output;
    }
    
    // 4. 没有行为在执行，开始新的决策
    
    // 检查复活状态（最高优先级）
    if (blackboard_->resurrection_flag) {
        // 开始复活行为
        blackboard_->startBehavior(BehaviorType::RESURRECTION, RED_SUPPLY, 0.0);
        
        output.target_position = RED_SUPPLY;
        output.decision_reason = "START_RESURRECTION";
        output.target_needs_publishing = true;
        output.control_msg = *blackboard_->getControlMsg();
        return output;
    }
    
    // 检查关键事件
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
    
    // 如果初始化已完成，直接进入主决策
    if (blackboard_->initialization_complete) {
        std::cout << "[MAIN] 进入主决策循环" << std::endl;
        
        // 先尝试执行行为树决策
        if (startNewBehaviorDecision()) {
            std::cout << "[MAIN] 行为树启动新行为: " 
                      << getBehaviorTypeName(blackboard_->current_behavior.type) << std::endl;
            
            BehaviorInfo& behavior = blackboard_->current_behavior;
            output.decision_reason = "START_NEW_BEHAVIOR_FROM_MAIN_TREE";
            
            if (behavior.state == BehaviorState::MOVING && !blackboard_->isTargetPublished()) {
                output.target_position = behavior.target;
                output.target_needs_publishing = true;
                
                // 在前往目标点的过程中，设置移动姿态
                blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
                output.control_needs_publishing = true;
            }
            output.control_msg = *blackboard_->getControlMsg();
            return output;
        }
        
        // 如果行为树没有返回新行为，再尝试其他条件
        // 检查关键事件（最高优先级）
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
        
        // 默认行为：前往基地增益点防御
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
    
    // 初始化阶段处理 - 关键修复：使用成员变量避免重复启动
    // 第一阶段：打符
    if (!blackboard_->energy_activated && !init_energy_started_ && !blackboard_->isBehaviorInProgress()) {
        std::cout << "[INIT] 开始打符" << std::endl;
        blackboard_->startBehavior(BehaviorType::ACTIVATE_ENERGY, RED_ENERGY_POINT, 10.0);
        init_energy_started_ = true;
    }
    // 第二阶段：前往前哨站攻击点
    else if (blackboard_->energy_activated && !blackboard_->initialization_complete && 
             !init_outpost_started_ && !blackboard_->isBehaviorInProgress()) {
        std::cout << "[INIT] 打符完成，前往前哨站攻击点" << std::endl;
        blackboard_->startBehavior(BehaviorType::ATTACK_OUTPOST, ENEMY_OUTPOST_ATTACK, 10.0);
        init_outpost_started_ = true;
        
        // 设置移动姿态并立即发布
        blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
        output.control_needs_publishing = true;
    }
    
    // 如果已经有行为在执行，处理输出
    if (blackboard_->isBehaviorInProgress()) {
        BehaviorInfo& behavior = blackboard_->current_behavior;
        
        if (behavior.state == BehaviorState::MOVING && !blackboard_->isTargetPublished()) {
            output.target_position = behavior.target;
            output.target_needs_publishing = true;
            
            // 确保移动过程中发布移动姿态
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

// 处理关键事件
void DecisionManager::handleCriticalEvent(GameStateProcessor::CriticalEvent event) {
    switch (event) {
        case GameStateProcessor::OUR_BASE_CRITICAL:
            {
                // 开始基地防御行为
                if (!blackboard_->isBehaviorInProgress()) {
                    blackboard_->startBehavior(BehaviorType::DEFEND_BASE, RED_BASE_GAIN, 10.0);
                    blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
                }
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
                    // 开始堡垒占领行为
                    if (!blackboard_->isBehaviorInProgress()) {
                        blackboard_->startBehavior(BehaviorType::OCCUPY_FORTRESS, ENEMY_FORTRESS_OCCUPY, 10.0);
                        blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
                    }
                } else {
                    // 开始基地防御行为
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
