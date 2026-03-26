#include "sentry_decision/DecisionManager.hpp"
#include "sentry_decision/Models.hpp"
#include "sentry_decision/GameConstants.hpp"
#include <iostream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using namespace GameConstants;

DecisionManager::DecisionManager()
    : blackboard_(std::make_shared<Blackboard>()) {
    current_state_ = State::IDLE;
    last_state_entry_time_ = 0.0;
}

void DecisionManager::updateOurState(const OurRobotState::SharedPtr msg) {
    blackboard_->updateOurState(msg);
}

void DecisionManager::updateGameState(const GameState::SharedPtr msg) {
    blackboard_->updateGameState(msg);
}

bool DecisionManager::needSupply() const {
    double Z = Models::calculateSituationZ(*blackboard_);
    return Z < blackboard_->getSupplyThreshold();
}

bool DecisionManager::shouldInterruptAttack() const {
    return blackboard_->resurrection_flag || needSupply();
}

bool DecisionManager::supplyComplete() const {
    return blackboard_->current_hp >= blackboard_->getMaxHp();
}

bool DecisionManager::resurrectionComplete() const {
    return blackboard_->current_hp >= blackboard_->getMaxHp();
}

void DecisionManager::transitionTo(State new_state) {
    if (current_state_ == new_state) return;

    std::cout << "[STATE] " << stateToString(current_state_) << " -> " << stateToString(new_state) << std::endl;

    current_state_ = new_state;
    last_state_entry_time_ = blackboard_->current_time;

    blackboard_->resetAllPublishStates();

    uint8_t gimbal_mode = blackboard_->getGimbalModeByStage();

    switch (new_state) {
        case State::MOVING_TO_ATTACK:
            blackboard_->startBehavior(BehaviorType::MOVE_TO_ATTACK, blackboard_->getAttackPoint(), 0.0);
            blackboard_->updateControlMsg(gimbal_mode, SPIN_OFF);
            break;
        case State::MOVING_TO_SUPPLY:
            blackboard_->startBehavior(BehaviorType::MOVE_TO_SUPPLY, blackboard_->getSupplyPoint(), 0.0);
            blackboard_->updateControlMsg(gimbal_mode, SPIN_OFF);
            break;
        case State::ATTACKING:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(gimbal_mode, SPIN_ON);
            break;
        case State::SUPPLYING:
            blackboard_->current_behavior.type = BehaviorType::SUPPLY;
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(gimbal_mode, SPIN_OFF);
            break;
        case State::RESURRECTING:
            blackboard_->current_behavior.type = BehaviorType::RESURRECTION;
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(gimbal_mode, SPIN_OFF);
            break;
        case State::IDLE:
            blackboard_->resetCurrentBehavior();
            blackboard_->updateControlMsg(gimbal_mode, SPIN_OFF);
            break;
        default:
            break;
    }
}

std::string DecisionManager::stateToString(State state) const {
    switch (state) {
        case State::IDLE: return "IDLE";
        case State::MOVING_TO_ATTACK: return "MOVING_TO_ATTACK";
        case State::MOVING_TO_SUPPLY: return "MOVING_TO_SUPPLY";
        case State::ATTACKING: return "ATTACKING";
        case State::SUPPLYING: return "SUPPLYING";
        case State::RESURRECTING: return "RESURRECTING";
        default: return "UNKNOWN";
    }
}

DecisionOutput DecisionManager::executeDecision() {
    DecisionOutput output;
    output.target_needs_publishing = false;
    output.control_needs_publishing = false;

    // 非比赛阶段
    if (blackboard_->stage != STAGE_BATTLE) {
        if (current_state_ != State::IDLE) {
            transitionTo(State::IDLE);
        } else {
            // 根据阶段设置云台模式
            uint8_t gimbal_mode = blackboard_->getGimbalModeByStage();
            blackboard_->updateControlMsg(gimbal_mode, SPIN_OFF);
        }
        output.control_msg = *blackboard_->getControlMsg();
        output.decision_reason = "WAITING_FOR_START";
        if (blackboard_->isControlUpdated()) {
            output.control_needs_publishing = true;
            blackboard_->setControlUpdated(false);
        }
        return output;
    }

    // 比赛阶段状态机
    switch (current_state_) {
        case State::IDLE: {
            if (blackboard_->resurrection_flag) {
                transitionTo(State::MOVING_TO_SUPPLY);
            } else if (needSupply()) {
                transitionTo(State::MOVING_TO_SUPPLY);
            } else {
                transitionTo(State::MOVING_TO_ATTACK);
            }
            break;
        }

        case State::MOVING_TO_ATTACK: {
            if (shouldInterruptAttack()) {
                if (blackboard_->at_current_target) {
                    blackboard_->setTargetReached(false);
                }
                transitionTo(State::MOVING_TO_SUPPLY);
                break;
            }
            if (blackboard_->isAtTarget(blackboard_->getAttackPoint(), blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                }
                if (blackboard_->hasWaitedAtTarget(blackboard_->getArrivalWaitTime())) {
                    transitionTo(State::ATTACKING);
                }
            } else {
                if (blackboard_->at_current_target) {
                    blackboard_->setTargetReached(false);
                }
                // 只要未到达，就持续发布目标点
                output.target_position = blackboard_->getAttackPoint();
                output.target_needs_publishing = true;
            }
            break;
        }

        case State::MOVING_TO_SUPPLY: {
            if (blackboard_->current_hp >= blackboard_->getMaxHp()) {
                transitionTo(State::IDLE);
                break;
            }
            geometry_msgs::msg::Point supply_point = blackboard_->getSupplyPoint();
            if (blackboard_->isAtTarget(supply_point, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                }
                if (blackboard_->hasWaitedAtTarget(blackboard_->getArrivalWaitTime())) {
                    if (blackboard_->resurrection_flag) {
                        transitionTo(State::RESURRECTING);
                    } else {
                        transitionTo(State::SUPPLYING);
                    }
                }
            } else {
                if (blackboard_->at_current_target) {
                    blackboard_->setTargetReached(false);
                }
                // 持续发布目标点
                output.target_position = supply_point;
                output.target_needs_publishing = true;
            }
            break;
        }

        case State::ATTACKING: {
            if (shouldInterruptAttack()) {
                transitionTo(State::MOVING_TO_SUPPLY);
                break;
            }
            // 偏离检测
            geometry_msgs::msg::Point attack_point = blackboard_->getAttackPoint();
            if (!blackboard_->isAtTarget(attack_point, blackboard_->getDeviationThreshold())) {
                transitionTo(State::MOVING_TO_ATTACK);
            }
            break;
        }

        case State::SUPPLYING: {
            if (supplyComplete()) {
                transitionTo(State::IDLE);
            }
            break;
        }

        case State::RESURRECTING: {
            if (resurrectionComplete()) {
                transitionTo(State::IDLE);
            }
            break;
        }
    }

    output.control_msg = *blackboard_->getControlMsg();
    output.decision_reason = stateToString(current_state_);

    if (blackboard_->isControlUpdated()) {
        output.control_needs_publishing = true;
        blackboard_->setControlUpdated(false);
    }

    return output;
}
