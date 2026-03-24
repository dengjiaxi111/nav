#include "sentry_decision/DecisionManager.hpp"
#include "sentry_decision/Constants.hpp"
#include <iostream>
#include <cmath>

using namespace SentryConstants;

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
    return Z < SUPPLY_THRESHOLD;
}

bool DecisionManager::shouldInterruptAttack() const {
    return blackboard_->resurrection_flag || needSupply();
}

bool DecisionManager::supplyComplete() const {
    return blackboard_->current_hp >= 400.0 && blackboard_->allowance_17mm >= 300.0;
}

bool DecisionManager::resurrectionComplete() const {
    return !blackboard_->resurrection_flag && blackboard_->current_hp >= 400.0;
}

void DecisionManager::transitionTo(State new_state) {
    if (current_state_ == new_state) return;

    std::cout << "[STATE] " << stateToString(current_state_) << " -> " << stateToString(new_state) << std::endl;

    current_state_ = new_state;
    last_state_entry_time_ = blackboard_->current_time;

    blackboard_->resetAllPublishStates();

    switch (new_state) {
        case State::MOVING_TO_ATTACK:
            blackboard_->startBehavior(BehaviorType::MOVE_TO_ATTACK, blackboard_->getAttackPoint(), 0.0);
            blackboard_->updateControlMsg(SPIN_OFF);
            break;
        case State::MOVING_TO_SUPPLY:
            blackboard_->startBehavior(BehaviorType::MOVE_TO_SUPPLY, blackboard_->getSupplyPoint(), 0.0);
            blackboard_->updateControlMsg(SPIN_OFF);
            break;
        case State::ATTACKING:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(SPIN_ON);
            break;
        case State::SUPPLYING:
            blackboard_->current_behavior.type = BehaviorType::SUPPLY;
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(SPIN_ON);
            break;
        case State::RESURRECTING:
            blackboard_->current_behavior.type = BehaviorType::RESURRECTION;
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(SPIN_ON);
            break;
        case State::IDLE:
            blackboard_->resetCurrentBehavior();
            blackboard_->updateControlMsg(SPIN_OFF);
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

    if (blackboard_->stage != STAGE_BATTLE) {
        if (current_state_ != State::IDLE) {
            transitionTo(State::IDLE);
        } else {
            blackboard_->updateControlMsg(SPIN_OFF);
        }
        output.control_msg = *blackboard_->getControlMsg();
        output.decision_reason = "WAITING_FOR_START";
        if (blackboard_->isControlUpdated()) {
            output.control_needs_publishing = true;
            blackboard_->setControlUpdated(false);
        }
        return output;
    }

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
            if (blackboard_->isAtTarget(blackboard_->getAttackPoint(), 50.0)) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                }
                if (blackboard_->hasWaitedAtTarget(ARRIVAL_WAIT_TIME)) {
                    transitionTo(State::ATTACKING);
                }
            } else {
                if (blackboard_->at_current_target) {
                    blackboard_->setTargetReached(false);
                }
                if (!blackboard_->isTargetPublished()) {
                    output.target_position = blackboard_->getAttackPoint();
                    output.target_needs_publishing = true;
                }
            }
            break;
        }

        case State::MOVING_TO_SUPPLY: {
            geometry_msgs::msg::Point supply_point = blackboard_->getSupplyPoint();
            if (blackboard_->isAtTarget(supply_point, 50.0)) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                }
                if (blackboard_->hasWaitedAtTarget(ARRIVAL_WAIT_TIME)) {
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
                if (!blackboard_->isTargetPublished()) {
                    output.target_position = supply_point;
                    output.target_needs_publishing = true;
                }
            }
            break;
        }

        case State::ATTACKING: {
            if (shouldInterruptAttack()) {
                transitionTo(State::MOVING_TO_SUPPLY);
                break;
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
