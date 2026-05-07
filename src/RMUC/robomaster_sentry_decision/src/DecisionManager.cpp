#include "sentry_decision/DecisionManager.hpp"
#include "sentry_decision/GameConstants.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace GameConstants;

DecisionManager::DecisionManager()
    : blackboard_(std::make_shared<Blackboard>()),
      region_manager_(std::make_shared<RegionManager>()),
      current_state_(State::IDLE),
      last_state_entry_time_(0.0)
{}

void DecisionManager::updateOurState(const OurRobotState::SharedPtr msg) {
    blackboard_->updateOurState(msg);
}
void DecisionManager::updateEnemyState(const EnemyRobotState::SharedPtr msg) {
    blackboard_->updateEnemyState(msg);
}
void DecisionManager::updateGameState(const GameState::SharedPtr msg) {
    blackboard_->updateGameState(msg);
}

bool DecisionManager::needSupply() const {
    double hp_ratio = blackboard_->current_hp / blackboard_->getMaxHp();
    double ammo_ratio = blackboard_->allowance_17mm / blackboard_->getMaxAmmo();
    double threshold = blackboard_->getSupplyThreshold();
    return (hp_ratio < threshold || ammo_ratio < threshold);
}
bool DecisionManager::shouldInterruptForResurrectionOrSupply() const {
    return blackboard_->resurrection_flag || needSupply();
}
bool DecisionManager::checkBaseCritical() const {
    return blackboard_->our_base_hp < 1500;
}
bool DecisionManager::checkOutpostDestroyed() const {
    return blackboard_->our_outpost_hp == 0;
}
bool DecisionManager::checkFortressOccupy() const {
    double Z = Models::calculateSituationZ(*blackboard_);
    double distance = Models::calculateDistance(blackboard_->x, blackboard_->y,
                                                blackboard_->getFortressOccupyPoint().x,
                                                blackboard_->getFortressOccupyPoint().y);
    double F = Models::calculateFortressValue(*blackboard_, distance);
    double hp_ratio = blackboard_->current_hp / blackboard_->getMaxHp();
    return (Z > blackboard_->getFortressOccupyZThreshold() &&
            F > blackboard_->getFortressOccupyFThreshold() &&
            hp_ratio > blackboard_->getFortressOccupyHpRatio());
}

bool DecisionManager::checkGainPoint() const {
    auto scores = Models::calculateGainPointScores(*blackboard_);
    if (scores.empty()) return false;
    double Z = Models::calculateSituationZ(*blackboard_);
    double threshold;
    if (Z > blackboard_->getGainPointZHigh())
        threshold = blackboard_->getGainPointThresholdHigh();
    else if (Z >= blackboard_->getGainPointZMid())
        threshold = blackboard_->getGainPointThresholdMid();
    else
        threshold = blackboard_->getGainPointThresholdLow();
    return scores[0].score > threshold;
}

void DecisionManager::updateHeroDeployFlag() {
    blackboard_->hero_in_deploy_zone = region_manager_->isInHeroDeployZone(
        blackboard_->enemy_hero.x, blackboard_->enemy_hero.y);
}

PriorityTargetResult DecisionManager::selectPriorityTarget() {
    PriorityTargetResult result;
    const auto& configs = blackboard_->getPriorityTargets();

    for (const auto& cfg : configs) {
        double score = 0.0;
        std::string best_id;
        bool available = false;

        if (cfg.type == "hero_deploy") {
            if (blackboard_->enemy_hero.visible && blackboard_->enemy_hero.hp > 0 &&
                blackboard_->hero_in_deploy_zone) {
                double distance = Models::calculateDistance(blackboard_->x, blackboard_->y,
                                                            blackboard_->enemy_hero.x, blackboard_->enemy_hero.y);
                score = Models::calculateHeroAttackValue(*blackboard_, distance, true);
                best_id = "hero";
                available = true;
            }
        } else if (cfg.type == "hero") {
            if (blackboard_->enemy_hero.visible && blackboard_->enemy_hero.hp > 0) {
                score = Models::calculateGeneralTargetScore(*blackboard_, blackboard_->enemy_hero,
                                                            cfg.weight_hp, cfg.weight_distance, cfg.weight_ammo);
                best_id = "hero";
                available = true;
            }
        } else if (cfg.type == "engineer") {
            if (blackboard_->enemy_engineer.visible && blackboard_->enemy_engineer.hp > 0) {
                score = Models::calculateGeneralTargetScore(*blackboard_, blackboard_->enemy_engineer,
                                                            cfg.weight_hp, cfg.weight_distance, cfg.weight_ammo);
                best_id = "engineer";
                available = true;
            }
        } else if (cfg.type == "infantry") {
            double best_score = -1.0;
            auto checkInfantry = [&](const EnemyInfo& info, const std::string& id) {
                if (info.visible && info.hp > 0) {
                    double s = Models::calculateGeneralTargetScore(*blackboard_, info,
                                                                   cfg.weight_hp, cfg.weight_distance, cfg.weight_ammo);
                    if (s > best_score) {
                        best_score = s;
                        best_id = id;
                    }
                }
            };
            checkInfantry(blackboard_->enemy_infantry3, "infantry3");
            checkInfantry(blackboard_->enemy_infantry4, "infantry4");
            score = best_score;
            available = (best_score >= 0.0);
        } else if (cfg.type == "sentry") {
            if (blackboard_->enemy_sentry.visible && blackboard_->enemy_sentry.hp > 0) {
                score = Models::calculateGeneralTargetScore(*blackboard_, blackboard_->enemy_sentry,
                                                            cfg.weight_hp, cfg.weight_distance, cfg.weight_ammo);
                best_id = "sentry";
                available = true;
            }
        }

        std::cout << "[PRIORITY] " << cfg.type << " score=" << score
                  << " threshold=" << cfg.threshold;
        if (available && score > cfg.threshold) {
            std::cout << " -> SELECTED enemy_id=" << best_id << std::endl;
            result.enemy_id = best_id;
            result.type = cfg.type;
            result.score = score;
            result.valid = true;
            return result;
        } else {
            std::cout << " -> SKIP" << std::endl;
        }
    }

    return result;
}

geometry_msgs::msg::Point DecisionManager::getTargetPointForEnemy(const std::string& enemy_id) const {
    const EnemyInfo* enemy = blackboard_->getEnemyById(enemy_id);
    if (!enemy || !enemy->visible || enemy->hp <= 0) {
        return geometry_msgs::msg::Point();
    }
    geometry_msgs::msg::Point hex = region_manager_->findSameRegionHexPoint(
        enemy->x, enemy->y, blackboard_->x, blackboard_->y);
    return region_manager_->clampPointToAllowedRegion(hex);
}

Models::GainPointScore DecisionManager::getBestGainPoint() const {
    auto scores = Models::calculateGainPointScores(*blackboard_);
    if (scores.empty()) {
        Models::GainPointScore empty;
        empty.score = 0.0;
        return empty;
    }
    return scores[0];
}

void DecisionManager::transitionTo(State new_state) {
    if (current_state_ == new_state) return;
    std::cout << "[STATE] " << stateToString(current_state_) << " -> " << stateToString(new_state) << std::endl;
    current_state_ = new_state;
    last_state_entry_time_ = blackboard_->current_time;

    blackboard_->resetAllPublishStates();

    switch (new_state) {
        case State::IDLE:
            blackboard_->resetCurrentBehavior();
            blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
            break;
        case State::INIT_MOVE:
            blackboard_->startBehavior(BehaviorType::INIT_MOVE, blackboard_->getAttackPoint(), 0.0);
            blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
            break;
        case State::INIT_ATTACK:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_OUTPOST, SPIN_LOW, POSTURE_ATTACK, RAMP_OFF);
            break;
        case State::MOVE_TO_ATTACK_HERO:
        case State::MOVE_TO_ATTACK_ROBOT: {
            geometry_msgs::msg::Point target = getTargetPointForEnemy(current_enemy_id_);
            if (target.x == 0 && target.y == 0) {
                transitionTo(State::IDLE);
                return;
            }
            BehaviorType bt = (new_state == State::MOVE_TO_ATTACK_HERO) ?
                              BehaviorType::MOVE_TO_ATTACK_HERO : BehaviorType::MOVE_TO_ATTACK_ROBOT;
            blackboard_->startBehavior(bt, target, 0.0);
            blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
            break;
        }
        case State::MOVE_TO_SUPPLY:
        case State::RESURRECTION_MOVE:
        case State::MOVE_TO_BASE_DEFENSE:
        case State::MOVE_TO_GAIN_POINT:
        case State::MOVE_TO_FORTRESS:
        case State::MOVE_TO_GUARD:       // 新增
            blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
            break;
        case State::ATTACK_HERO:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_VARIABLE, POSTURE_ATTACK, RAMP_OFF);
            break;
        case State::ATTACK_ROBOT:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_VARIABLE, POSTURE_ATTACK, RAMP_OFF);
            break;
        case State::SUPPLYING:
            blackboard_->current_behavior.type = BehaviorType::SUPPLY;
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
            break;
        case State::RESURRECTING:
            blackboard_->current_behavior.type = BehaviorType::RESURRECTING;
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
            break;
        case State::BASE_DEFENSE:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
            break;
        case State::OCCUPY_GAIN_POINT:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
            break;
        case State::OCCUPY_FORTRESS:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_LOW, POSTURE_ATTACK, RAMP_OFF);
            break;
        case State::GUARD:               // 新增
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
            break;
        default:
            break;
    }
}

std::string DecisionManager::stateToString(State state) const {
    switch (state) {
        case State::IDLE: return "IDLE";
        case State::INIT_MOVE: return "INIT_MOVE";
        case State::INIT_ATTACK: return "INIT_ATTACK";
        case State::MOVE_TO_ATTACK_HERO: return "MOVE_TO_ATTACK_HERO";
        case State::ATTACK_HERO: return "ATTACK_HERO";
        case State::MOVE_TO_ATTACK_ROBOT: return "MOVE_TO_ATTACK_ROBOT";
        case State::ATTACK_ROBOT: return "ATTACK_ROBOT";
        case State::MOVE_TO_SUPPLY: return "MOVE_TO_SUPPLY";
        case State::SUPPLYING: return "SUPPLYING";
        case State::RESURRECTION_MOVE: return "RESURRECTION_MOVE";
        case State::RESURRECTING: return "RESURRECTING";
        case State::MOVE_TO_BASE_DEFENSE: return "MOVE_TO_BASE_DEFENSE";
        case State::BASE_DEFENSE: return "BASE_DEFENSE";
        case State::MOVE_TO_GAIN_POINT: return "MOVE_TO_GAIN_POINT";
        case State::OCCUPY_GAIN_POINT: return "OCCUPY_GAIN_POINT";
        case State::MOVE_TO_FORTRESS: return "MOVE_TO_FORTRESS";
        case State::OCCUPY_FORTRESS: return "OCCUPY_FORTRESS";
        case State::RAMP_PROCESS: return "RAMP_PROCESS";
        case State::MOVE_TO_GUARD: return "MOVE_TO_GUARD";
        case State::GUARD: return "GUARD";
        default: return "UNKNOWN";
    }
}

bool DecisionManager::handleRampProcess(DecisionOutput& output) {
    static enum RampSubState {
        RAMP_MOVING_TO_RAMP,
        RAMP_FLYING,
        RAMP_MOVING_TO_ORIGINAL
    } ramp_substate = RAMP_MOVING_TO_RAMP;

    if (!blackboard_->ramp_in_process) {
        blackboard_->ramp_in_process = true;
        blackboard_->activateRampLock();
        ramp_substate = RAMP_MOVING_TO_RAMP;
        blackboard_->startBehavior(BehaviorType::RAMP_PROCESS, blackboard_->getRampPoint(), 0.0);
        blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
        output.control_needs_publishing = true;
        output.control_msg = *blackboard_->getControlMsg();
        return true;
    }

    switch (ramp_substate) {
        case RAMP_MOVING_TO_RAMP: {
            geometry_msgs::msg::Point ramp_pt = blackboard_->getRampPoint();
            if (blackboard_->isAtTarget(ramp_pt, 50.0)) {
                std::cout << "[RAMP] 到达飞坡点，开始飞坡" << std::endl;
                blackboard_->setRampLockActive();
                blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_ON);
                output.control_needs_publishing = true;
                ramp_substate = RAMP_FLYING;
                if (blackboard_->original_target_before_ramp.x != 0 || blackboard_->original_target_before_ramp.y != 0) {
                    output.target_position = blackboard_->original_target_before_ramp;
                    output.target_needs_publishing = true;
                    blackboard_->setTargetPublished(true);
                }
            } else {
                if (!blackboard_->isTargetPublished()) {
                    output.target_position = ramp_pt;
                    output.target_needs_publishing = true;
                    blackboard_->setTargetPublished(true);
                }
            }
            output.control_msg = *blackboard_->getControlMsg();
            return true;
        }
        case RAMP_FLYING: {
            if (blackboard_->original_target_before_ramp.x != 0 || blackboard_->original_target_before_ramp.y != 0) {
                if (blackboard_->isAtTarget(blackboard_->original_target_before_ramp, 50.0)) {
                    std::cout << "[RAMP] 飞坡完成，到达目标" << std::endl;
                    blackboard_->deactivateRampLock();
                    blackboard_->ramp_in_process = false;
                    blackboard_->original_target_before_ramp = geometry_msgs::msg::Point();
                    blackboard_->ramp_mode_active = false;
                    blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
                    output.control_needs_publishing = true;
                    ramp_substate = RAMP_MOVING_TO_RAMP;
                    transitionTo(State::IDLE);
                    return false;
                }
            } else {
                blackboard_->deactivateRampLock();
                blackboard_->ramp_in_process = false;
                transitionTo(State::IDLE);
                return false;
            }
            output.control_msg = *blackboard_->getControlMsg();
            return true;
        }
        default:
            blackboard_->ramp_in_process = false;
            transitionTo(State::IDLE);
            return false;
    }
}

DecisionOutput DecisionManager::executeDecision() {
    DecisionOutput output;
    output.target_needs_publishing = false;
    output.control_needs_publishing = false;

    updateHeroDeployFlag();

    if (blackboard_->stage != STAGE_BATTLE) {
        if (current_state_ != State::IDLE) transitionTo(State::IDLE);
        else {
            uint8_t gimbal = (blackboard_->stage == 3 || blackboard_->stage == 4) ? GIMBAL_ENEMY : GIMBAL_IDLE;
            blackboard_->updateControlMsg(gimbal, SPIN_OFF, POSTURE_MOVE, RAMP_OFF);
        }
        output.control_msg = *blackboard_->getControlMsg();
        output.decision_reason = "NOT_IN_BATTLE";
        if (blackboard_->isControlUpdated()) {
            output.control_needs_publishing = true;
            blackboard_->setControlUpdated(false);
        }
        return output;
    }

    if (blackboard_->ramp_in_process || blackboard_->isRampLockPending()) {
        if (handleRampProcess(output)) return output;
    }

    switch (current_state_) {
        case State::IDLE: {
            if (blackboard_->resurrection_flag) {
                transitionTo(State::RESURRECTION_MOVE);
            } else if (needSupply()) {
                transitionTo(State::MOVE_TO_SUPPLY);
            } else if (checkBaseCritical()) {
                transitionTo(State::MOVE_TO_BASE_DEFENSE);
            } else if (checkOutpostDestroyed() && checkFortressOccupy()) {
                transitionTo(State::MOVE_TO_FORTRESS);
            }
            else if (!blackboard_->initialization_complete) {
                transitionTo(State::INIT_MOVE);
            }
            else {
                PriorityTargetResult ptarget = selectPriorityTarget();
                if (ptarget.valid) {
                    current_enemy_id_ = ptarget.enemy_id;
                    if (ptarget.type == "hero" || ptarget.type == "hero_deploy") {
                        transitionTo(State::MOVE_TO_ATTACK_HERO);
                    } else {
                        transitionTo(State::MOVE_TO_ATTACK_ROBOT);
                    }
                } else if (checkGainPoint()) {
                    transitionTo(State::MOVE_TO_GAIN_POINT);
                } else {
                    // 无攻击目标，移动到固定警戒点
                    transitionTo(State::MOVE_TO_GUARD);
                }
            }
            break;
        }
        case State::INIT_MOVE: {
            geometry_msgs::msg::Point target = blackboard_->getAttackPoint();
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) blackboard_->setTargetReached(true);
                if (blackboard_->hasWaitedAtTarget(blackboard_->getArrivalWaitTime()))
                    transitionTo(State::INIT_ATTACK);
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::INIT_ATTACK: {
            if (shouldInterruptForResurrectionOrSupply()) {
                transitionTo(State::IDLE);
                break;
            }
            double elapsed = blackboard_->getExecutionElapsedTime();
            if (elapsed >= blackboard_->getInitAttackDuration()) {
                blackboard_->initialization_complete = true;
                transitionTo(State::IDLE);
            }
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_OUTPOST, SPIN_LOW, POSTURE_ATTACK, RAMP_OFF);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::MOVE_TO_ATTACK_HERO:
        case State::MOVE_TO_ATTACK_ROBOT: {
            if (shouldInterruptForResurrectionOrSupply()) {
                transitionTo(State::IDLE);
                break;
            }
            geometry_msgs::msg::Point target = getTargetPointForEnemy(current_enemy_id_);
            if (target.x == 0 && target.y == 0) {
                transitionTo(State::IDLE);
                break;
            }
            bool robot_in_red_half = (blackboard_->x < blackboard_->getHalfMapX());
            bool target_in_blue_half = (target.x >= blackboard_->getHalfMapX());
            bool robot_not_central = !region_manager_->isInCentralRegion(blackboard_->x, blackboard_->y);
            bool target_not_central = !region_manager_->isInCentralRegion(target.x, target.y);

            if (robot_in_red_half && target_in_blue_half &&
                blackboard_->isRampLockInactive() &&
                robot_not_central && target_not_central)
            {
                blackboard_->original_target_before_ramp = target;
                blackboard_->ramp_in_process = true;
                blackboard_->activateRampLock();
                transitionTo(State::RAMP_PROCESS);
                break;
            }
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) blackboard_->setTargetReached(true);
                if (blackboard_->hasWaitedAtTarget(blackboard_->getArrivalWaitTime())) {
                    if (current_state_ == State::MOVE_TO_ATTACK_HERO)
                        transitionTo(State::ATTACK_HERO);
                    else
                        transitionTo(State::ATTACK_ROBOT);
                }
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::ATTACK_HERO:
        case State::ATTACK_ROBOT: {
            if (shouldInterruptForResurrectionOrSupply()) {
                transitionTo(State::IDLE);
                break;
            }
            geometry_msgs::msg::Point target = getTargetPointForEnemy(current_enemy_id_);
            if (target.x == 0 || target.y == 0) {
                transitionTo(State::IDLE);
                break;
            }
            if (!blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (current_state_ == State::ATTACK_HERO)
                    transitionTo(State::MOVE_TO_ATTACK_HERO);
                else
                    transitionTo(State::MOVE_TO_ATTACK_ROBOT);
                break;
            }
            double elapsed = blackboard_->getExecutionElapsedTime();
            if (elapsed >= blackboard_->getAttackDuration())
                transitionTo(State::IDLE);
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_VARIABLE, POSTURE_ATTACK, RAMP_OFF);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::MOVE_TO_SUPPLY:
        case State::RESURRECTION_MOVE: {
            geometry_msgs::msg::Point target = blackboard_->getSupplyPoint();
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) blackboard_->setTargetReached(true);
                if (blackboard_->hasWaitedAtTarget(blackboard_->getArrivalWaitTime())) {
                    if (current_state_ == State::RESURRECTION_MOVE)
                        transitionTo(State::RESURRECTING);
                    else
                        transitionTo(State::SUPPLYING);
                }
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::SUPPLYING: {
            if (blackboard_->current_hp >= blackboard_->getMaxHp() &&
                blackboard_->allowance_17mm >= blackboard_->getMaxAmmo())
                transitionTo(State::IDLE);
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::RESURRECTING: {
            if (!blackboard_->resurrection_flag && blackboard_->current_hp >= blackboard_->getMaxHp())
                transitionTo(State::IDLE);
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::MOVE_TO_BASE_DEFENSE: {
            geometry_msgs::msg::Point target = blackboard_->getBaseGainPoint();
            if (shouldInterruptForResurrectionOrSupply()) {
                transitionTo(State::IDLE);
                break;
            }
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) blackboard_->setTargetReached(true);
                if (blackboard_->hasWaitedAtTarget(blackboard_->getArrivalWaitTime()))
                    transitionTo(State::BASE_DEFENSE);
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::BASE_DEFENSE: {
            if (shouldInterruptForResurrectionOrSupply()) {
                transitionTo(State::IDLE);
                break;
            }
            double elapsed = blackboard_->getExecutionElapsedTime();
            if (elapsed >= blackboard_->getDefendDuration())
                transitionTo(State::IDLE);
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::MOVE_TO_GAIN_POINT: {
            auto best = getBestGainPoint();
            if (best.name.empty()) {
                transitionTo(State::IDLE);
                break;
            }
            geometry_msgs::msg::Point target = region_manager_->clampPointToAllowedRegion(best.position);
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) blackboard_->setTargetReached(true);
                if (blackboard_->hasWaitedAtTarget(blackboard_->getArrivalWaitTime()))
                    transitionTo(State::OCCUPY_GAIN_POINT);
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::OCCUPY_GAIN_POINT: {
            if (shouldInterruptForResurrectionOrSupply()) {
                transitionTo(State::IDLE);
                break;
            }
            double elapsed = blackboard_->getExecutionElapsedTime();
            if (elapsed >= blackboard_->getDefendDuration())
                transitionTo(State::IDLE);
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::MOVE_TO_FORTRESS: {
            geometry_msgs::msg::Point target = blackboard_->getFortressOccupyPoint();
            if (shouldInterruptForResurrectionOrSupply()) {
                transitionTo(State::IDLE);
                break;
            }
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) blackboard_->setTargetReached(true);
                if (blackboard_->hasWaitedAtTarget(blackboard_->getArrivalWaitTime()))
                    transitionTo(State::OCCUPY_FORTRESS);
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::OCCUPY_FORTRESS: {
            if (shouldInterruptForResurrectionOrSupply()) {
                transitionTo(State::IDLE);
                break;
            }
            double elapsed = blackboard_->getExecutionElapsedTime();
            if (elapsed >= blackboard_->getDefendDuration())
                transitionTo(State::IDLE);
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_LOW, POSTURE_ATTACK, RAMP_OFF);
                output.control_needs_publishing = true;
            }
            break;
        }
        // ---------- 新增警戒状态处理 ----------
        case State::MOVE_TO_GUARD: {
            geometry_msgs::msg::Point target;
            target.x = GUARD_X;
            target.y = GUARD_Y;
            // 高优先级打断
            if (shouldInterruptForResurrectionOrSupply() || checkBaseCritical()) {
                transitionTo(State::IDLE);   // 回到 IDLE 重新判断
                break;
            }
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) blackboard_->setTargetReached(true);
                transitionTo(State::GUARD);
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::GUARD: {
            // 检查更高优先级任务
            if (shouldInterruptForResurrectionOrSupply() || checkBaseCritical()) {
                transitionTo(State::IDLE);
                break;
            }
            // 警戒期间也尝试搜索攻击目标，若发现则退出警戒
            PriorityTargetResult ptarget = selectPriorityTarget();
            if (ptarget.valid) {
                current_enemy_id_ = ptarget.enemy_id;
                if (ptarget.type == "hero" || ptarget.type == "hero_deploy") {
                    transitionTo(State::MOVE_TO_ATTACK_HERO);
                } else {
                    transitionTo(State::MOVE_TO_ATTACK_ROBOT);
                }
                break;
            }
            // 固定时间后重返 IDLE 重新全面评估（避免永远警戒）
            double elapsed = blackboard_->getExecutionElapsedTime();
            if (elapsed >= 5.0) {   // 警戒 5 秒后回 IDLE
                transitionTo(State::IDLE);
                break;
            }
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_LOW, POSTURE_DEFENSE, RAMP_OFF);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::RAMP_PROCESS:
            transitionTo(State::IDLE);
            break;
    }

    output.control_msg = *blackboard_->getControlMsg();
    if (blackboard_->isControlUpdated()) {
        output.control_needs_publishing = true;
        blackboard_->setControlUpdated(false);
    }
    output.decision_reason = stateToString(current_state_);
    return output;
}
