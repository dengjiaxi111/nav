#include "sentry_decision/DecisionManager.hpp"
#include "sentry_decision/GameConstants.hpp"
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
    double threshold = blackboard_->getSupplyThreshold();
    return (hp_ratio < threshold || blackboard_->allowance_17mm <= 0.0);
}
bool DecisionManager::shouldInterruptForResurrectionOrSupply() const {
    return blackboard_->resurrection_flag || needSupply();
}
bool DecisionManager::isInitAttackForcedTimerActive() const {
    return init_attack_forced_start_time_ >= 0.0 &&
           (blackboard_->current_time - init_attack_forced_start_time_) < blackboard_->getInitAttackDuration() &&
           !blackboard_->initialization_complete;
}
bool DecisionManager::checkBaseCritical() const {
    return blackboard_->our_base_hp < 2500;
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

bool DecisionManager::checkEnemyFortress() const {
    if (blackboard_->must_occupy_enemy_fortress) return true;
    if (blackboard_->current_time < blackboard_->getEnemyFortressOccupyTime()) return false;
    if (blackboard_->robot_id_ == 1) {
        if (blackboard_->base_open == 1) return false;
    } else {
        if (blackboard_->base_open == 2) return false;
    }
    double hp_ratio = blackboard_->current_hp / blackboard_->getMaxHp();
    double ammo_ratio = blackboard_->allowance_17mm / blackboard_->getMaxAmmo();
    if (hp_ratio < blackboard_->getEnemyFortressHpThreshold() ||
        ammo_ratio < blackboard_->getEnemyFortressAmmoThreshold()) return false;
    return (blackboard_->enemy_fortress_gain_point_occupation == 0);
}

void DecisionManager::updateMustOccupyFlag() {
    if (!blackboard_->must_occupy_enemy_fortress) {
        if (blackboard_->enemy_hero.hp <= 0 || blackboard_->enemy_engineer.hp <= 0) {
            blackboard_->must_occupy_enemy_fortress = true;
        }
    }
}

void DecisionManager::updateHeroDeployFlag() {
    blackboard_->hero_in_deploy_zone = region_manager_->isInEnemyHeroDeployZone(
        blackboard_->enemy_hero.x, blackboard_->enemy_hero.y, blackboard_->robot_id_);
}

PriorityTargetResult DecisionManager::selectPriorityTarget() {
    PriorityTargetResult result;
    const auto& configs = blackboard_->getPriorityTargets();

    for (const auto& cfg : configs) {
        double score = 0.0;
        std::string best_id;
        bool available = false;

        if (cfg.type == "hero_deploy" || cfg.type == "hero") {
            if (blackboard_->enemy_hero.visible && blackboard_->enemy_hero.hp > 0) {
                score = Models::calculateGeneralTargetScore(*blackboard_, blackboard_->enemy_hero,
                                                            cfg.weight_hp, cfg.weight_distance, cfg.weight_ammo);
                best_id = "hero";
                available = true;
            }
        } else if (cfg.type == "engineer") {
            bool in_zone = region_manager_->isInEnemyEngineerMiningZone(
                blackboard_->enemy_engineer.x, blackboard_->enemy_engineer.y, blackboard_->robot_id_);
            if (!in_zone && blackboard_->enemy_engineer.visible && blackboard_->enemy_engineer.hp > 0) {
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

        if (available && score > cfg.threshold) {
            result.enemy_id = best_id;
            result.type = cfg.type;
            result.score = score;
            result.valid = true;
            return result;
        }
    }
    return result;
}

geometry_msgs::msg::Point DecisionManager::constrainTargetPoint(const geometry_msgs::msg::Point& point,
                                                               bool clamp_to_allowed_region) const {
    geometry_msgs::msg::Point target = point;
    if (!clamp_to_allowed_region) {
        return target;
    }
    if (blackboard_->enemy_fortress_gain_point_captured_by_us) {
        constexpr double MAP_MID_X = 1400.0;
        if (blackboard_->robot_id_ == 1) {
            target.x = std::max(target.x, MAP_MID_X);
        } else {
            target.x = std::min(target.x, MAP_MID_X);
        }
    }
    return region_manager_->clampPointToAllowedRegion(target, blackboard_->x, blackboard_->y);
}

geometry_msgs::msg::Point DecisionManager::getTargetPointForEnemy(const std::string& enemy_id) const {
    const EnemyInfo* enemy = blackboard_->getEnemyById(enemy_id);
    if (!enemy || !enemy->visible || enemy->hp <= 0) {
        return geometry_msgs::msg::Point();
    }
    geometry_msgs::msg::Point hex = region_manager_->findSameRegionHexPoint(
        enemy->x, enemy->y, blackboard_->x, blackboard_->y);
    return constrainTargetPoint(hex);
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

bool DecisionManager::beginTargetOffsetCorrection(State move_state, DecisionOutput& output) {
    const auto& target = blackboard_->current_behavior.target;
    if ((target.x == 0.0 && target.y == 0.0) ||
        blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
        return false;
    }

    correcting_target_offset_ = true;
    correction_resume_state_ = current_state_;
    correction_target_ = target;
    transitionTo(move_state);
    blackboard_->current_behavior.target = correction_target_;
    blackboard_->updateControlMsg(blackboard_->getControlMsg()->gimbal_mode, SPIN_OFF, POSTURE_MOVE);
    output.target_position = correction_target_;
    output.target_needs_publishing = true;
    output.control_needs_publishing = true;
    return true;
}

State DecisionManager::consumeCorrectionResumeState(State normal_state) {
    if (!correcting_target_offset_) return normal_state;
    State resume_state = correction_resume_state_;
    correcting_target_offset_ = false;
    correction_resume_state_ = State::IDLE;
    correction_target_ = geometry_msgs::msg::Point();
    return resume_state;
}

geometry_msgs::msg::Point DecisionManager::getInitPreAttackTarget() const {
    return constrainTargetPoint(blackboard_->getInitPreAttackPoint(), false);
}

geometry_msgs::msg::Point DecisionManager::getMainDecisionTarget() const {
    return constrainTargetPoint(blackboard_->getMainDecisionPoint(), false);
}

geometry_msgs::msg::Point DecisionManager::getSimpleDecisionTarget() const {
    return constrainTargetPoint(blackboard_->getSimpleDecisionPoint(), false);
}

void DecisionManager::transitionTo(State new_state) {
    if (current_state_ == new_state) return;
    current_state_ = new_state;
    last_state_entry_time_ = blackboard_->current_time;

    blackboard_->resetAllPublishStates();

    switch (new_state) {
        case State::IDLE:
            correcting_target_offset_ = false;
            correction_resume_state_ = State::IDLE;
            correction_target_ = geometry_msgs::msg::Point();
            blackboard_->resetCurrentBehavior();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_MOVE);
            break;
        case State::INIT_PRE_MOVE:
            blackboard_->startBehavior(BehaviorType::INIT_PRE_MOVE, getInitPreAttackTarget(), 0.0);
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_MOVE);
            break;
        case State::INIT_MOVE:
            blackboard_->startBehavior(BehaviorType::INIT_MOVE, constrainTargetPoint(blackboard_->getAttackPoint(), false), 0.0);
            blackboard_->updateControlMsg(GIMBAL_OUTPOST, SPIN_OFF, POSTURE_MOVE);
            break;
        case State::INIT_ATTACK:
            blackboard_->current_behavior.type = BehaviorType::INIT_ATTACK;
            blackboard_->current_behavior.target = constrainTargetPoint(blackboard_->getAttackPoint(), false);
            blackboard_->setTargetReached(false);
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            if (init_attack_forced_start_time_ < 0.0) {
                init_attack_forced_start_time_ = blackboard_->current_time;
            }
            blackboard_->current_behavior.execution_start_time = init_attack_forced_start_time_;
            blackboard_->updateControlMsg(GIMBAL_OUTPOST, SPIN_ON, POSTURE_ATTACK);
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
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_MOVE);
            break;
        }
        case State::MOVE_TO_ENEMY_FORTRESS: {
            geometry_msgs::msg::Point target = blackboard_->getEnemyFortressPoint();
            target = constrainTargetPoint(target);
            blackboard_->startBehavior(BehaviorType::MOVE_TO_ENEMY_FORTRESS, target, 0.0);
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_MOVE);
            break;
        }
        case State::OCCUPY_ENEMY_FORTRESS:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_ATTACK);
            break;
        case State::MOVE_TO_SUPPLY:
            blackboard_->startBehavior(BehaviorType::MOVE_TO_SUPPLY, constrainTargetPoint(blackboard_->getSupplyPoint(), false), 0.0);
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_MOVE);
            break;
        case State::RESURRECTION_MOVE:
            blackboard_->startBehavior(BehaviorType::RESURRECTION_MOVE, constrainTargetPoint(blackboard_->getSupplyPoint(), false), 0.0);
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_MOVE);
            break;
        case State::MOVE_TO_BASE_DEFENSE:
            blackboard_->startBehavior(BehaviorType::MOVE_TO_BASE_DEFENSE, constrainTargetPoint(blackboard_->getBaseGainPoint(), false), 0.0);
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_MOVE);
            break;
        case State::MOVE_TO_GAIN_POINT: {
            auto best = getBestGainPoint();
            geometry_msgs::msg::Point target = constrainTargetPoint(best.position);
            blackboard_->startBehavior(BehaviorType::MOVE_TO_GAIN_POINT, target, 0.0);
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_MOVE);
            break;
        }
        case State::MOVE_TO_FORTRESS:
            blackboard_->startBehavior(BehaviorType::MOVE_TO_FORTRESS, constrainTargetPoint(blackboard_->getFortressOccupyPoint(), false), 0.0);
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_MOVE);
            break;
        case State::MOVE_TO_GUARD: {
            geometry_msgs::msg::Point target;
            target.x = GUARD_X; target.y = GUARD_Y;
            blackboard_->startBehavior(BehaviorType::MOVE_TO_GUARD, constrainTargetPoint(target, false), 0.0);
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_MOVE);
            break;
        }
        case State::ATTACK_HERO:
        case State::ATTACK_ROBOT:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_ATTACK);
            break;
        case State::SUPPLYING:
            blackboard_->current_behavior.type = BehaviorType::SUPPLY;
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_DEFENSE);
            break;
        case State::RESURRECTING:
            blackboard_->current_behavior.type = BehaviorType::RESURRECTING;
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_DEFENSE);
            break;
        case State::BASE_DEFENSE:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_DEFENSE);
            break;
        case State::OCCUPY_GAIN_POINT:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_DEFENSE);
            break;
        case State::OCCUPY_FORTRESS:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_ATTACK);
            break;
        case State::GUARD:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_DEFENSE);
            break;
        case State::MOVE_TO_PATROL: {
            geometry_msgs::msg::Point target = blackboard_->getPatrolPoint();
            target = constrainTargetPoint(target);
            blackboard_->startBehavior(BehaviorType::MOVE_TO_PATROL, target, 0.0);
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_MOVE);
            break;
        }
        case State::PATROL:
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_ATTACK);
            break;
        case State::MOVE_TO_MAIN_POINT:
            blackboard_->startBehavior(BehaviorType::MOVE_TO_MAIN_POINT, getMainDecisionTarget(), 0.0);
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_MOVE);
            break;
        case State::MAIN_POINT_ATTACK:
            blackboard_->current_behavior.type = BehaviorType::MAIN_POINT_ATTACK;
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_ATTACK);
            break;
        case State::MOVE_TO_SIMPLE_POINT:
            blackboard_->startBehavior(BehaviorType::MOVE_TO_SIMPLE_POINT, getSimpleDecisionTarget(), 0.0);
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_MOVE);
            break;
        case State::SIMPLE_POINT_ATTACK:
            blackboard_->current_behavior.type = BehaviorType::SIMPLE_POINT_ATTACK;
            blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
            blackboard_->startExecutionTime();
            blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_ATTACK);
            break;
        default:
            break;
    }
}

std::string DecisionManager::stateToString(State state) const {
    switch (state) {
        case State::IDLE: return "IDLE";
        case State::INIT_PRE_MOVE: return "INIT_PRE_MOVE";
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
        case State::MOVE_TO_GUARD: return "MOVE_TO_GUARD";
        case State::GUARD: return "GUARD";
        case State::MOVE_TO_ENEMY_FORTRESS: return "MOVE_TO_ENEMY_FORTRESS";
        case State::OCCUPY_ENEMY_FORTRESS: return "OCCUPY_ENEMY_FORTRESS";
        case State::MOVE_TO_PATROL: return "MOVE_TO_PATROL";
        case State::PATROL: return "PATROL";
        case State::MOVE_TO_MAIN_POINT: return "MOVE_TO_MAIN_POINT";
        case State::MAIN_POINT_ATTACK: return "MAIN_POINT_ATTACK";
        case State::MOVE_TO_SIMPLE_POINT: return "MOVE_TO_SIMPLE_POINT";
        case State::SIMPLE_POINT_ATTACK: return "SIMPLE_POINT_ATTACK";
        default: return "UNKNOWN";
    }
}

DecisionOutput DecisionManager::executeDecision() {
    DecisionOutput output;
    output.target_needs_publishing = false;
    output.control_needs_publishing = false;

    updateHeroDeployFlag();

    if (blackboard_->stage != STAGE_BATTLE) {
        init_attack_forced_start_time_ = -1.0;
        if (current_state_ != State::IDLE) transitionTo(State::IDLE);
        else {
            uint8_t gimbal = (blackboard_->stage_remaining_time <= 5.0) ? GIMBAL_ENEMY : GIMBAL_IDLE;
            blackboard_->updateControlMsg(gimbal, SPIN_OFF, POSTURE_MOVE);
        }
        output.control_msg = *blackboard_->getControlMsg();
        output.decision_reason = "NOT_IN_BATTLE";
        if (blackboard_->isControlUpdated()) {
            output.control_needs_publishing = true;
            blackboard_->setControlUpdated(false);
        }
        output.control_msg.avoidengineer_flag = 0;
        return output;
    }

    switch (current_state_) {
        case State::IDLE: {
            if (isInitAttackForcedTimerActive()) {
                transitionTo(State::INIT_ATTACK);
            } else if (blackboard_->resurrection_flag) {
                transitionTo(State::RESURRECTION_MOVE);
            } else if (needSupply()) {
                transitionTo(State::MOVE_TO_SUPPLY);
            } else if (blackboard_->getDecisionMode() == 1) {
                transitionTo(State::MOVE_TO_SIMPLE_POINT);
            } else if (!blackboard_->initialization_complete) {
                transitionTo(State::INIT_PRE_MOVE);
            } else {
                transitionTo(State::MOVE_TO_MAIN_POINT);
            }
            break;
        }

        case State::MOVE_TO_SIMPLE_POINT: {
            if (shouldInterruptForResurrectionOrSupply()) {
                transitionTo(State::IDLE);
                break;
            }
            geometry_msgs::msg::Point target = correcting_target_offset_ ? correction_target_ : getSimpleDecisionTarget();
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                    transitionTo(consumeCorrectionResumeState(State::SIMPLE_POINT_ATTACK));
                }
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                blackboard_->current_behavior.target = target;
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }

        case State::SIMPLE_POINT_ATTACK: {
            if (shouldInterruptForResurrectionOrSupply()) {
                transitionTo(State::IDLE);
                break;
            }
            if (beginTargetOffsetCorrection(State::MOVE_TO_SIMPLE_POINT, output)) break;
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_ATTACK);
                output.control_needs_publishing = true;
            }
            break;
        }

        case State::MOVE_TO_MAIN_POINT: {
            if (shouldInterruptForResurrectionOrSupply()) {
                transitionTo(State::IDLE);
                break;
            }
            geometry_msgs::msg::Point target = correcting_target_offset_ ? correction_target_ : getMainDecisionTarget();
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                    transitionTo(consumeCorrectionResumeState(State::MAIN_POINT_ATTACK));
                }
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                blackboard_->current_behavior.target = target;
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }

        case State::MAIN_POINT_ATTACK: {
            if (shouldInterruptForResurrectionOrSupply()) {
                transitionTo(State::IDLE);
                break;
            }
            if (beginTargetOffsetCorrection(State::MOVE_TO_MAIN_POINT, output)) break;
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_ATTACK);
                output.control_needs_publishing = true;
            }
            break;
        }

        case State::MOVE_TO_PATROL: {
            if (shouldInterruptForResurrectionOrSupply()) {
                transitionTo(State::IDLE);
                break;
            }
            geometry_msgs::msg::Point target = correcting_target_offset_ ? correction_target_ : blackboard_->getPatrolPoint();
            if (!correcting_target_offset_)
                target = constrainTargetPoint(target);
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                    transitionTo(consumeCorrectionResumeState(State::PATROL));
                }
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                blackboard_->current_behavior.target = target;
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::PATROL: {
            if (shouldInterruptForResurrectionOrSupply()) {
                transitionTo(State::IDLE);
                break;
            }
            if (beginTargetOffsetCorrection(State::MOVE_TO_PATROL, output)) break;
            double elapsed = blackboard_->getExecutionElapsedTime();
            if (elapsed >= blackboard_->getPatrolStayDuration()) {
                transitionTo(State::IDLE);
                break;
            }
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_ATTACK);
                output.control_needs_publishing = true;
            }
            break;
        }

        case State::INIT_PRE_MOVE: {
            if (shouldInterruptForResurrectionOrSupply()) { transitionTo(State::IDLE); break; }
            geometry_msgs::msg::Point target = correcting_target_offset_ ? correction_target_ : getInitPreAttackTarget();
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                    transitionTo(consumeCorrectionResumeState(State::INIT_MOVE));
                    if (current_state_ == State::INIT_MOVE) {
                        output.target_position = constrainTargetPoint(blackboard_->getAttackPoint(), false);
                        output.target_needs_publishing = true;
                    }
                }
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                blackboard_->current_behavior.target = target;
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }

        case State::INIT_MOVE: {
            if (!isInitAttackForcedTimerActive() && shouldInterruptForResurrectionOrSupply()) { transitionTo(State::IDLE); break; }
            geometry_msgs::msg::Point target = correcting_target_offset_ ? correction_target_ : constrainTargetPoint(blackboard_->getAttackPoint(), false);
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                    transitionTo(consumeCorrectionResumeState(State::INIT_ATTACK));
                }
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                blackboard_->current_behavior.target = target;
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::INIT_ATTACK: {
            if (blackboard_->current_behavior.type != BehaviorType::INIT_ATTACK) {
                blackboard_->current_behavior.type = BehaviorType::INIT_ATTACK;
                blackboard_->current_behavior.target = constrainTargetPoint(blackboard_->getAttackPoint(), false);
                blackboard_->setTargetReached(false);
                blackboard_->updateBehaviorState(BehaviorState::EXECUTING);
                blackboard_->current_behavior.execution_start_time = init_attack_forced_start_time_;
            }
            const auto& init_target = blackboard_->current_behavior.target;
            if (!((init_target.x == 0.0 && init_target.y == 0.0) ||
                  blackboard_->isAtTarget(init_target, blackboard_->getDeviationThreshold()))) {
                if (beginTargetOffsetCorrection(State::INIT_MOVE, output)) break;
            }
            double elapsed = blackboard_->current_time - init_attack_forced_start_time_;
            blackboard_->init_attack_elapsed_time = std::min(blackboard_->getInitAttackDuration(),
                                                             std::max(0.0, elapsed));
            if (elapsed >= blackboard_->getInitAttackDuration()) {
                blackboard_->init_attack_elapsed_time = blackboard_->getInitAttackDuration();
                blackboard_->initialization_complete = true;
                init_attack_forced_start_time_ = -1.0;
                transitionTo(State::IDLE);
                break;
            }
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_OUTPOST, SPIN_ON, POSTURE_ATTACK);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::MOVE_TO_ENEMY_FORTRESS: {
            if (shouldInterruptForResurrectionOrSupply()) { transitionTo(State::IDLE); break; }
            geometry_msgs::msg::Point target = correcting_target_offset_ ? correction_target_ : blackboard_->getEnemyFortressPoint();
            if (!correcting_target_offset_)
                target = constrainTargetPoint(target);
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                    transitionTo(consumeCorrectionResumeState(State::OCCUPY_ENEMY_FORTRESS));
                }
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                blackboard_->current_behavior.target = target;
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::OCCUPY_ENEMY_FORTRESS: {
            if (shouldInterruptForResurrectionOrSupply()) { transitionTo(State::IDLE); break; }
            if (beginTargetOffsetCorrection(State::MOVE_TO_ENEMY_FORTRESS, output)) break;
            double elapsed = blackboard_->getExecutionElapsedTime();
            if (elapsed >= blackboard_->getDefendDuration())
                transitionTo(State::IDLE);
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_ATTACK);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::MOVE_TO_ATTACK_HERO:
        case State::MOVE_TO_ATTACK_ROBOT: {
            if (shouldInterruptForResurrectionOrSupply()) { transitionTo(State::IDLE); break; }
            geometry_msgs::msg::Point target = correcting_target_offset_ ? correction_target_ : getTargetPointForEnemy(current_enemy_id_);
            if (target.x == 0 && target.y == 0) {
                transitionTo(State::IDLE);
                break;
            }
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                    State normal_state = (current_state_ == State::MOVE_TO_ATTACK_HERO) ?
                                         State::ATTACK_HERO : State::ATTACK_ROBOT;
                    transitionTo(consumeCorrectionResumeState(normal_state));
                }
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                blackboard_->current_behavior.target = target;
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::ATTACK_HERO:
        case State::ATTACK_ROBOT: {
            if (shouldInterruptForResurrectionOrSupply()) { transitionTo(State::IDLE); break; }
            geometry_msgs::msg::Point target = getTargetPointForEnemy(current_enemy_id_);
            if (target.x == 0 || target.y == 0) {
                transitionTo(State::IDLE);
                break;
            }
            if (!blackboard_->isAtTarget(target, blackboard_->getEnemyChaseRepathThreshold())) {
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
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_ATTACK);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::MOVE_TO_SUPPLY:
        case State::RESURRECTION_MOVE: {
            geometry_msgs::msg::Point target = correcting_target_offset_ ? correction_target_ : constrainTargetPoint(blackboard_->getSupplyPoint(), false);
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                    State normal_state = (current_state_ == State::RESURRECTION_MOVE) ?
                                         State::RESURRECTING : State::SUPPLYING;
                    transitionTo(consumeCorrectionResumeState(normal_state));
                }
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                blackboard_->current_behavior.target = target;
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::SUPPLYING: {
            if (beginTargetOffsetCorrection(State::MOVE_TO_SUPPLY, output)) break;
            if (blackboard_->current_hp >= blackboard_->getMaxHp())
                transitionTo(State::IDLE);
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_DEFENSE);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::RESURRECTING: {
            if (beginTargetOffsetCorrection(State::RESURRECTION_MOVE, output)) break;
            if (!blackboard_->resurrection_flag && blackboard_->current_hp >= blackboard_->getMaxHp())
                transitionTo(State::IDLE);
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_DEFENSE);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::MOVE_TO_BASE_DEFENSE: {
            if (shouldInterruptForResurrectionOrSupply()) { transitionTo(State::IDLE); break; }
            geometry_msgs::msg::Point target = correcting_target_offset_ ? correction_target_ : constrainTargetPoint(blackboard_->getBaseGainPoint(), false);
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                    transitionTo(consumeCorrectionResumeState(State::BASE_DEFENSE));
                }
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                blackboard_->current_behavior.target = target;
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::BASE_DEFENSE: {
            if (shouldInterruptForResurrectionOrSupply()) { transitionTo(State::IDLE); break; }
            if (beginTargetOffsetCorrection(State::MOVE_TO_BASE_DEFENSE, output)) break;
            double elapsed = blackboard_->getExecutionElapsedTime();
            if (elapsed >= blackboard_->getDefendDuration())
                transitionTo(State::IDLE);
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_DEFENSE);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::MOVE_TO_GAIN_POINT: {
            if (shouldInterruptForResurrectionOrSupply()) { transitionTo(State::IDLE); break; }
            geometry_msgs::msg::Point target;
            if (correcting_target_offset_) {
                target = correction_target_;
            } else {
                auto best = getBestGainPoint();
                if (best.name.empty()) {
                    transitionTo(State::IDLE);
                    break;
                }
                target = constrainTargetPoint(best.position);
            }
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                    transitionTo(consumeCorrectionResumeState(State::OCCUPY_GAIN_POINT));
                }
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                blackboard_->current_behavior.target = target;
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::OCCUPY_GAIN_POINT: {
            if (shouldInterruptForResurrectionOrSupply()) { transitionTo(State::IDLE); break; }
            if (beginTargetOffsetCorrection(State::MOVE_TO_GAIN_POINT, output)) break;
            double elapsed = blackboard_->getExecutionElapsedTime();
            if (elapsed >= blackboard_->getDefendDuration())
                transitionTo(State::IDLE);
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_DEFENSE);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::MOVE_TO_FORTRESS: {
            if (shouldInterruptForResurrectionOrSupply()) { transitionTo(State::IDLE); break; }
            geometry_msgs::msg::Point target = correcting_target_offset_ ? correction_target_ : constrainTargetPoint(blackboard_->getFortressOccupyPoint(), false);
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                    transitionTo(consumeCorrectionResumeState(State::OCCUPY_FORTRESS));
                }
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                blackboard_->current_behavior.target = target;
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::OCCUPY_FORTRESS: {
            if (shouldInterruptForResurrectionOrSupply()) { transitionTo(State::IDLE); break; }
            if (beginTargetOffsetCorrection(State::MOVE_TO_FORTRESS, output)) break;
            double elapsed = blackboard_->getExecutionElapsedTime();
            if (elapsed >= blackboard_->getDefendDuration())
                transitionTo(State::IDLE);
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_ATTACK);
                output.control_needs_publishing = true;
            }
            break;
        }
        case State::MOVE_TO_GUARD: {
            if (shouldInterruptForResurrectionOrSupply()) { transitionTo(State::IDLE); break; }
            geometry_msgs::msg::Point target;
            if (correcting_target_offset_) {
                target = correction_target_;
            } else {
                target.x = GUARD_X; target.y = GUARD_Y;
                target = constrainTargetPoint(target, false);
            }
            if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
                if (!blackboard_->at_current_target) {
                    blackboard_->setTargetReached(true);
                    transitionTo(consumeCorrectionResumeState(State::GUARD));
                }
            } else {
                if (blackboard_->at_current_target) blackboard_->setTargetReached(false);
                blackboard_->current_behavior.target = target;
                output.target_position = target;
                output.target_needs_publishing = true;
            }
            break;
        }
        case State::GUARD: {
            if (shouldInterruptForResurrectionOrSupply()) { transitionTo(State::IDLE); break; }
            if (beginTargetOffsetCorrection(State::MOVE_TO_GUARD, output)) break;
            double elapsed = blackboard_->getExecutionElapsedTime();
            if (elapsed >= 5.0) {
                transitionTo(State::IDLE);
                break;
            }
            if (!blackboard_->isControlPublished()) {
                blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_ON, POSTURE_DEFENSE);
                output.control_needs_publishing = true;
            }
            break;
        }
    }

    if (blackboard_->enemy_engineer.visible && blackboard_->enemy_engineer.hp > 0) {
        bool in_zone = region_manager_->isInEnemyEngineerMiningZone(
            blackboard_->enemy_engineer.x, blackboard_->enemy_engineer.y, blackboard_->robot_id_);
        blackboard_->getControlMsg()->avoidengineer_flag = in_zone ? 1 : 0;
    } else {
        blackboard_->getControlMsg()->avoidengineer_flag = 0;
    }

    output.control_msg = *blackboard_->getControlMsg();
    if (blackboard_->isControlUpdated()) {
        output.control_needs_publishing = true;
        blackboard_->setControlUpdated(false);
    }
    output.decision_reason = stateToString(current_state_);
    return output;
}
