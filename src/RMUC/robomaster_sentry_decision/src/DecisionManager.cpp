// src/DecisionManager.cpp
#include "sentry_decision/DecisionManager.hpp"
#include "sentry_decision/GameConstants.hpp"
#include <cmath>
#include <algorithm>

using namespace GameConstants;

DecisionManager::DecisionManager()
    : blackboard_(std::make_shared<Blackboard>()),
      region_manager_(std::make_shared<RegionManager>()) {}

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
    return (hp_ratio < blackboard_->getSupplyThreshold() || ammo_ratio < blackboard_->getSupplyThreshold());
}

bool DecisionManager::isAnyEnemyVisible() const {
    return (blackboard_->enemy_hero.visible ||
            blackboard_->enemy_engineer.visible ||
            blackboard_->enemy_infantry3.visible ||
            blackboard_->enemy_infantry4.visible ||
            blackboard_->enemy_sentry.visible);
}

std::string DecisionManager::selectNearestEnemy() const {
    double min_dist = 1e9;
    std::string best;
    auto check = [&](const EnemyInfo& e) {
        if (e.visible) {
            double d = std::hypot(blackboard_->x - e.x, blackboard_->y - e.y);
            if (d < min_dist) { min_dist = d; best = e.id; }
        }
    };
    check(blackboard_->enemy_hero);
    check(blackboard_->enemy_engineer);
    check(blackboard_->enemy_infantry3);
    check(blackboard_->enemy_infantry4);
    check(blackboard_->enemy_sentry);
    return best;
}

geometry_msgs::msg::Point DecisionManager::getTargetPointForEnemy(const std::string& enemy_id) const {
    const EnemyInfo* enemy = blackboard_->getEnemyById(enemy_id);
    if (!enemy || !enemy->visible) return geometry_msgs::msg::Point();
    geometry_msgs::msg::Point hex = region_manager_->findSameTeamHexPoint(
        enemy->x, enemy->y, blackboard_->x, blackboard_->y, blackboard_->robot_id_);
    return region_manager_->clampToTeamRegion(hex, blackboard_->robot_id_,
                                              blackboard_->x, blackboard_->y);
}

bool DecisionManager::shouldRunInitialOutpostTask() const {
    return !initial_outpost_done_ &&
           blackboard_->outpost_alive != 0 &&
           blackboard_->stage_remaining_time > blackboard_->getInitialOutpostEndRemainingTime();
}

bool DecisionManager::isInitialOutpostTaskFinished() const {
    return blackboard_->outpost_alive == 0 ||
           blackboard_->stage_remaining_time <= blackboard_->getInitialOutpostEndRemainingTime();
}

void DecisionManager::transitionToInitialOutpost() {
    transitionTo(State::MOVE_TO_INITIAL_OUTPOST);
}

void DecisionManager::transitionToPrimaryTask() {
    if (shouldRunInitialOutpostTask()) {
        transitionToInitialOutpost();
        return;
    }

    if (isInitialOutpostTaskFinished()) {
        initial_outpost_done_ = true;
    }
    transitionToScheduledPatrol();
}

void DecisionManager::handleMoveToInitialOutpost(DecisionOutput& output) {
    if (checkInterrupts(State::MOVE_TO_INITIAL_OUTPOST)) return;

    if (isInitialOutpostTaskFinished()) {
        initial_outpost_done_ = true;
        transitionToPrimaryTask();
        return;
    }

    geometry_msgs::msg::Point target = blackboard_->getInitialOutpostPoint();
    if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
        if (!blackboard_->at_current_target) {
            blackboard_->setTargetReached(true);
            transitionTo(State::INITIAL_OUTPOST_HOLD);
        }
    } else {
        blackboard_->setTargetReached(false);
        output.target_position = target;
        output.target_needs_publishing = true;
    }
}

void DecisionManager::handleInitialOutpostHold(DecisionOutput& output) {
    if (checkInterrupts(State::INITIAL_OUTPOST_HOLD)) return;

    if (isInitialOutpostTaskFinished()) {
        initial_outpost_done_ = true;
        transitionToPrimaryTask();
        return;
    }

    output.control_msg = *blackboard_->getControlMsg();
    output.control_needs_publishing = true;
}

void DecisionManager::transitionToScheduledPatrol() {
    constexpr double kPatrolBStartRemainingTime = 4.0 * 60.0 + 15.0;

    if (scheduled_patrol_phase_ == 0 &&
        blackboard_->stage_remaining_time <= kPatrolBStartRemainingTime) {
        scheduled_patrol_phase_ = 1;
    }
    if (scheduled_patrol_phase_ == 1 &&
        patrol_b_reached_ &&
        blackboard_->outpost_alive == 0) {
        scheduled_patrol_phase_ = 2;
    }

    if (scheduled_patrol_phase_ == 0) {
        transitionTo(State::MOVE_TO_PATROL_A);
    } else if (scheduled_patrol_phase_ == 1) {
        transitionTo(State::MOVE_TO_PATROL_B);
    } else {
        transitionTo(State::MOVE_TO_PATROL_C);
    }
}

// 统一中断检查：根据当前状态允许被打断的任务
// 返回 true 表示已发生状态转换
bool DecisionManager::checkInterrupts(State current) {
    // 补给优先级最高；复活次之。补给状态本身不再被中断。
    if (current != State::MOVE_TO_SUPPLY && current != State::SUPPLYING) {
        if (needSupply()) {
            transitionTo(State::MOVE_TO_SUPPLY);
            return true;
        }
        if (current != State::RESURRECTION_MOVE && current != State::RESURRECTING &&
            blackboard_->resurrection_flag) {
            transitionTo(State::RESURRECTION_MOVE);
            return true;
        }
    }
    return false;
}

void DecisionManager::transitionTo(State new_state) {
    if (current_state_ == new_state) return;
    current_state_ = new_state;
    last_state_entry_time_ = blackboard_->current_time;
    blackboard_->resetAllPublishStates();

    auto setBehaviorTarget = [&](const geometry_msgs::msg::Point& target) {
        blackboard_->current_behavior.target = target;
        blackboard_->current_behavior.start_time = blackboard_->current_time;
        blackboard_->setTargetReached(false);
    };

    switch (new_state) {
    case State::IDLE:
        blackboard_->updateControlMsg(GIMBAL_ENEMY, SPIN_OFF, POSTURE_MOVE);
        break;
    case State::MOVE_TO_PATROL_A:
        setBehaviorTarget(blackboard_->getPatrolPointA());
        blackboard_->updateControlMsg(GIMBAL_ENEMY, blackboard_->getBattleSpinMode(), POSTURE_MOVE);
        break;
    case State::MOVE_TO_PATROL_B:
        setBehaviorTarget(blackboard_->getPatrolPointB());
        blackboard_->updateControlMsg(GIMBAL_ENEMY, blackboard_->getBattleSpinMode(), POSTURE_MOVE);
        break;
    case State::MOVE_TO_PATROL_C:
        setBehaviorTarget(blackboard_->getPatrolPointC());
        blackboard_->updateControlMsg(GIMBAL_ENEMY, blackboard_->getBattleSpinMode(), POSTURE_MOVE);
        break;
    case State::PATROL_A:
    case State::PATROL_B:
    case State::PATROL_C:
        if (new_state == State::PATROL_B) {
            patrol_b_reached_ = true;
        } else if (new_state == State::PATROL_C) {
            scheduled_patrol_phase_ = 2;
        }
        blackboard_->updateControlMsg(GIMBAL_ENEMY, blackboard_->getBattleSpinMode(), POSTURE_ATTACK);
        break;
    case State::MOVE_TO_INITIAL_OUTPOST:
        setBehaviorTarget(blackboard_->getInitialOutpostPoint());
        blackboard_->updateControlMsg(GIMBAL_OUTPOST, blackboard_->getBattleSpinMode(), POSTURE_MOVE);
        break;
    case State::INITIAL_OUTPOST_HOLD:
        blackboard_->updateControlMsg(GIMBAL_OUTPOST, blackboard_->getBattleSpinMode(), POSTURE_ATTACK);
        break;
    case State::MOVE_TO_ATTACK: {
        geometry_msgs::msg::Point target = getTargetPointForEnemy(current_enemy_id_);
        if (target.x == 0 && target.y == 0) {
            transitionTo(State::IDLE);
            return;
        }
        setBehaviorTarget(target);
        blackboard_->updateControlMsg(GIMBAL_ENEMY, blackboard_->getBattleSpinMode(), POSTURE_MOVE);
        break;
    }
    case State::ATTACK:
        blackboard_->updateControlMsg(GIMBAL_ENEMY, blackboard_->getBattleSpinMode(), POSTURE_ATTACK);
        break;
    case State::MOVE_TO_SUPPLY:
        setBehaviorTarget(blackboard_->getSupplyPoint());
        blackboard_->updateControlMsg(GIMBAL_ENEMY, blackboard_->getBattleSpinMode(), POSTURE_MOVE);
        break;
    case State::SUPPLYING:
        blackboard_->updateControlMsg(GIMBAL_ENEMY, blackboard_->getBattleSpinMode(), POSTURE_DEFENSE);
        break;
    case State::RESURRECTION_MOVE:
        setBehaviorTarget(blackboard_->getSupplyPoint());
        blackboard_->updateControlMsg(GIMBAL_ENEMY, blackboard_->getBattleSpinMode(), POSTURE_MOVE);
        break;
    case State::RESURRECTING:
        blackboard_->updateControlMsg(GIMBAL_ENEMY, blackboard_->getBattleSpinMode(), POSTURE_DEFENSE);
        break;
    }
}

std::string DecisionManager::stateToString(State state) const {
    switch (state) {
    case State::IDLE: return "IDLE";
    case State::PATROL_A: return "PATROL_A";
    case State::PATROL_B: return "PATROL_B";
    case State::PATROL_C: return "PATROL_C";
    case State::MOVE_TO_PATROL_A: return "MOVE_TO_PATROL_A";
    case State::MOVE_TO_PATROL_B: return "MOVE_TO_PATROL_B";
    case State::MOVE_TO_PATROL_C: return "MOVE_TO_PATROL_C";
    case State::MOVE_TO_ATTACK: return "MOVE_TO_ATTACK";
    case State::ATTACK: return "ATTACK";
    case State::MOVE_TO_SUPPLY: return "MOVE_TO_SUPPLY";
    case State::SUPPLYING: return "SUPPLYING";
    case State::MOVE_TO_INITIAL_OUTPOST: return "MOVE_TO_INITIAL_OUTPOST";
    case State::INITIAL_OUTPOST_HOLD: return "INITIAL_OUTPOST_HOLD";
    case State::RESURRECTION_MOVE: return "RESURRECTION_MOVE";
    case State::RESURRECTING: return "RESURRECTING";
    default: return "UNKNOWN";
    }
}

DecisionOutput DecisionManager::executeDecision() {
    DecisionOutput output;
    output.target_needs_publishing = false;
    output.control_needs_publishing = false;

    // 非战斗阶段
    if (blackboard_->stage != STAGE_BATTLE) {
        scheduled_patrol_phase_ = 0;
        patrol_b_reached_ = false;
        initial_outpost_done_ = false;
        if (current_state_ != State::IDLE) transitionTo(State::IDLE);
        blackboard_->updateControlMsg(GIMBAL_IDLE, SPIN_OFF, POSTURE_MOVE);
        output.control_msg = *blackboard_->getControlMsg();
        output.control_needs_publishing = true;
        output.decision_reason = "NOT_IN_BATTLE";
        return output;
    }

    // ---- 在所有状态中首先检查最高优先级：补给优先，复活次之 ----
    if (current_state_ != State::MOVE_TO_SUPPLY &&
        current_state_ != State::SUPPLYING) {
        if (needSupply()) {
            transitionTo(State::MOVE_TO_SUPPLY);
        } else if (current_state_ != State::RESURRECTION_MOVE &&
                   current_state_ != State::RESURRECTING &&
                   blackboard_->resurrection_flag) {
            transitionTo(State::RESURRECTION_MOVE);
        }
    }

    // 状态机执行
    switch (current_state_) {
    case State::IDLE: {
        // 再次确认补给/复活（已在上面检查，但IDLE需保证）
        if (needSupply()) {
            transitionTo(State::MOVE_TO_SUPPLY);
            break;
        }
        if (blackboard_->resurrection_flag) {
            transitionTo(State::RESURRECTION_MOVE);
            break;
        }
        transitionToPrimaryTask();
        break;
    }

    // ---------- 巡逻 A ----------
    case State::MOVE_TO_PATROL_A: {
        if (checkInterrupts(State::MOVE_TO_PATROL_A)) break;
        if (blackboard_->stage_remaining_time <= 4.0 * 60.0 + 15.0) {
            scheduled_patrol_phase_ = 1;
            transitionTo(State::MOVE_TO_PATROL_B);
            break;
        }
        geometry_msgs::msg::Point target = blackboard_->getPatrolPointA();
        if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
            if (!blackboard_->at_current_target) {
                blackboard_->setTargetReached(true);
                transitionTo(State::PATROL_A);
            }
        } else {
            blackboard_->setTargetReached(false);
            output.target_position = target;
            output.target_needs_publishing = true;
        }
        break;
    }
    case State::PATROL_A: {
        if (checkInterrupts(State::PATROL_A)) break;
        if (blackboard_->stage_remaining_time <= 4.0 * 60.0 + 15.0) {
            scheduled_patrol_phase_ = 1;
            transitionTo(State::MOVE_TO_PATROL_B);
        } else {
            output.control_msg = *blackboard_->getControlMsg();
            output.control_needs_publishing = true;
        }
        break;
    }

    // ---------- 巡逻 B ----------
    case State::MOVE_TO_PATROL_B: {
        if (checkInterrupts(State::MOVE_TO_PATROL_B)) break;
        geometry_msgs::msg::Point target = blackboard_->getPatrolPointB();
        if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
            if (!blackboard_->at_current_target) {
                blackboard_->setTargetReached(true);
                transitionTo(State::PATROL_B);
            }
        } else {
            blackboard_->setTargetReached(false);
            output.target_position = target;
            output.target_needs_publishing = true;
        }
        break;
    }
    case State::PATROL_B: {
        if (checkInterrupts(State::PATROL_B)) break;
        if (blackboard_->outpost_alive == 0) {
            scheduled_patrol_phase_ = 2;
            transitionTo(State::MOVE_TO_PATROL_C);
        } else {
            output.control_msg = *blackboard_->getControlMsg();
            output.control_needs_publishing = true;
        }
        break;
    }

    // ---------- 巡逻 C ----------
    case State::MOVE_TO_PATROL_C: {
        if (checkInterrupts(State::MOVE_TO_PATROL_C)) break;
        geometry_msgs::msg::Point target = blackboard_->getPatrolPointC();
        if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
            if (!blackboard_->at_current_target) {
                blackboard_->setTargetReached(true);
                transitionTo(State::PATROL_C);
            }
        } else {
            blackboard_->setTargetReached(false);
            output.target_position = target;
            output.target_needs_publishing = true;
        }
        break;
    }
    case State::PATROL_C: {
        if (checkInterrupts(State::PATROL_C)) break;
        output.control_msg = *blackboard_->getControlMsg();
        output.control_needs_publishing = true;
        break;
    }

    // ---------- 开局前哨 ----------
    case State::MOVE_TO_INITIAL_OUTPOST: {
        handleMoveToInitialOutpost(output);
        break;
    }
    case State::INITIAL_OUTPOST_HOLD: {
        handleInitialOutpostHold(output);
        break;
    }

    // ---------- 打人 ----------
    case State::MOVE_TO_ATTACK: {
        if (checkInterrupts(State::MOVE_TO_ATTACK)) break;
        geometry_msgs::msg::Point target = getTargetPointForEnemy(current_enemy_id_);
        const EnemyInfo* enemy = blackboard_->getEnemyById(current_enemy_id_);
        if (!enemy || !enemy->visible || (target.x == 0 && target.y == 0)) {
            transitionTo(State::IDLE);
            break;
        }
        if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
            if (!blackboard_->at_current_target) {
                blackboard_->setTargetReached(true);
                transitionTo(State::ATTACK);
            }
        } else {
            blackboard_->setTargetReached(false);
            output.target_position = target;
            output.target_needs_publishing = true;
        }
        break;
    }
    case State::ATTACK: {
        if (checkInterrupts(State::ATTACK)) break;
        const EnemyInfo* enemy = blackboard_->getEnemyById(current_enemy_id_);
        if (!enemy || !enemy->visible) {
            transitionTo(State::IDLE);
            break;
        }
        double elapsed = blackboard_->current_time - last_state_entry_time_;
        if (elapsed >= blackboard_->getAttackDuration()) {
            transitionTo(State::IDLE);
            break;
        }
        geometry_msgs::msg::Point target = getTargetPointForEnemy(current_enemy_id_);
        if (!blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
            transitionTo(State::MOVE_TO_ATTACK);
            break;
        }
        output.control_msg = *blackboard_->getControlMsg();
        output.control_needs_publishing = true;
        break;
    }

    // ---------- 补给/复活 ----------
    case State::MOVE_TO_SUPPLY:
    case State::RESURRECTION_MOVE: {
        geometry_msgs::msg::Point target = blackboard_->getSupplyPoint();
        if (blackboard_->isAtTarget(target, blackboard_->getDeviationThreshold())) {
            if (!blackboard_->at_current_target) {
                blackboard_->setTargetReached(true);
                if (current_state_ == State::RESURRECTION_MOVE)
                    transitionTo(State::RESURRECTING);
                else
                    transitionTo(State::SUPPLYING);
            }
        } else {
            blackboard_->setTargetReached(false);
            output.target_position = target;
            output.target_needs_publishing = true;
        }
        break;
    }
    case State::SUPPLYING: {
        if (blackboard_->current_hp >= blackboard_->getMaxHp() &&
            blackboard_->allowance_17mm >= blackboard_->getMaxAmmo())
            transitionToPrimaryTask();
        output.control_msg = *blackboard_->getControlMsg();
        output.control_needs_publishing = true;
        break;
    }
    case State::RESURRECTING: {
        if (!blackboard_->resurrection_flag && blackboard_->current_hp >= blackboard_->getMaxHp())
            transitionToPrimaryTask();
        output.control_msg = *blackboard_->getControlMsg();
        output.control_needs_publishing = true;
        break;
    }

    }

    // 避让工程标志位
    if (blackboard_->enemy_engineer.visible && blackboard_->enemy_engineer.hp > 0) {
        blackboard_->getControlMsg()->avoidengineer_flag = 0;
    } else {
        blackboard_->getControlMsg()->avoidengineer_flag = 0;
    }

    blackboard_->getControlMsg()->spin_mode = blackboard_->getBattleSpinMode();
    output.control_msg = *blackboard_->getControlMsg();
    output.control_needs_publishing = true;
    output.decision_reason = stateToString(current_state_);
    return output;
}
