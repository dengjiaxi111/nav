#ifndef DECISION_MANAGER_HPP
#define DECISION_MANAGER_HPP

#include "Blackboard.hpp"
#include "RegionManager.hpp"
#include <memory>
#include <string>

struct DecisionOutput {
    geometry_msgs::msg::Point target_position;
    SentryControl control_msg;
    bool target_needs_publishing = false;
    bool control_needs_publishing = false;
    std::string decision_reason;
};

enum class State {
    IDLE,
    PATROL_A,
    PATROL_B,
    PATROL_C,
    MOVE_TO_PATROL_A,
    MOVE_TO_PATROL_B,
    MOVE_TO_PATROL_C,
    MOVE_TO_ATTACK,
    ATTACK,
    MOVE_TO_SUPPLY,
    SUPPLYING,
    MOVE_TO_INITIAL_OUTPOST,
    INITIAL_OUTPOST_HOLD,
    RESURRECTION_MOVE,
    RESURRECTING
};

class DecisionManager {
public:
    DecisionManager();
    void updateOurState(const OurRobotState::SharedPtr msg);
    void updateEnemyState(const EnemyRobotState::SharedPtr msg);
    void updateGameState(const GameState::SharedPtr msg);
    DecisionOutput executeDecision();
    std::shared_ptr<Blackboard> getBlackboard() const { return blackboard_; }

private:
    std::shared_ptr<Blackboard> blackboard_;
    std::shared_ptr<RegionManager> region_manager_;
    State current_state_ = State::IDLE;
    double last_state_entry_time_ = 0.0;
    std::string current_enemy_id_;
    int scheduled_patrol_phase_ = 0;
    bool patrol_b_reached_ = false;
    bool initial_outpost_done_ = false;

    bool checkInterrupts(State current);
    bool needSupply() const;
    bool isAnyEnemyVisible() const;
    std::string selectNearestEnemy() const;
    geometry_msgs::msg::Point getTargetPointForEnemy(const std::string& enemy_id) const;
    bool shouldRunInitialOutpostTask() const;
    bool isInitialOutpostTaskFinished() const;
    void transitionToInitialOutpost();
    void handleMoveToInitialOutpost(DecisionOutput& output);
    void handleInitialOutpostHold(DecisionOutput& output);
    void transitionToPrimaryTask();
    void transitionToScheduledPatrol();

    void transitionTo(State new_state);
    std::string stateToString(State state) const;
};

#endif
