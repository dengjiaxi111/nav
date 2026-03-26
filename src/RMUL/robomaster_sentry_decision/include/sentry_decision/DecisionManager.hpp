#ifndef DECISION_MANAGER_HPP
#define DECISION_MANAGER_HPP

#include "Blackboard.hpp"
#include "Models.hpp"
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
    MOVING_TO_ATTACK,
    MOVING_TO_SUPPLY,
    ATTACKING,
    SUPPLYING,
    RESURRECTING
};

class DecisionManager {
public:
    DecisionManager();

    void updateOurState(const OurRobotState::SharedPtr msg);
    void updateGameState(const GameState::SharedPtr msg);

    DecisionOutput executeDecision();

    std::shared_ptr<Blackboard> getBlackboard() const { return blackboard_; }

private:
    std::shared_ptr<Blackboard> blackboard_;
    State current_state_ = State::IDLE;
    double last_state_entry_time_ = 0.0;

    bool needSupply() const;
    bool shouldInterruptAttack() const;
    bool supplyComplete() const;
    bool resurrectionComplete() const;
    void transitionTo(State new_state);
    std::string stateToString(State state) const;
};

#endif // DECISION_MANAGER_HPP
