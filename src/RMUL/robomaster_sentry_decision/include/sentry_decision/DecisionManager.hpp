#ifndef DECISION_MANAGER_HPP
#define DECISION_MANAGER_HPP

#include "Blackboard.hpp"
#include "Models.hpp"
#include <memory>
#include <string>

// 决策输出结构体（保持不变）
struct DecisionOutput {
    geometry_msgs::msg::Point target_position;
    SentryControl control_msg;
    bool target_needs_publishing = false;
    bool control_needs_publishing = false;
    std::string decision_reason;
};

// 状态机状态枚举
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

    // 更新接口
    void updateOurState(const OurRobotState::SharedPtr msg);
    void updateGameState(const GameState::SharedPtr msg);

    // 主决策函数
    DecisionOutput executeDecision();

    std::shared_ptr<Blackboard> getBlackboard() const { return blackboard_; }

private:
    std::shared_ptr<Blackboard> blackboard_;
    State current_state_ = State::IDLE;
    double last_state_entry_time_ = 0.0;

    // 工具函数
    bool needSupply() const;                // 判断是否需要补给
    bool shouldInterruptAttack() const;     // 判断是否应该中断攻击
    bool supplyComplete() const;            // 判断补给是否完成
    bool resurrectionComplete() const;      // 判断复活是否完成
    void transitionTo(State new_state);     // 状态转移
    std::string stateToString(State state) const;
};

#endif // DECISION_MANAGER_HPP
