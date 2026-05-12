#ifndef DECISION_MANAGER_HPP
#define DECISION_MANAGER_HPP

#include "Blackboard.hpp"
#include "Models.hpp"
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
    INIT_MOVE,
    INIT_ATTACK,
    MOVE_TO_ATTACK_HERO,
    ATTACK_HERO,
    MOVE_TO_ATTACK_ROBOT,
    ATTACK_ROBOT,
    MOVE_TO_SUPPLY,
    SUPPLYING,
    RESURRECTION_MOVE,
    RESURRECTING,
    MOVE_TO_BASE_DEFENSE,
    BASE_DEFENSE,
    MOVE_TO_GAIN_POINT,
    OCCUPY_GAIN_POINT,
    MOVE_TO_FORTRESS,
    OCCUPY_FORTRESS,
    MOVE_TO_GUARD,
    GUARD,
    MOVE_TO_ENEMY_FORTRESS,   // 新增：前往敌方堡垒
    OCCUPY_ENEMY_FORTRESS,    // 新增：占领敌方堡垒
    MOVE_TO_RAMP              // 新增：飞坡移动
};

struct PriorityTargetResult {
    std::string enemy_id;
    std::string type;
    double score;
    bool valid = false;
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

    // 飞坡暂存
    State pending_state_ = State::IDLE;
    geometry_msgs::msg::Point pending_target_;
    bool has_pending_state_ = false;

    // 辅助判断函数
    bool needSupply() const;
    bool shouldInterruptForResurrectionOrSupply() const;
    bool checkBaseCritical() const;
    bool checkOutpostDestroyed() const;
    bool checkFortressOccupy() const;
    bool checkGainPoint() const;
    bool checkEnemyFortress() const;      // 是否可以占领敌方堡垒
    bool needRamp(const geometry_msgs::msg::Point& target) const;   // 是否需要飞坡

    void updateHeroDeployFlag();
    void updateMustOccupyFlag();         // 更新强制占领标志

    PriorityTargetResult selectPriorityTarget();

    geometry_msgs::msg::Point getTargetPointForEnemy(const std::string& enemy_id) const;
    Models::GainPointScore getBestGainPoint() const;

    void transitionTo(State new_state);
    std::string stateToString(State state) const;

    static constexpr double GUARD_X = 690.0;
    static constexpr double GUARD_Y = 760.0;
};

#endif // DECISION_MANAGER_HPP
