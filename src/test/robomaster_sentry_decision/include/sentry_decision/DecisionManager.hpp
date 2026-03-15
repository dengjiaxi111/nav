#ifndef DECISION_MANAGER_HPP
#define DECISION_MANAGER_HPP

#include "Blackboard.hpp"
#include "RegionManager.hpp"
#include "sentry_decision/bt/TreeBuilder.hpp"
#include "sentry_decision/msg/sentry_control.hpp"
#include <memory>
#include <string>

// 包含决策消息
#include "decision_messages/msg/our_robot_state.hpp"
#include "decision_messages/msg/enemy_robot_state.hpp"
#include "decision_messages/msg/game_state.hpp"

using OurRobotState = decision_messages::msg::OurRobotState;
using EnemyRobotState = decision_messages::msg::EnemyRobotState;
using GameState = decision_messages::msg::GameState;
using SentryControl = sentry_decision::msg::SentryControl;

struct DecisionOutput {
    geometry_msgs::msg::Point target_position;
    SentryControl control_msg;
    std::string decision_reason;
    bool target_needs_publishing;  // 是否需要发布目标点
    bool control_needs_publishing; // 是否需要发布控制消息
};

class GameStateProcessor {
public:
    enum CriticalEvent {
        NONE = 0,
        OUR_BASE_CRITICAL = 1,
        OUTPOST_DESTROYED = 2
    };
    
    GameStateProcessor(std::shared_ptr<Blackboard> blackboard) 
        : blackboard_(blackboard), init_state_(INIT_WAIT_FOR_START) {}
    
    void processGameStage();
    bool isInInitializationPhase() const { 
        return !blackboard_->initialization_complete && blackboard_->stage == 4; 
    }
    
    CriticalEvent checkCriticalEvents() const;
    
    enum InitState {
        INIT_WAIT_FOR_START = 0,
        INIT_GO_TO_ENERGY = 1,
        INIT_WAIT_ENERGY_ACTIVATION = 2,
        INIT_CHECK_OUTPOST = 3,
        INIT_GO_TO_OUTPOST = 4,
        INIT_ATTACK_OUTPOST = 5,
        INIT_COMPLETE = 6
    };
    
    InitState getInitState() const { return init_state_; }
    void setInitState(InitState state) { init_state_ = state; }
    
private:
    std::shared_ptr<Blackboard> blackboard_;
    InitState init_state_;
};

class DecisionManager {
public:
    DecisionManager();
    
    // 更新状态
    void updateOurState(const OurRobotState::SharedPtr msg);
    void updateEnemyState(const EnemyRobotState::SharedPtr msg);
    void updateGameState(const GameState::SharedPtr msg);
    
    // 执行决策
    DecisionOutput executeDecision();
    
    // 获取黑板的共享指针
    std::shared_ptr<Blackboard> getBlackboard() const { return blackboard_; }
    
    // 行为状态名称
    std::string getBehaviorStateName(BehaviorState state) const;
    
    // 行为类型名称
    std::string getBehaviorTypeName(BehaviorType type) const;
    
private:
    // 处理关键事件
    void handleCriticalEvent(GameStateProcessor::CriticalEvent event);
    
    // 成员变量
    std::shared_ptr<Blackboard> blackboard_;
    std::shared_ptr<RegionManager> region_manager_;
    std::shared_ptr<GameStateProcessor> game_state_processor_;
    std::shared_ptr<BTNode> behavior_tree_;
    std::shared_ptr<BTNode> initialization_tree_;
    
    // 决策记录
    std::string last_decision_reason_;
    double last_decision_time_;
    
    // 初始化流程控制标志
    bool init_energy_started_;   // 打符是否已开始
    bool init_outpost_started_;  // 打前哨站是否已开始
    
    // 初始化流程
    bool executeInitialization();
    
    // 检查行为是否完成
    bool checkBehaviorCompletion();
    
    // 开始新的行为决策
    bool startNewBehaviorDecision();
};

#endif // DECISION_MANAGER_HPP
