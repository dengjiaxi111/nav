// nav_core/include/nav_core/nav_fsm.hpp
// 导航状态机 - 核心逻辑

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include "nav_core/types.hpp"

namespace nav_core {

class NavFSM {
public:
    using StateChangeCallback = std::function<void(NavState, NavState)>;
    
    explicit NavFSM(rclcpp::Logger logger) : logger_(logger) {}
    
    NavState state() const { return state_; }
    RecoveryTrigger recoveryTrigger() const { return trigger_; }
    int recoveryCount() const { return recovery_count_; }
    
    // 状态转换
    bool transitionTo(NavState new_state) {
        if (state_ == new_state) return true;
        
        NavState old = state_;
        state_ = new_state;
        
        RCLCPP_INFO(logger_, "状态: %s -> %s", toString(old), toString(new_state));
        
        if (on_change_) on_change_(old, new_state);
        return true;
    }
    
    // 触发恢复
    void triggerRecovery(RecoveryTrigger trigger) {
        trigger_ = trigger;
        recovery_count_++;
        transitionTo(NavState::RECOVERY);
    }
    
    // 重置
    void reset() {
        state_ = NavState::IDLE;
        trigger_ = RecoveryTrigger::NONE;
        recovery_count_ = 0;
    }
    
    // 检查是否超过最大恢复次数
    bool recoveryExhausted() const {
        return recovery_count_ >= max_recovery_;
    }
    
    void setMaxRecovery(int n) { max_recovery_ = n; }
    void onStateChange(StateChangeCallback cb) { on_change_ = cb; }
    
private:
    rclcpp::Logger logger_;
    NavState state_ = NavState::IDLE;
    RecoveryTrigger trigger_ = RecoveryTrigger::NONE;
    int recovery_count_ = 0;
    int max_recovery_ = 3;
    StateChangeCallback on_change_;
};

}  // namespace nav_core
