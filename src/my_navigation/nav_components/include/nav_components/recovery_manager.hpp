// nav_components/include/nav_components/recovery_manager.hpp
// 恢复行为管理器

#pragma once
#include <nav_core/recovery_base.hpp>
#include <nav_core/types.hpp>
#include <vector>
#include <memory>

namespace nav_components {

class RecoveryManager {
public:
    void addRecovery(std::shared_ptr<nav_core::RecoveryBase> recovery) {
        recoveries_.push_back(recovery);
    }
    
    // 开始恢复序列
    void start(nav_core::RecoveryTrigger trigger,
               const geometry_msgs::msg::PoseStamped& pose) {
        trigger_ = trigger;
        current_idx_ = 0;
        if (!recoveries_.empty()) {
            recoveries_[0]->start(pose);
        }
    }
    
    // 更新当前恢复行为
    nav_core::RecoveryStatus update(const geometry_msgs::msg::PoseStamped& pose) {
        if (current_idx_ >= recoveries_.size()) {
            return nav_core::RecoveryStatus::FAILED;  // 所有恢复都失败
        }
        
        auto status = recoveries_[current_idx_]->update(pose);
        
        if (status == nav_core::RecoveryStatus::SUCCEEDED) {
            return nav_core::RecoveryStatus::SUCCEEDED;
        }
        
        if (status == nav_core::RecoveryStatus::FAILED) {
            // 尝试下一个恢复行为
            current_idx_++;
            if (current_idx_ < recoveries_.size()) {
                recoveries_[current_idx_]->start(pose);
                return nav_core::RecoveryStatus::RUNNING;
            }
            return nav_core::RecoveryStatus::FAILED;
        }
        
        return status;
    }

    // 只更新当前恢复行为，不在 FAILED 时自动切换到下一个。
    nav_core::RecoveryStatus updateCurrent(const geometry_msgs::msg::PoseStamped& pose) {
        if (current_idx_ >= recoveries_.size()) {
            return nav_core::RecoveryStatus::FAILED;
        }
        return recoveries_[current_idx_]->update(pose);
    }

    bool advanceToNext(const geometry_msgs::msg::PoseStamped& pose) {
        if (current_idx_ < recoveries_.size()) {
            recoveries_[current_idx_]->cancel();
        }
        current_idx_++;
        if (current_idx_ < recoveries_.size()) {
            recoveries_[current_idx_]->start(pose);
            return true;
        }
        return false;
    }
    
    void cancel() {
        if (current_idx_ < recoveries_.size()) {
            recoveries_[current_idx_]->cancel();
        }
    }
    
    void reset() {
        current_idx_ = 0;
        trigger_ = nav_core::RecoveryTrigger::NONE;
    }
    
    const char* currentRecoveryName() const {
        if (current_idx_ < recoveries_.size()) {
            return recoveries_[current_idx_]->name();
        }
        return "none";
    }

private:
    std::vector<std::shared_ptr<nav_core::RecoveryBase>> recoveries_;
    size_t current_idx_ = 0;
    nav_core::RecoveryTrigger trigger_ = nav_core::RecoveryTrigger::NONE;
};

}  // namespace nav_components
