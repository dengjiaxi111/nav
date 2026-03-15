#ifndef BTCOMPOSITE_HPP
#define BTCOMPOSITE_HPP

#include "BTNode.hpp"

class BTSelector : public BTComposite {
public:
    BTSelector() = default;
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override {
        // 选择器：按顺序执行子节点，直到有一个成功或全部失败
        for (auto& child : children_) {
            BTStatus status = child->execute(blackboard, region_manager);
            if (status == BTStatus::SUCCESS) {
                return BTStatus::SUCCESS;
            } else if (status == BTStatus::RUNNING) {
                return BTStatus::RUNNING;
            }
        }
        return BTStatus::FAILURE;
    }
};

class BTSequence : public BTComposite {
public:
    BTSequence() = default;
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override {
        // 序列：按顺序执行子节点，直到有一个失败或全部成功
        for (auto& child : children_) {
            BTStatus status = child->execute(blackboard, region_manager);
            if (status == BTStatus::FAILURE) {
                return BTStatus::FAILURE;
            } else if (status == BTStatus::RUNNING) {
                return BTStatus::RUNNING;
            }
        }
        return BTStatus::SUCCESS;
    }
};

#endif // BTCOMPOSITE_HPP
