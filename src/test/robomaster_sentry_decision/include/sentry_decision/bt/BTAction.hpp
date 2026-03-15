#ifndef BTACTION_HPP
#define BTACTION_HPP

#include "BTNode.hpp"
#include "sentry_decision/Blackboard.hpp"
#include "sentry_decision/RegionManager.hpp"
#include "sentry_decision/Constants.hpp"
#include "sentry_decision/Models.hpp"
#include <iostream>

using namespace SentryConstants;

// 基础动作节点
class BTAction : public BTNode {
public:
    BTAction(const std::string& name) {
        setNodeName(name);
    }
    
    virtual ~BTAction() = default;
    
protected:
    // 更新控制消息
    void updateControlMsg(std::shared_ptr<Blackboard> blackboard,
                         uint8_t gimbal_mode,
                         uint8_t spin_mode,
                         uint8_t posture,
                         uint8_t ramp_mode) {
        if (blackboard) {
            blackboard->updateControlMsg(gimbal_mode, spin_mode, posture, ramp_mode);
        }
    }
};

// 能量机关激活目标
class SetEnergyActivationTarget : public BTAction {
public:
    SetEnergyActivationTarget() : BTAction("SetEnergyActivationTarget") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 前哨站攻击目标
class SetOutpostAttackTarget : public BTAction {
public:
    SetOutpostAttackTarget() : BTAction("SetOutpostAttackTarget") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 基地防御目标
class SetBaseDefenseTarget : public BTAction {
public:
    SetBaseDefenseTarget() : BTAction("SetBaseDefenseTarget") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 补给目标
class SetSupplyTarget : public BTAction {
public:
    SetSupplyTarget() : BTAction("SetSupplyTarget") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 英雄攻击目标
class SetHeroAttackTarget : public BTAction {
public:
    SetHeroAttackTarget() : BTAction("SetHeroAttackTarget") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
    
    void reset() override;
};

// 机器人攻击目标
class SetRobotAttackTarget : public BTAction {
public:
    explicit SetRobotAttackTarget(const std::string& target_id) 
        : BTAction("SetRobotAttackTarget_" + target_id), target_id_(target_id) {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
    
private:
    std::string target_id_;
};

// 飞坡点目标
class SetRampPointTarget : public BTAction {
public:
    SetRampPointTarget() : BTAction("SetRampPointTarget") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 增益点目标
class SetGainPointTarget : public BTAction {
public:
    SetGainPointTarget(const std::string& gain_point_name, 
                      const geometry_msgs::msg::Point& position)
        : BTAction("SetGainPointTarget_" + gain_point_name), 
          gain_point_name_(gain_point_name), 
          position_(position) {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
    
private:
    std::string gain_point_name_;
    geometry_msgs::msg::Point position_;
};

// 复活目标
class SetResurrectionTarget : public BTAction {
public:
    SetResurrectionTarget() : BTAction("SetResurrectionTarget") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 敌方堡垒占领目标
class SetFortressOccupyTarget : public BTAction {
public:
    SetFortressOccupyTarget() : BTAction("SetFortressOccupyTarget") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

#endif // BTACTION_HPP
