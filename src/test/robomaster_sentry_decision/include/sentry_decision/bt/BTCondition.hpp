#ifndef BTCONDITION_HPP
#define BTCONDITION_HPP

#include "BTNode.hpp"
#include "sentry_decision/Blackboard.hpp"
#include "sentry_decision/RegionManager.hpp"
#include "sentry_decision/Models.hpp"
#include "sentry_decision/Constants.hpp"
#include <string>

// 基础条件节点
class BTCondition : public BTNode {
public:
    BTCondition(const std::string& name) {
        setNodeName(name);
    }
};

// 检查游戏是否开始
class CheckGameStarted : public BTCondition {
public:
    CheckGameStarted() : BTCondition("CheckGameStarted") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 检查复活状态
class CheckResurrection : public BTCondition {
public:
    CheckResurrection() : BTCondition("CheckResurrection") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 检查基地危急
class CheckBaseCritical : public BTCondition {
public:
    CheckBaseCritical() : BTCondition("CheckBaseCritical") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 检查前哨站被毁
class CheckOutpostDestroyed : public BTCondition {
public:
    CheckOutpostDestroyed() : BTCondition("CheckOutpostDestroyed") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 检查补给RFID
class CheckSupplyRFID : public BTCondition {
public:
    CheckSupplyRFID() : BTCondition("CheckSupplyRFID") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 检查是否在飞坡点
class CheckAtRampPoint : public BTCondition {
public:
    CheckAtRampPoint() : BTCondition("CheckAtRampPoint") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 检查攻击持续时间
class CheckAttackDuration : public BTCondition {
public:
    CheckAttackDuration() : BTCondition("CheckAttackDuration") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 检查补给完成
class CheckSupplyComplete : public BTCondition {
public:
    CheckSupplyComplete() : BTCondition("CheckSupplyComplete") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 能量机关激活决策
class EvaluateEnergyActivation : public BTCondition {
public:
    EvaluateEnergyActivation() : BTCondition("EvaluateEnergyActivation") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 补给需求评估
class EvaluateSupplyNeed : public BTCondition {
public:
    EvaluateSupplyNeed() : BTCondition("EvaluateSupplyNeed") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 英雄攻击评估
class EvaluateHeroAttack : public BTCondition {
public:
    EvaluateHeroAttack() : BTCondition("EvaluateHeroAttack") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 检查英雄攻击条件（高优先级）
class CheckHeroAttackCondition : public BTCondition {
public:
    CheckHeroAttackCondition() : BTCondition("CheckHeroAttackCondition") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 检查是否需要飞坡
class CheckNeedRamp : public BTCondition {
public:
    CheckNeedRamp() : BTCondition("CheckNeedRamp") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 目标选择评估
class EvaluateTargetSelection : public BTCondition {
public:
    EvaluateTargetSelection() : BTCondition("EvaluateTargetSelection") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
    
    std::string getSelectedTarget() const { return selected_target_; }
    
private:
    std::string selected_target_;
};

// 飞坡需求评估
class EvaluateRampNeed : public BTCondition {
public:
    EvaluateRampNeed() : BTCondition("EvaluateRampNeed") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 增益点价值评估
class EvaluateGainPoint : public BTCondition {
public:
    EvaluateGainPoint() : BTCondition("EvaluateGainPoint") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
    
    Models::GainPointScore getSelectedGainPoint() const { return selected_gain_point_; }
    
private:
    Models::GainPointScore selected_gain_point_;
};

// 检查堡垒占领条件
class CheckFortressOccupyCondition : public BTCondition {
public:
    CheckFortressOccupyCondition() : BTCondition("CheckFortressOccupyCondition") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 检查初始化完成
class CheckInitializationComplete : public BTCondition {
public:
    CheckInitializationComplete() : BTCondition("CheckInitializationComplete") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 检查是否在能量机关点
class CheckAtEnergyPoint : public BTCondition {
public:
    CheckAtEnergyPoint() : BTCondition("CheckAtEnergyPoint") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 检查能量机关是否激活
class CheckEnergyActivated : public BTCondition {
public:
    CheckEnergyActivated() : BTCondition("CheckEnergyActivated") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

// 检查前哨站状态
class CheckOutpostStatus : public BTCondition {
public:
    CheckOutpostStatus() : BTCondition("CheckOutpostStatus") {}
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override;
};

#endif // BTCONDITION_HPP
