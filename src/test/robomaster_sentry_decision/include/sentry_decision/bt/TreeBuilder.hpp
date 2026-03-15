#ifndef TREEBUILDER_HPP
#define TREEBUILDER_HPP

#include "BTNode.hpp"
#include "BTComposite.hpp"
#include "BTAction.hpp"
#include "BTCondition.hpp"
#include <memory>

class TreeBuilder {
public:
    // 构建主决策树
    static std::shared_ptr<BTNode> buildMainDecisionTree();
    
    // 构建初始化行为树
    static std::shared_ptr<BTNode> buildInitializationTree();
    
    // 动态目标选择节点
    class DynamicTargetAction : public BTAction {
    public:
        DynamicTargetAction() : BTAction("DynamicTargetAction") {}
        
        BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                        std::shared_ptr<RegionManager> region_manager) override;
    };
    
    // 动态增益点选择节点
    class DynamicGainPointAction : public BTAction {
    public:
        DynamicGainPointAction() : BTAction("DynamicGainPointAction") {}
        
        BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                        std::shared_ptr<RegionManager> region_manager) override;
    };
};

#endif // TREEBUILDER_HPP
