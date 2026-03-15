#include "sentry_decision/bt/TreeBuilder.hpp"
#include "sentry_decision/Models.hpp"          
#include <memory>
#include <iostream>

// DynamicTargetAction 实现
BTStatus TreeBuilder::DynamicTargetAction::execute(std::shared_ptr<Blackboard> blackboard,
                                                  std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard || !region_manager) return BTStatus::FAILURE;
    
    if (!blackboard->current_target_id.empty()) {
        auto attack_node = std::make_shared<SetRobotAttackTarget>(blackboard->current_target_id);
        return attack_node->execute(blackboard, region_manager);
    }
    return BTStatus::FAILURE;
}

// DynamicGainPointAction 实现
BTStatus TreeBuilder::DynamicGainPointAction::execute(std::shared_ptr<Blackboard> blackboard,
                                                     std::shared_ptr<RegionManager> region_manager) {
    if (!blackboard) return BTStatus::FAILURE;
    
    auto gain_point_scores = Models::calculateGainPointScores(*blackboard);
    if (!gain_point_scores.empty()) {
        auto& best_gain = gain_point_scores[0];
        auto gain_node = std::make_shared<SetGainPointTarget>(best_gain.name, best_gain.position);
        return gain_node->execute(blackboard, region_manager);
    }
    return BTStatus::FAILURE;
}

// 创建一个简单的具体节点类用于初始化树
class SimpleInitNode : public BTNode {
public:
    SimpleInitNode() {
        setNodeName("SimpleInitNode");
    }
    
    BTStatus execute(std::shared_ptr<Blackboard> blackboard,
                    std::shared_ptr<RegionManager> region_manager) override {
        (void)blackboard;
        (void)region_manager;
        // 这个节点总是返回RUNNING，让初始化流程由DecisionManager控制
        return BTStatus::RUNNING;
    }
};

std::shared_ptr<BTNode> TreeBuilder::buildMainDecisionTree() {
    // 根节点是选择器，按优先级检查各个条件
    auto root = std::make_shared<BTSelector>();
    
    // 1. 复活处理（最高优先级）
    auto resurrection_seq = std::make_shared<BTSequence>();
    auto check_resurrection = std::make_shared<CheckResurrection>();
    auto set_resurrection_target = std::make_shared<SetResurrectionTarget>();
    resurrection_seq->addChild(check_resurrection);
    resurrection_seq->addChild(set_resurrection_target);
    root->addChild(resurrection_seq);
    
    // 2. 关键事件检查（次高优先级）
    auto critical_events_selector = std::make_shared<BTSelector>();
    
    // 2.1 我方基地危急
    auto base_critical_seq = std::make_shared<BTSequence>();
    auto check_base_critical = std::make_shared<CheckBaseCritical>();
    auto set_base_defense = std::make_shared<SetBaseDefenseTarget>();
    base_critical_seq->addChild(check_base_critical);
    base_critical_seq->addChild(set_base_defense);
    critical_events_selector->addChild(base_critical_seq);
    
    // 2.2 前哨站被毁
    auto outpost_destroyed_seq = std::make_shared<BTSequence>();
    auto check_outpost_destroyed = std::make_shared<CheckOutpostDestroyed>();
    auto fortress_decision_selector = std::make_shared<BTSelector>();
    
    auto fortress_occupy_seq = std::make_shared<BTSequence>();
    auto check_fortress_condition = std::make_shared<CheckFortressOccupyCondition>();
    auto set_fortress_occupy = std::make_shared<SetFortressOccupyTarget>();
    fortress_occupy_seq->addChild(check_fortress_condition);
    fortress_occupy_seq->addChild(set_fortress_occupy);
    fortress_decision_selector->addChild(fortress_occupy_seq);
    
    auto base_defense_fallback_seq = std::make_shared<BTSequence>();
    auto set_base_defense_fallback = std::make_shared<SetBaseDefenseTarget>();
    base_defense_fallback_seq->addChild(set_base_defense_fallback);
    fortress_decision_selector->addChild(base_defense_fallback_seq);
    
    outpost_destroyed_seq->addChild(check_outpost_destroyed);
    outpost_destroyed_seq->addChild(fortress_decision_selector);
    critical_events_selector->addChild(outpost_destroyed_seq);
    
    root->addChild(critical_events_selector);
    
    // 3. 补给需求
    auto supply_seq = std::make_shared<BTSequence>();
    auto evaluate_supply = std::make_shared<EvaluateSupplyNeed>();
    auto set_supply_target = std::make_shared<SetSupplyTarget>();
    auto check_supply_complete = std::make_shared<CheckSupplyComplete>();
    supply_seq->addChild(evaluate_supply);
    supply_seq->addChild(set_supply_target);
    supply_seq->addChild(check_supply_complete);
    root->addChild(supply_seq);
    
    // 4. 高优先级英雄攻击（英雄在部署区）
    auto hero_attack_high_seq = std::make_shared<BTSequence>();
    auto check_hero_condition = std::make_shared<CheckHeroAttackCondition>();
    auto set_hero_target = std::make_shared<SetHeroAttackTarget>();
    auto check_attack_duration = std::make_shared<CheckAttackDuration>();
    hero_attack_high_seq->addChild(check_hero_condition);
    hero_attack_high_seq->addChild(set_hero_target);
    hero_attack_high_seq->addChild(check_attack_duration);
    root->addChild(hero_attack_high_seq);
    
    // 5. 能量机关激活
    auto energy_activation_seq = std::make_shared<BTSequence>();
    auto evaluate_energy = std::make_shared<EvaluateEnergyActivation>();
    auto set_energy_target = std::make_shared<SetEnergyActivationTarget>();
    energy_activation_seq->addChild(evaluate_energy);
    energy_activation_seq->addChild(set_energy_target);
    root->addChild(energy_activation_seq);
    
    // 6. 英雄攻击（常规）
    auto hero_attack_seq = std::make_shared<BTSequence>();
    auto evaluate_hero = std::make_shared<EvaluateHeroAttack>();
    hero_attack_seq->addChild(evaluate_hero);
    hero_attack_seq->addChild(set_hero_target);
    hero_attack_seq->addChild(check_attack_duration);
    root->addChild(hero_attack_seq);
    
    // 7. 飞坡决策已移除，现在飞坡流程由SetHeroAttackTarget等节点直接触发
    
    // 8. 目标选择（攻击其他机器人）
    auto target_selection_seq = std::make_shared<BTSequence>();
    auto evaluate_target = std::make_shared<EvaluateTargetSelection>();
    target_selection_seq->addChild(evaluate_target);
    auto dynamic_target_action = std::make_shared<DynamicTargetAction>();
    target_selection_seq->addChild(dynamic_target_action);
    target_selection_seq->addChild(check_attack_duration);
    root->addChild(target_selection_seq);
    
    // 9. 增益点占领
    auto gain_point_seq = std::make_shared<BTSequence>();
    auto evaluate_gain = std::make_shared<EvaluateGainPoint>();
    auto dynamic_gain_action = std::make_shared<DynamicGainPointAction>();
    gain_point_seq->addChild(evaluate_gain);
    gain_point_seq->addChild(dynamic_gain_action);
    root->addChild(gain_point_seq);
    
    // 10. 基地防御（最低优先级）
    auto base_defense_seq = std::make_shared<BTSequence>();
    auto set_base_defense_default = std::make_shared<SetBaseDefenseTarget>();
    base_defense_seq->addChild(set_base_defense_default);
    root->addChild(base_defense_seq);
    
    return root;
}

std::shared_ptr<BTNode> TreeBuilder::buildInitializationTree() {
    // 简化初始化流程：返回一个简单的节点，让DecisionManager处理初始化
    auto simple_init_node = std::make_shared<SimpleInitNode>();
    return simple_init_node;
}
