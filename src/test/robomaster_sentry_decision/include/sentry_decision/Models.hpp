#ifndef MODELS_HPP
#define MODELS_HPP

#include "Blackboard.hpp"
#include "Constants.hpp"
#include <vector>
#include <string>
#include <map>

class Models {
public:
    // 构造函数
    Models() = default;
    
    // 综合态势评分 (Z值计算)
    static double calculateSituationZ(const Blackboard& bb);
    
    // 能量机关激活价值评估
    static double calculateEnergyValue(const Blackboard& bb, double distance_to_energy);
    
    // 目标选择模型
    struct TargetScore {
        std::string id;      // 目标ID (hero, engineer, infantry3, infantry4, sentry)
        std::string type;    // 目标类型
        double score;        // 综合评分
    };
    
    static std::vector<TargetScore> calculateAllTargetScores(const Blackboard& bb);
    
    // 姿态决策模型
    static double calculatePostureScore(const Blackboard& bb, double distance_to_nearest_enemy);
    
    // 补给需求评估模型
    static double calculateSupplyScore(const Blackboard& bb, double distance_to_supply);
    
    // 堡垒占领决策模型
    static double calculateFortressValue(const Blackboard& bb, double distance_to_fortress);
    
    // 英雄攻击价值评估
    static double calculateHeroAttackValue(const Blackboard& bb, double distance_to_hero, bool deploy_state);
    
    // 增益点价值评估
    struct GainPointScore {
        std::string name;
        geometry_msgs::msg::Point position;
        double score;
        double defense_gain;
    };
    
    static std::vector<GainPointScore> calculateGainPointScores(const Blackboard& bb);
    
    // 计算两点距离 (单位：cm)
    static double calculateDistance(double x1, double y1, double x2, double y2);
    
    // 获取距离最近敌人的距离
    static double getNearestEnemyDistance(const Blackboard& bb);
    
    // 检查是否需要飞坡
    static bool needRamp(const Blackboard& bb, const geometry_msgs::msg::Point& target);
    
    // 工具函数：获取机器人类型权重
    static double getTypeWeight(const std::string& robot_type);
    
    // 工具函数：获取机器人最大血量
    static double getMaxHP(const std::string& robot_type);
    
    // 工具函数：获取机器人最大弹量
    static double getMaxAmmo(const std::string& robot_type);
    
private:
    // 内部使用的权重配置
    static const std::map<std::string, double> TARGET_WEIGHTS;
    static const std::map<std::string, double> HP_WEIGHTS;
    static const std::map<std::string, double> AMMO_WEIGHTS;
    
    // 计算攻击评分
    static double calculateAttackScore(const Blackboard& bb, const EnemyInfo& enemy, 
                                      double distance, const std::string& type);
    
    // 计算防御评分
    static double calculateDefenseScore(const Blackboard& bb, double distance_to_target);
    
    // 计算增益点距离衰减因子
    static double calculateDistanceDecay(double distance, double max_distance);
};

#endif // MODELS_HPP
