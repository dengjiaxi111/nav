#ifndef MODELS_HPP
#define MODELS_HPP

#include "Blackboard.hpp"
#include <vector>
#include <string>

class Models {
public:
    // 综合态势评分 (Z值)
    static double calculateSituationZ(const Blackboard& bb);

    // 英雄攻击价值评估（用于部署模式）
    static double calculateHeroAttackValue(const Blackboard& bb, double distance_to_hero, bool deploy_state);

    // 通用目标评分（血量、距离、弹量，4:4:2 权重）
    static double calculateGeneralTargetScore(const Blackboard& bb, const EnemyInfo& enemy,
                                              double weight_hp, double weight_distance, double weight_ammo);

    // 目标选择评分（返回所有可见敌人的评分，按分数降序）—— 可能不再使用，保留兼容
    struct TargetScore {
        std::string id;
        std::string type;
        double score;
    };
    static std::vector<TargetScore> calculateAllTargetScores(const Blackboard& bb);

    // 增益点价值评估
    struct GainPointScore {
        std::string name;
        geometry_msgs::msg::Point position;
        double score;
        double defense_gain;
    };
    static std::vector<GainPointScore> calculateGainPointScores(const Blackboard& bb);

    // 堡垒占领价值评估
    static double calculateFortressValue(const Blackboard& bb, double distance_to_fortress);

    // 辅助函数
    static double calculateDistance(double x1, double y1, double x2, double y2);
    static double getNearestEnemyDistance(const Blackboard& bb);
    static double getTypeWeight(const std::string& robot_type);
    static double getMaxHP(const std::string& robot_type);
    static double getMaxAmmo(const std::string& robot_type);
};

#endif // MODELS_HPP
