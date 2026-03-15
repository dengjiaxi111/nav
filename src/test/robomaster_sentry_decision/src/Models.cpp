#include "sentry_decision/Models.hpp"
#include <cmath>
#include <algorithm>
#include <functional>

using namespace SentryConstants;

double Models::calculateSituationZ(const Blackboard& bb) {
    double hp_ratio = bb.current_hp / 400.0;
    double ammo_ratio = bb.allowance_17mm / 300.0;
    double base_ratio = bb.our_base_hp / 5000.0;
    
    return 0.3 * hp_ratio + 0.3 * ammo_ratio + 0.4 * base_ratio;
}


double Models::calculateEnergyValue(const Blackboard& bb, double distance_to_energy) {
    double activatable = bb.energy_mechanism_activatable ? 1.0 : 0.0;
    double distance_score = 1.0 - std::min(distance_to_energy / 1400.0, 1.0);  // 14m = 1400cm
    double ammo_score = bb.allowance_17mm / 300.0;
    
    return 0.40 * activatable + 0.40 * distance_score + 0.20 * ammo_score;
}

std::vector<Models::TargetScore> Models::calculateAllTargetScores(const Blackboard& bb) {
    std::vector<TargetScore> scores;
    
    // 英雄机器人
    if (bb.enemy_hero.visible && bb.enemy_hero.hp > 0) {
        double distance = calculateDistance(bb.x, bb.y, bb.enemy_hero.x, bb.enemy_hero.y);
        double hp_score = 1.0 - (bb.enemy_hero.hp / getMaxHP("hero"));
        double distance_score = 1.0 - std::min(distance / 2800.0, 1.0);  // 28m = 2800cm
        double type_score = getTypeWeight("hero") / 1.6;
        double ammo_score = 1.0 - (bb.enemy_hero.allowance / getMaxAmmo("hero"));
        
        double total_score = 0.35 * hp_score + 0.30 * distance_score + 
                            0.25 * type_score + 0.10 * ammo_score;
        
        scores.push_back({"hero", "hero", total_score});
    }
    
    // 工程机器人
    if (bb.enemy_engineer.visible && bb.enemy_engineer.hp > 0) {
        double distance = calculateDistance(bb.x, bb.y, bb.enemy_engineer.x, bb.enemy_engineer.y);
        double hp_score = 1.0 - (bb.enemy_engineer.hp / getMaxHP("engineer"));
        double distance_score = 1.0 - std::min(distance / 2800.0, 1.0);
        double type_score = getTypeWeight("engineer") / 1.6;
        double ammo_score = 1.0 - (bb.enemy_engineer.allowance / getMaxAmmo("engineer"));
        
        double total_score = 0.35 * hp_score + 0.30 * distance_score + 
                            0.25 * type_score + 0.10 * ammo_score;
        
        scores.push_back({"engineer", "engineer", total_score});
    }
    
    // 步兵3机器人
    if (bb.enemy_infantry3.visible && bb.enemy_infantry3.hp > 0) {
        double distance = calculateDistance(bb.x, bb.y, bb.enemy_infantry3.x, bb.enemy_infantry3.y);
        double hp_score = 1.0 - (bb.enemy_infantry3.hp / getMaxHP("infantry"));
        double distance_score = 1.0 - std::min(distance / 2800.0, 1.0);
        double type_score = getTypeWeight("infantry") / 1.6;
        double ammo_score = 1.0 - (bb.enemy_infantry3.allowance / getMaxAmmo("infantry"));
        
        double total_score = 0.35 * hp_score + 0.30 * distance_score + 
                            0.25 * type_score + 0.10 * ammo_score;
        
        scores.push_back({"infantry3", "infantry", total_score});
    }
    
    // 步兵4机器人
    if (bb.enemy_infantry4.visible && bb.enemy_infantry4.hp > 0) {
        double distance = calculateDistance(bb.x, bb.y, bb.enemy_infantry4.x, bb.enemy_infantry4.y);
        double hp_score = 1.0 - (bb.enemy_infantry4.hp / getMaxHP("infantry"));
        double distance_score = 1.0 - std::min(distance / 2800.0, 1.0);
        double type_score = getTypeWeight("infantry") / 1.6;
        double ammo_score = 1.0 - (bb.enemy_infantry4.allowance / getMaxAmmo("infantry"));
        
        double total_score = 0.35 * hp_score + 0.30 * distance_score + 
                            0.25 * type_score + 0.10 * ammo_score;
        
        scores.push_back({"infantry4", "infantry", total_score});
    }
    
    // 哨兵机器人
    if (bb.enemy_sentry.visible && bb.enemy_sentry.hp > 0) {
        double distance = calculateDistance(bb.x, bb.y, bb.enemy_sentry.x, bb.enemy_sentry.y);
        double hp_score = 1.0 - (bb.enemy_sentry.hp / getMaxHP("sentry"));
        double distance_score = 1.0 - std::min(distance / 2800.0, 1.0);
        double type_score = getTypeWeight("sentry") / 1.6;
        double ammo_score = 1.0 - (bb.enemy_sentry.allowance / getMaxAmmo("sentry"));
        
        double total_score = 0.35 * hp_score + 0.30 * distance_score + 
                            0.25 * type_score + 0.10 * ammo_score;
        
        scores.push_back({"sentry", "sentry", total_score});
    }
    
    // 按分数从高到低排序
    std::sort(scores.begin(), scores.end(), 
        [](const TargetScore& a, const TargetScore& b) {
            return a.score > b.score;
        });
    
    return scores;
}

double Models::calculatePostureScore(const Blackboard& bb, double distance_to_nearest_enemy) {
    double hp_score = 1.0 - (bb.current_hp / 400.0);
    double ammo_score = 1.0 - (bb.allowance_17mm / 300.0);
    double distance_score = 1.0 - std::min(distance_to_nearest_enemy / 1500.0, 1.0);  // 15m = 1500cm
    
    return 0.40 * hp_score + 0.40 * ammo_score + 0.20 * distance_score;
}

double Models::calculateSupplyScore(const Blackboard& bb, double distance_to_supply) {
    double hp_need = std::max(0.0, 0.4 - (bb.current_hp / 400.0));
    double ammo_need = std::max(0.0, 0.4 - (bb.allowance_17mm / 300.0));
    double distance_score = 1.0 - std::min(distance_to_supply / 1400.0, 1.0);  // 14m = 1400cm
    
    return 0.60 * hp_need + 0.30 * ammo_need + 0.10 * distance_score;
}

double Models::calculateFortressValue(const Blackboard& bb, double distance_to_fortress) {
    double hp_score = bb.current_hp / 400.0;
    double base_score = bb.our_base_hp / 5000.0;
    double outpost_indicator = (bb.our_outpost_hp == 0) ? 1.0 : 0.0;
    double distance_score = 1.0 - std::min(distance_to_fortress / 2000.0, 1.0);  // 20m = 2000cm
    
    return 0.30 * hp_score + 0.30 * base_score + 0.30 * outpost_indicator + 0.10 * distance_score;
}

double Models::calculateHeroAttackValue(const Blackboard& bb, double distance_to_hero, bool deploy_state) {
    if (!bb.enemy_hero.visible || bb.enemy_hero.hp <= 0) return 0.0;
    
    double hp_score = 1.0 - (bb.enemy_hero.hp / 450.0);
    double distance_score = 1.0 - std::min(distance_to_hero / 1500.0, 1.0);  // 15m = 1500cm
    double deploy_score = deploy_state ? 1.0 : 0.0;
    
    return 0.50 * hp_score + 0.35 * distance_score + 0.15 * deploy_score;
}

std::vector<Models::GainPointScore> Models::calculateGainPointScores(const Blackboard& bb) {
    std::vector<GainPointScore> scores;
    double hp_ratio = bb.current_hp / 400.0;
    
    // 遍历所有增益点
    for (const auto& gp : bb.gain_points) {
        // 只考虑未被占领的增益点
        if (gp.neutral) {
            double distance = calculateDistance(bb.x, bb.y, gp.position.x, gp.position.y);
            double distance_score = 1.0 - std::min(distance / 1500.0, 1.0);  // 15m = 1500cm
            double score = 0.60 * gp.defense_gain * (1.0 - hp_ratio) + 0.40 * distance_score;
            scores.push_back({gp.name, gp.position, score, gp.defense_gain});
        }
    }
    
    // 按分数从高到低排序
    std::sort(scores.begin(), scores.end(), 
        [](const GainPointScore& a, const GainPointScore& b) {
            return a.score > b.score;
        });
    
    return scores;
}

double Models::calculateDistance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double Models::getNearestEnemyDistance(const Blackboard& bb) {
    double min_distance = 100000.0;  // 很大的初始值
    
    auto checkDistance = [&](const EnemyInfo& enemy) {
        if (enemy.visible && enemy.hp > 0) {
            double distance = calculateDistance(bb.x, bb.y, enemy.x, enemy.y);
            if (distance < min_distance) {
                min_distance = distance;
            }
        }
    };
    
    checkDistance(bb.enemy_hero);
    checkDistance(bb.enemy_engineer);
    checkDistance(bb.enemy_infantry3);
    checkDistance(bb.enemy_infantry4);
    checkDistance(bb.enemy_sentry);
    
    return min_distance;
}

// 修改 Models::needRamp 函数
bool Models::needRamp(const Blackboard& bb, const geometry_msgs::msg::Point& target) {
    // 如果飞坡锁已激活，返回false
    if (bb.isRampLockActive() || bb.isRampLockPending()) {
        return false;
    }
    
    // 如果目标点在敌方半区（x >= 1400），而自身在己方半区（x < 1400）
    if (target.x >= HALF_MAP_X && bb.x < HALF_MAP_X) {
        return true;
    }
    return false;
}

double Models::getTypeWeight(const std::string& robot_type) {
    auto it = ROBOT_TYPE_WEIGHTS.find(robot_type);
    if (it != ROBOT_TYPE_WEIGHTS.end()) {
        return it->second;
    }
    return 1.0;  // 默认权重
}

double Models::getMaxHP(const std::string& robot_type) {
    auto it = ROBOT_MAX_HP.find(robot_type);
    if (it != ROBOT_MAX_HP.end()) {
        return it->second;
    }
    return 400.0;  // 默认最大血量
}

double Models::getMaxAmmo(const std::string& robot_type) {
    auto it = ROBOT_MAX_AMMO.find(robot_type);
    if (it != ROBOT_MAX_AMMO.end()) {
        return it->second;
    }
    return 100.0;  // 默认最大弹量
}
