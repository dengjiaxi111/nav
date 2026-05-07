#include "sentry_decision/Models.hpp"
#include <cmath>
#include <algorithm>

double Models::calculateSituationZ(const Blackboard& bb) {
    double hp_ratio = bb.current_hp / bb.getMaxHp();
    double ammo_ratio = bb.allowance_17mm / bb.getMaxAmmo();
    double base_ratio = bb.our_base_hp / 5000.0;
    return bb.getHpWeight() * hp_ratio + bb.getAmmoWeight() * ammo_ratio + bb.getBaseWeight() * base_ratio;
}

double Models::calculateHeroAttackValue(const Blackboard& bb, double distance_to_hero, bool deploy_state) {
    if (!bb.enemy_hero.visible || bb.enemy_hero.hp <= 0) return 0.0;
    double hp_score = 1.0 - (bb.enemy_hero.hp / 450.0);
    double distance_score = 1.0 - std::min(distance_to_hero / 1500.0, 1.0);
    double deploy_score = deploy_state ? 1.0 : 0.0;
    return 0.50 * hp_score + 0.35 * distance_score + 0.15 * deploy_score;
}

double Models::calculateGeneralTargetScore(const Blackboard& bb, const EnemyInfo& enemy,
                                           double weight_hp, double weight_distance, double weight_ammo) {
    double hp_ratio = 1.0 - (enemy.hp / getMaxHP(enemy.type)); // 血量越低分数越高
    double distance = calculateDistance(bb.x, bb.y, enemy.x, enemy.y);
    double distance_score = 1.0 - std::min(distance / 2800.0, 1.0);
    double ammo_ratio = 1.0 - (enemy.allowance / getMaxAmmo(enemy.type));
    return weight_hp * hp_ratio + weight_distance * distance_score + weight_ammo * ammo_ratio;
}

std::vector<Models::TargetScore> Models::calculateAllTargetScores(const Blackboard& bb) {
    std::vector<TargetScore> scores;
    auto addScore = [&](const EnemyInfo& enemy, const std::string& type) {
        double distance = calculateDistance(bb.x, bb.y, enemy.x, enemy.y);
        double hp_score = 1.0 - (enemy.hp / getMaxHP(type));
        double distance_score = 1.0 - std::min(distance / 2800.0, 1.0);
        double type_weight = getTypeWeight(type) / 1.6;
        double ammo_score = 1.0 - (enemy.allowance / getMaxAmmo(type));
        double total = 0.35 * hp_score + 0.30 * distance_score + 0.25 * type_weight + 0.10 * ammo_score;
        scores.push_back({enemy.id, type, total});
    };
    if (bb.enemy_hero.visible && bb.enemy_hero.hp > 0) addScore(bb.enemy_hero, "hero");
    if (bb.enemy_engineer.visible && bb.enemy_engineer.hp > 0) addScore(bb.enemy_engineer, "engineer");
    if (bb.enemy_infantry3.visible && bb.enemy_infantry3.hp > 0) addScore(bb.enemy_infantry3, "infantry");
    if (bb.enemy_infantry4.visible && bb.enemy_infantry4.hp > 0) addScore(bb.enemy_infantry4, "infantry");
    if (bb.enemy_sentry.visible && bb.enemy_sentry.hp > 0) addScore(bb.enemy_sentry, "sentry");
    std::sort(scores.begin(), scores.end(), [](const TargetScore& a, const TargetScore& b) { return a.score > b.score; });
    return scores;
}

std::vector<Models::GainPointScore> Models::calculateGainPointScores(const Blackboard& bb) {
    std::vector<GainPointScore> scores;
    double hp_ratio = bb.current_hp / bb.getMaxHp();
    for (const auto& gp : bb.gain_points) {
        if (gp.neutral) {
            double distance = calculateDistance(bb.x, bb.y, gp.position.x, gp.position.y);
            double distance_score = 1.0 - std::min(distance / 1500.0, 1.0);
            double score = 0.60 * gp.defense_gain * (1.0 - hp_ratio) + 0.40 * distance_score;
            scores.push_back({gp.name, gp.position, score, gp.defense_gain});
        }
    }
    std::sort(scores.begin(), scores.end(), [](const GainPointScore& a, const GainPointScore& b) { return a.score > b.score; });
    return scores;
}

double Models::calculateFortressValue(const Blackboard& bb, double distance_to_fortress) {
    double hp_score = bb.current_hp / bb.getMaxHp();
    double base_score = bb.our_base_hp / 5000.0;
    double outpost_indicator = (bb.our_outpost_hp == 0) ? 1.0 : 0.0;
    double distance_score = 1.0 - std::min(distance_to_fortress / 2000.0, 1.0);
    return 0.30 * hp_score + 0.30 * base_score + 0.30 * outpost_indicator + 0.10 * distance_score;
}

double Models::calculateDistance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1, dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

double Models::getNearestEnemyDistance(const Blackboard& bb) {
    double min_dist = 1e9;
    auto check = [&](const EnemyInfo& e) {
        if (e.visible && e.hp > 0) min_dist = std::min(min_dist, calculateDistance(bb.x, bb.y, e.x, e.y));
    };
    check(bb.enemy_hero);
    check(bb.enemy_engineer);
    check(bb.enemy_infantry3);
    check(bb.enemy_infantry4);
    check(bb.enemy_sentry);
    return min_dist;
}

double Models::getTypeWeight(const std::string& robot_type) {
    static const std::map<std::string, double> weights = {{"hero",1.6},{"sentry",1.3},{"infantry",1.0},{"engineer",0.7}};
    auto it = weights.find(robot_type);
    return (it != weights.end()) ? it->second : 1.0;
}

double Models::getMaxHP(const std::string& robot_type) {
    static const std::map<std::string, double> max_hp = {{"hero",450.0},{"engineer",400.0},{"infantry",250.0},{"sentry",400.0}};
    auto it = max_hp.find(robot_type);
    return (it != max_hp.end()) ? it->second : 400.0;
}

double Models::getMaxAmmo(const std::string& robot_type) {
    static const std::map<std::string, double> max_ammo = {{"hero",100.0},{"sentry",300.0},{"infantry",200.0},{"engineer",50.0}};
    auto it = max_ammo.find(robot_type);
    return (it != max_ammo.end()) ? it->second : 100.0;
}
