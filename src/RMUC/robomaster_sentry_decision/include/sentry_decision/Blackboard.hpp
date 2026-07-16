#ifndef BLACKBOARD_HPP
#define BLACKBOARD_HPP

#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <geometry_msgs/msg/point.hpp>
#include "decision_messages/msg/our_robot_state.hpp"
#include "decision_messages/msg/enemy_robot_state.hpp"
#include "decision_messages/msg/game_state.hpp"
#include "sentry_decision/msg/sentry_control.hpp"
#include "GameConstants.hpp"

using OurRobotState = decision_messages::msg::OurRobotState;
using EnemyRobotState = decision_messages::msg::EnemyRobotState;
using GameState = decision_messages::msg::GameState;
using SentryControl = sentry_decision::msg::SentryControl;

struct EnemyInfo {
    std::string id;
    std::string type;
    double x = 0.0, y = 0.0;
    double hp = 0.0;
    double allowance = 0.0;
    bool visible = false;
    double last_update_time = 0.0;
    EnemyInfo(const std::string& id_val, const std::string& type_val)
        : id(id_val), type(type_val) {}
};

class Blackboard {
public:
    Blackboard();
    bool loadConfigFromYAML(const std::string& filepath);

    // 坐标获取
    geometry_msgs::msg::Point getPatrolPointA() const;
    geometry_msgs::msg::Point getPatrolPointB() const;
    geometry_msgs::msg::Point getPatrolPointC() const;
    geometry_msgs::msg::Point getProtectHeroPoint() const;
    geometry_msgs::msg::Point getSupplyPoint() const;
    geometry_msgs::msg::Point getInitialOutpostPoint() const;

    // 参数获取
    double getArrivalWaitTime() const;
    double getDeviationThreshold() const;
    double getPatrolStayDurationA() const;
    double getPatrolStayDurationB() const;
    double getPatrolStayDurationC() const;
    double getAttackDuration() const;
    double getDefendDuration() const;
    double getInitialOutpostEndRemainingTime() const;
    double getSupplyThreshold() const;
    double getMaxHp() const;
    double getMaxAmmo() const;

    // 状态更新
    void updateOurState(const OurRobotState::SharedPtr msg);
    void updateEnemyState(const EnemyRobotState::SharedPtr msg);
    void updateGameState(const GameState::SharedPtr msg);
    void updatePositionFromTF(double x_m, double y_m, double yaw_rad);
    void resetForNewMatch();

    // 公共成员
    double current_hp = 400.0;
    double allowance_17mm = 300.0;
    double x = 0.0, y = 0.0;
    double robot_yaw = 0.0;
    double stage_remaining_time = 420.0;
    double current_time = 0.0;
    uint8_t stage = 0;
    uint8_t robot_id_ = 0;

    bool resurrection_flag = false;
    // 我方英雄坐标（cm）
    double our_hero_x = 0.0;
    double our_hero_y = 0.0;
    bool hero_in_deploy_zone = false;

    // 敌方信息
    EnemyInfo enemy_hero;
    EnemyInfo enemy_engineer;
    EnemyInfo enemy_infantry3;
    EnemyInfo enemy_infantry4;
    EnemyInfo enemy_sentry;

    // 游戏状态
    int8_t base_open = 0;
    int8_t outpost_alive = 1;

    // 目标到达检测
    bool at_current_target = false;
    double target_arrival_time = -1.0;

    // 行为管理
    struct BehaviorInfo {
        geometry_msgs::msg::Point target;
        double start_time = -1.0;
        bool target_published = false;
        bool control_published = false;
    } current_behavior;

    bool isAtTarget(const geometry_msgs::msg::Point& target, double tolerance = 50.0) const;
    void setTargetReached(bool reached);
    bool hasWaitedAtTarget(double wait_seconds) const;

    void updateControlMsg(uint8_t gimbal_mode, uint8_t spin_mode, uint8_t posture);
    uint8_t getBattleSpinMode() const;
    std::shared_ptr<SentryControl> getControlMsg() const { return control_msg_; }
    void resetAllPublishStates();

    EnemyInfo* getEnemyById(const std::string& id);
    const EnemyInfo* getEnemyById(const std::string& id) const;

private:
    struct Config {
        geometry_msgs::msg::Point red_patrol_A, red_patrol_B, red_patrol_C;
        geometry_msgs::msg::Point blue_patrol_A, blue_patrol_B, blue_patrol_C;
        geometry_msgs::msg::Point red_protect_hero, blue_protect_hero;
        geometry_msgs::msg::Point red_supply, blue_supply;
        geometry_msgs::msg::Point red_initial_outpost, blue_initial_outpost;

        double arrival_wait_time;
        double deviation_threshold;
        double patrol_A_stay_duration;
        double patrol_B_stay_duration;
        double patrol_C_stay_duration;
        double attack_duration;
        double defend_duration;
        double initial_outpost_end_remaining_time;
        double supply_threshold;
        double max_hp;
        double max_ammo;
    } config_;

    std::shared_ptr<SentryControl> control_msg_;
    uint8_t last_stage_ = 0;
    bool under_attack_ = false;
    std::deque<std::pair<double, double>> hp_history_;

    void onRobotIdChanged(uint8_t old_id, uint8_t new_id);
};

#endif
