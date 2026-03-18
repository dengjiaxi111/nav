#ifndef BLACKBOARD_HPP
#define BLACKBOARD_HPP

#include <memory>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>

// ROS2消息类型
#include "decision_messages/msg/our_robot_state.hpp"
#include "decision_messages/msg/enemy_robot_state.hpp"
#include "decision_messages/msg/game_state.hpp"
#include "sentry_decision/msg/sentry_control.hpp"

using OurRobotState = decision_messages::msg::OurRobotState;
using EnemyRobotState = decision_messages::msg::EnemyRobotState;
using GameState = decision_messages::msg::GameState;
using SentryControl = sentry_decision::msg::SentryControl;

// 行为状态枚举
enum class BehaviorState {
    IDLE, MOVING, EXECUTING, COMPLETED
};

// 行为类型枚举
enum class BehaviorType {
    NONE, MOVE_TO_ENERGY, ACTIVATE_ENERGY, MOVE_TO_OUTPOST, ATTACK_OUTPOST,
    MOVE_TO_SUPPLY, SUPPLY, MOVE_TO_GAIN_POINT, DEFEND_GAIN_POINT,
    MOVE_TO_HERO, ATTACK_HERO, MOVE_TO_BASE, DEFEND_BASE,
    MOVE_TO_FORTRESS, OCCUPY_FORTRESS, RESURRECTION, RAMP_PROCESS
};

struct EnemyInfo {
    std::string id;
    std::string type;
    double x = 0.0;
    double y = 0.0;
    double hp = 0.0;
    double allowance = 0.0;
    bool visible = false;
    double last_update_time = 0.0;
    EnemyInfo(const std::string& id_val, const std::string& type_val) : id(id_val), type(type_val) {}
};

struct GainPointStatus {
    std::string name;
    geometry_msgs::msg::Point position;
    double defense_gain = 0.0;
    bool occupied_by_us = false;
    bool occupied_by_enemy = false;
    bool neutral = true;
};

struct BehaviorInfo {
    BehaviorType type = BehaviorType::NONE;
    BehaviorState state = BehaviorState::IDLE;
    geometry_msgs::msg::Point target;          // 原始目标点（用于发布）
    geometry_msgs::msg::Point real_target;     // 真实目标点（用于内部计算）
    double start_time = -1.0;
    double execution_start_time = -1.0;
    double execution_duration = 0.0;
    bool target_published = false;
    bool control_published = false;
    bool control_updated = false;
};

class Blackboard {
public:
    Blackboard();
    
    void updateOurState(const OurRobotState::SharedPtr msg);
    void updateEnemyState(const EnemyRobotState::SharedPtr msg);
    void updateGameState(const GameState::SharedPtr msg);
    void resetForNewMatch();
    std::shared_ptr<SentryControl> getControlMsg() const { return control_msg_; }
    
    // 自身状态
    double current_hp = 400.0;
    double max_hp = 400.0;
    double allowance_17mm = 300.0;
    double x = 0.0;
    double y = 0.0;
    uint32_t rfid_status = 0;
    
    // 全局状态
    double our_base_hp = 5000.0;
    double our_outpost_hp = 1500.0;
    uint8_t stage = 0;
    double stage_remaining_time = 420.0;
    double current_time = 0.0;
    bool energy_mechanism_activatable = false;
    uint8_t large_energy_mechanism_activation = 0;
    
    // 增益点状态
    bool base_gain_point_occupied = false;
    bool trapezoid_highland_occupied = false;
    bool fortress_gain_point_occupied_by_us = false;
    bool fortress_gain_point_occupied_by_enemy = false;
    bool central_highland_occupied_by_us = false;
    bool central_highland_occupied_by_enemy = false;
    bool outpost_gain_point_occupied_by_us = false;
    bool outpost_gain_point_occupied_by_enemy = false;
    
    // 敌人信息
    EnemyInfo enemy_hero;
    EnemyInfo enemy_engineer;
    EnemyInfo enemy_infantry3;
    EnemyInfo enemy_infantry4;
    EnemyInfo enemy_sentry;
    
    // 决策输出
    geometry_msgs::msg::Point target_position;
    std::string current_target_id;
    double attack_start_time = -1.0;
    
    // 飞坡相关
    geometry_msgs::msg::Point original_target_before_ramp;
    bool at_ramp_point = false;
    bool ramp_mode_active = false;
    bool ramp_in_process = false;
    
    // 状态标志
    bool resurrection_flag = false;
    bool initialization_complete = false;
    bool energy_activated = false;
    bool outpost_destroyed_init = false;
    bool at_supply_point = false;
    bool supply_rfid_detected = false;
    bool hero_in_deploy_zone = false;
    
    // 时间跟踪
    double supply_start_time = -1.0;
    double hero_last_x = 0.0;
    double hero_last_y = 0.0;
    double hero_static_start_time = 0.0;
    double hero_last_update_time = 0.0;
    
    // 目标点到达状态
    bool at_current_target = false;
    double target_arrival_time = -1.0;
    double min_stay_duration = 3.0;
    
    // 增益点列表
    std::vector<GainPointStatus> gain_points;
    
    // 行为管理
    BehaviorInfo current_behavior;
    
    // 获取敌人信息
    EnemyInfo* getEnemyById(const std::string& id);
    const EnemyInfo* getEnemyById(const std::string& id) const;
    std::vector<EnemyInfo*> getVisibleEnemies();
    
    // 工具函数
    bool checkRFIDBit(int bit_pos) const;
    bool isAtTarget(const geometry_msgs::msg::Point& target, double tolerance = 50.0) const;
    void updateControlMsg(uint8_t gimbal_mode, uint8_t spin_mode, 
                         uint8_t posture, uint8_t ramp_mode);
    
    void setTargetReached(bool reached);
    bool shouldLeaveTarget() const;
    
    void startBehavior(BehaviorType type, const geometry_msgs::msg::Point& target, double duration = 0.0);
    void updateBehaviorState(BehaviorState state);
    void startExecutionTime();
    void completeCurrentBehavior();
    void resetCurrentBehavior();
    
    bool isBehaviorInProgress() const { 
        return current_behavior.state != BehaviorState::IDLE && 
               current_behavior.state != BehaviorState::COMPLETED; 
    }
    
    bool isInRampProcess() const { return ramp_in_process; }
    
    double getExecutionElapsedTime() const {
        if (current_behavior.execution_start_time < 0) return 0.0;
        return current_time - current_behavior.execution_start_time;
    }
    
    void setTargetPublished(bool published) { current_behavior.target_published = published; }
    bool isTargetPublished() const { return current_behavior.target_published; }
    
    void setControlPublished(bool published) { current_behavior.control_published = published; }
    bool isControlPublished() const { return current_behavior.control_published; }
    
    void setControlUpdated(bool updated) { current_behavior.control_updated = updated; }
    bool isControlUpdated() const { return current_behavior.control_updated; }
    
    void resetAllPublishStates() {
        setTargetPublished(false);
        setControlPublished(false);
        setControlUpdated(false);
    }
    
    // 飞坡锁相关
    enum RampLockState {
        RAMP_LOCK_INACTIVE = 0,
        RAMP_LOCK_PENDING,
        RAMP_LOCK_ACTIVE,
        RAMP_LOCK_COMPLETING
    };
    
    void activateRampLock() { ramp_lock_state_ = RAMP_LOCK_PENDING; }
    void setRampLockActive() { ramp_lock_state_ = RAMP_LOCK_ACTIVE; }
    void setRampLockCompleting() { ramp_lock_state_ = RAMP_LOCK_COMPLETING; }
    void deactivateRampLock() { ramp_lock_state_ = RAMP_LOCK_INACTIVE; }
    bool isRampLockActive() const { return ramp_lock_state_ == RAMP_LOCK_ACTIVE || ramp_lock_state_ == RAMP_LOCK_COMPLETING; }
    bool isRampLockPending() const { return ramp_lock_state_ == RAMP_LOCK_PENDING; }
    bool isRampLockInactive() const { return ramp_lock_state_ == RAMP_LOCK_INACTIVE; }
    RampLockState getRampLockState() const { return ramp_lock_state_; }
    
    // 新增：获取原始目标点（用于发布）
    geometry_msgs::msg::Point getOriginalTarget() const { return current_behavior.target; }

private:
    void updateEnemyInfo(EnemyInfo& info, double x, double y, double hp, double allowance);
    void initializeGainPoints();
    void updateGainPointStatus();
    
    // 新增：逻辑坐标到真实坐标的转换
    geometry_msgs::msg::Point convertToReal(const geometry_msgs::msg::Point& logical) const;
    
    std::shared_ptr<SentryControl> control_msg_;
    bool target_published_ = false;
    bool control_published_ = false;
    bool control_updated_ = false;
    uint8_t last_stage_ = 0;
    RampLockState ramp_lock_state_ = RAMP_LOCK_INACTIVE;
    
    // 新增：逻辑坐标到真实坐标的映射表
    static std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> LOGICAL_TO_REAL_MAP;
};

#endif // BLACKBOARD_HPP
