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

// 包含自定义消息类型
#include "sentry_decision/msg/sentry_control.hpp"

using OurRobotState = decision_messages::msg::OurRobotState;
using EnemyRobotState = decision_messages::msg::EnemyRobotState;
using GameState = decision_messages::msg::GameState;
using SentryControl = sentry_decision::msg::SentryControl;

// 行为状态枚举
enum class BehaviorState {
    IDLE,               // 空闲
    MOVING,             // 移动中
    EXECUTING,          // 执行中（到达目标点后）
    COMPLETED           // 行为完成
};

// 行为类型枚举
enum class BehaviorType {
    NONE,
    MOVE_TO_ENERGY,     // 前往能量机关
    ACTIVATE_ENERGY,    // 激活能量机关
    MOVE_TO_OUTPOST,    // 前往前哨站
    ATTACK_OUTPOST,     // 攻击前哨站
    MOVE_TO_SUPPLY,     // 前往补给点
    SUPPLY,             // 补给
    MOVE_TO_GAIN_POINT, // 前往增益点
    DEFEND_GAIN_POINT,  // 防守增益点
    MOVE_TO_HERO,       // 前往攻击英雄
    ATTACK_HERO,        // 攻击英雄
    MOVE_TO_BASE,       // 前往基地
    DEFEND_BASE,        // 防守基地
    MOVE_TO_FORTRESS,   // 前往堡垒
    OCCUPY_FORTRESS,    // 占领堡垒
    RESURRECTION,       // 复活
    RAMP_PROCESS        // 飞坡流程（新增加）
};

struct EnemyInfo {
    std::string id;
    std::string type;
    double x = 0.0;          // cm
    double y = 0.0;          // cm
    double hp = 0.0;         // 血量
    double allowance = 0.0;  // 弹量
    bool visible = false;    // 是否可见
    double last_update_time = 0.0;
    
    EnemyInfo(const std::string& id_val, const std::string& type_val) 
        : id(id_val), type(type_val) {}
};

struct GainPointStatus {
    std::string name;
    geometry_msgs::msg::Point position;  // cm
    double defense_gain = 0.0;
    bool occupied_by_us = false;
    bool occupied_by_enemy = false;
    bool neutral = true;
};

struct BehaviorInfo {
    BehaviorType type = BehaviorType::NONE;
    BehaviorState state = BehaviorState::IDLE;
    geometry_msgs::msg::Point target;
    double start_time = -1.0;
    double execution_start_time = -1.0;  // 执行开始时间（到达目标点后）
    double execution_duration = 0.0;     // 执行持续时间
    bool target_published = false;       // 目标点是否已发布
    bool control_published = false;      // 控制消息是否已发布
    bool control_updated = false;        // 控制消息是否已更新
};

class Blackboard {
public:
    Blackboard();
    
    // ROS消息更新接口
    void updateOurState(const OurRobotState::SharedPtr msg);
    void updateEnemyState(const EnemyRobotState::SharedPtr msg);
    void updateGameState(const GameState::SharedPtr msg);
    
    // 重置函数（用于比赛重新开始）
    void resetForNewMatch();
    
    // 获取控制消息
    std::shared_ptr<SentryControl> getControlMsg() const { return control_msg_; }
    
    // 自身状态
    double current_hp = 400.0;
    double max_hp = 400.0;
    double allowance_17mm = 300.0;
    double x = 0.0;  // cm
    double y = 0.0;  // cm
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
    
    // 飞坡相关 - 修改
    geometry_msgs::msg::Point original_target_before_ramp;
    bool at_ramp_point = false;
    bool ramp_mode_active = false;
    bool ramp_in_process = false;  // 新增：是否在飞坡流程中
    
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
    double min_stay_duration = 3.0;  // 最小停留时间（秒）
    
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
    
    // 目标点管理
    void setTargetReached(bool reached);
    bool shouldLeaveTarget() const;
    
    // 行为管理
    void startBehavior(BehaviorType type, const geometry_msgs::msg::Point& target, double duration = 0.0);
    void updateBehaviorState(BehaviorState state);
    void startExecutionTime();  // 开始执行计时（到达目标点时调用）
    void completeCurrentBehavior();
    void resetCurrentBehavior();
    
    bool isBehaviorInProgress() const { 
        return current_behavior.state != BehaviorState::IDLE && 
               current_behavior.state != BehaviorState::COMPLETED; 
    }
    
    // 检查是否在飞坡过程中
    bool isInRampProcess() const {
        return ramp_in_process;
    }
    
    // 获取行为执行已用时间（从到达目标点开始）
    double getExecutionElapsedTime() const {
        if (current_behavior.execution_start_time < 0) {
            return 0.0;
        }
        return current_time - current_behavior.execution_start_time;
    }
    
    // 设置目标点发布状态
    void setTargetPublished(bool published) { 
        current_behavior.target_published = published; 
        target_published_ = published;
    }
    bool isTargetPublished() const { return current_behavior.target_published; }
    
    // 设置控制消息发布状态
    void setControlPublished(bool published) { 
        current_behavior.control_published = published; 
        control_published_ = published;
    }
    bool isControlPublished() const { return current_behavior.control_published; }
    
    // 设置控制消息更新状态
    void setControlUpdated(bool updated) { 
        current_behavior.control_updated = updated; 
        control_updated_ = updated;
    }
    bool isControlUpdated() const { return current_behavior.control_updated; }
    
    // 重置所有发布状态
    void resetAllPublishStates() {
        setTargetPublished(false);
        setControlPublished(false);
        setControlUpdated(false);
    }
    
    // 飞坡锁相关方法（新增）
    enum RampLockState {
        RAMP_LOCK_INACTIVE = 0,      // 飞坡锁未激活
        RAMP_LOCK_PENDING,           // 飞坡判定已触发，等待执行
        RAMP_LOCK_ACTIVE,            // 飞坡行为执行中
        RAMP_LOCK_COMPLETING         // 飞坡即将完成，但仍需锁定
    };
    
    void activateRampLock() { 
        ramp_lock_state_ = RAMP_LOCK_PENDING; 
        std::cout << "[RAMP_LOCK] 激活飞坡锁: PENDING" << std::endl;
    }
    
    void setRampLockActive() { 
        ramp_lock_state_ = RAMP_LOCK_ACTIVE; 
        std::cout << "[RAMP_LOCK] 设置飞坡锁: ACTIVE" << std::endl;
    }
    
    void setRampLockCompleting() { 
        ramp_lock_state_ = RAMP_LOCK_COMPLETING; 
        std::cout << "[RAMP_LOCK] 设置飞坡锁: COMPLETING" << std::endl;
    }
    
    void deactivateRampLock() { 
        ramp_lock_state_ = RAMP_LOCK_INACTIVE; 
        std::cout << "[RAMP_LOCK] 解除飞坡锁: INACTIVE" << std::endl;
    }
    
    bool isRampLockActive() const { 
        return ramp_lock_state_ == RAMP_LOCK_ACTIVE || 
               ramp_lock_state_ == RAMP_LOCK_COMPLETING; 
    }
    
    bool isRampLockPending() const { 
        return ramp_lock_state_ == RAMP_LOCK_PENDING; 
    }
    
    bool isRampLockInactive() const { 
        return ramp_lock_state_ == RAMP_LOCK_INACTIVE; 
    }
    
    RampLockState getRampLockState() const { return ramp_lock_state_; }
    
private:
    void updateEnemyInfo(EnemyInfo& info, double x, double y, double hp, double allowance);
    void initializeGainPoints();
    void updateGainPointStatus();
    
    // 控制消息
    std::shared_ptr<SentryControl> control_msg_;
    
    // 发布状态
    bool target_published_ = false;
    bool control_published_ = false;
    bool control_updated_ = false;
    
    // 比赛状态跟踪
    uint8_t last_stage_ = 0;
    
    // 飞坡锁状态（新增）
    RampLockState ramp_lock_state_ = RAMP_LOCK_INACTIVE;
};

#endif // BLACKBOARD_HPP
