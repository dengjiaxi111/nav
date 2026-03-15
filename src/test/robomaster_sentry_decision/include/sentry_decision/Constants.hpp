#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <geometry_msgs/msg/point.hpp>
#include <map>
#include <string>

namespace SentryConstants {
    // 辅助函数创建点（必须先声明）
    inline geometry_msgs::msg::Point createPoint(double x, double y) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.0;
        return p;
    }
    
    // 坐标点定义（单位：厘米）
    const geometry_msgs::msg::Point RED_FORTRESS = createPoint(690.0, 746.0);
    const geometry_msgs::msg::Point RED_RAMP_POINT = createPoint(1070.0, 94.0);
    const geometry_msgs::msg::Point RED_OUTPOST_GAIN = createPoint(1172.0, 378.0);
    const geometry_msgs::msg::Point RED_ENERGY_POINT = createPoint(1152.0, 992.0);
    const geometry_msgs::msg::Point RED_SUPPLY = createPoint(176.0, 262.0);
    const geometry_msgs::msg::Point RED_BASE_GAIN = createPoint(412.0, 754.0);
    const geometry_msgs::msg::Point ENEMY_FORTRESS_OCCUPY = createPoint(2118.0, 744.0);
    const geometry_msgs::msg::Point ENEMY_OUTPOST_ATTACK = createPoint(1642.0, 1146.0);
    const geometry_msgs::msg::Point ENEMY_BASE_ATTACK = createPoint(2744.0, 1164.0);
    const geometry_msgs::msg::Point CENTRAL_HIGHLAND_GAIN = createPoint(1220.0, 1092.0);
    const geometry_msgs::msg::Point TRAPEZOID_HIGHLAND_GAIN = createPoint(430.0, 1108.0);
    
    // 区域分界线
    const double HALF_MAP_X = 1400.0;
    
    // 机器人类型权重
    const std::map<std::string, double> ROBOT_TYPE_WEIGHTS = {
        {"hero", 1.6},
        {"sentry", 1.3},
        {"infantry", 1.0},
        {"engineer", 0.7}
    };
    
    // 机器人最大血量
    const std::map<std::string, double> ROBOT_MAX_HP = {
        {"hero", 450.0},
        {"engineer", 400.0},
        {"infantry", 250.0},
        {"sentry", 400.0}
    };
    
    // 机器人最大弹量
    const std::map<std::string, double> ROBOT_MAX_AMMO = {
        {"hero", 100.0},
        {"sentry", 300.0},
        {"infantry", 200.0},
        {"engineer", 50.0}
    };
    
    // 云台模式
    const uint8_t GIMBAL_ENERGY = 0;    // 打符
    const uint8_t GIMBAL_ENEMY = 1;     // 打人
    const uint8_t GIMBAL_OUTPOST = 2;   // 打前哨站
    const uint8_t GIMBAL_IDLE = 3;      // 不动
    
    // 小陀螺模式
    const uint8_t SPIN_OFF = 0;         // 不动
    const uint8_t SPIN_LOW = 1;         // 低速转
    const uint8_t SPIN_VARIABLE = 2;    // 变速转
    const uint8_t SPIN_HIGH = 3;        // 高速转
    
    // 姿态
    const uint8_t POSTURE_ATTACK = 1;   // 进攻姿态
    const uint8_t POSTURE_DEFENSE = 2;  // 防御姿态
    const uint8_t POSTURE_MOVE = 3;     // 移动姿态
    
    // 飞坡模式
    const uint8_t RAMP_OFF = 0;         // 不飞坡
    const uint8_t RAMP_ON = 1;          // 飞坡
    
    // 比赛阶段定义
    const uint8_t STAGE_PREPARATION = 0;    // 准备阶段
    const uint8_t STAGE_SELF_CHECK = 1;     // 自检阶段
    const uint8_t STAGE_COUNTDOWN = 2;      // 倒计时
    const uint8_t STAGE_FREE = 3;           // 自由时间
    const uint8_t STAGE_BATTLE = 4;         // 比赛中
    const uint8_t STAGE_SETTLEMENT = 5;     // 比赛结算
}

#endif // CONSTANTS_HPP
