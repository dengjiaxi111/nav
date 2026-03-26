#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <geometry_msgs/msg/point.hpp>

namespace SentryConstants {

    inline geometry_msgs::msg::Point createPoint(double x, double y) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.0;
        return p;
    }

    // 红方攻击点（robot_id=0）
    const geometry_msgs::msg::Point RED_ATTACK_POINT = createPoint(157.0, 31.5);
    // 蓝方攻击点（robot_id=1）
    const geometry_msgs::msg::Point BLUE_ATTACK_POINT = createPoint(602.9, 397.9);

    // 红方补给点（robot_id=0）
    const geometry_msgs::msg::Point RED_SUPPLY_POINT = createPoint(94.0, 257.0);
    // 蓝方补给点（robot_id=1）
    const geometry_msgs::msg::Point BLUE_SUPPLY_POINT = createPoint(1126.3, 68.1);

    // 云台模式（保留，但实际使用黑板的动态获取）
    const uint8_t GIMBAL_ENEMY = 1;   // 打人模式
    // 底盘旋转模式
    const uint8_t SPIN_OFF = 0;
    const uint8_t SPIN_ON  = 1;

    // 比赛阶段
    const uint8_t STAGE_BATTLE = 4;

    // 默认参数（会被 YAML 覆盖）
    const double ARRIVAL_WAIT_TIME = 1.0;
    const double SUPPLY_THRESHOLD = 0.3;
}

#endif // CONSTANTS_HPP
