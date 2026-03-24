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
    const geometry_msgs::msg::Point RED_ATTACK_POINT = createPoint(602.9, 397.9);
    // 蓝方攻击点（robot_id=1）
    const geometry_msgs::msg::Point BLUE_ATTACK_POINT = createPoint(602.9, 397.9);  // 实际比赛中会替换为蓝方实际坐标

    // 红方补给点（robot_id=0）
    const geometry_msgs::msg::Point RED_SUPPLY_POINT = createPoint(79.7, 698.3);
    // 蓝方补给点（robot_id=1）
    const geometry_msgs::msg::Point BLUE_SUPPLY_POINT = createPoint(1126.3, 68.1);

    // 云台模式
    const uint8_t GIMBAL_ENEMY = 1;   // 打人

    // 底盘旋转模式（简化）
    const uint8_t SPIN_OFF = 0;  // 不转
    const uint8_t SPIN_ON  = 1;  // 旋转

    // 比赛阶段
    const uint8_t STAGE_BATTLE = 4;

    // 到达目标点后等待时间（秒）
    const double ARRIVAL_WAIT_TIME = 2.0;

    // 补给判断阈值（综合健康度低于此值则需要补给）
    const double SUPPLY_THRESHOLD = 0.7;
}

#endif // CONSTANTS_HPP
