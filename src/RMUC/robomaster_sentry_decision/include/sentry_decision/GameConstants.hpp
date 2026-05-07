#ifndef GAME_CONSTANTS_HPP
#define GAME_CONSTANTS_HPP

namespace GameConstants {
    // 比赛阶段
    const uint8_t STAGE_BATTLE = 4;

    // 云台模式: 按新要求映射
    const uint8_t GIMBAL_IDLE = 0;     // 不动
    const uint8_t GIMBAL_ENEMY = 1;    // 打人
    const uint8_t GIMBAL_OUTPOST = 2;  // 打前哨站

    // 小陀螺模式: 0 不转, 1 转动
    const uint8_t SPIN_OFF = 0;
    const uint8_t SPIN_ON = 1;

    // 姿态
    const uint8_t POSTURE_ATTACK = 1;
    const uint8_t POSTURE_DEFENSE = 2;
    const uint8_t POSTURE_MOVE = 3;

    // 飞坡已移除
}

#endif // GAME_CONSTANTS_HPP
