#ifndef GAME_CONSTANTS_HPP
#define GAME_CONSTANTS_HPP

namespace GameConstants {
    // 比赛阶段
    const uint8_t STAGE_BATTLE = 4;

    // 云台模式
    const uint8_t GIMBAL_ENERGY = 0;   // 打符（保留但不再使用）
    const uint8_t GIMBAL_ENEMY = 1;    // 打人
    const uint8_t GIMBAL_OUTPOST = 2;  // 打前哨站
    const uint8_t GIMBAL_IDLE = 3;     // 不动

    // 小陀螺模式
    const uint8_t SPIN_OFF = 0;
    const uint8_t SPIN_LOW = 1;
    const uint8_t SPIN_VARIABLE = 2;
    const uint8_t SPIN_HIGH = 3;

    // 姿态
    const uint8_t POSTURE_ATTACK = 1;
    const uint8_t POSTURE_DEFENSE = 2;
    const uint8_t POSTURE_MOVE = 3;

    // 飞坡模式
    const uint8_t RAMP_OFF = 0;
    const uint8_t RAMP_ON = 1;
}

#endif // GAME_CONSTANTS_HPP
