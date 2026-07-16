#ifndef GAME_CONSTANTS_HPP
#define GAME_CONSTANTS_HPP

#include <cstdint>

namespace GameConstants {
    const uint8_t STAGE_BATTLE = 4;

    const uint8_t GIMBAL_IDLE = 0;
    const uint8_t GIMBAL_ENEMY = 1;
    const uint8_t GIMBAL_OUTPOST = 2;

    const uint8_t SPIN_OFF = 0;
    const uint8_t SPIN_HIGH = 1;
    const uint8_t SPIN_LOW = 2;

    const uint8_t POSTURE_ATTACK = 1;
    const uint8_t POSTURE_DEFENSE = 2;
    const uint8_t POSTURE_MOVE = 3;

    const uint8_t RED_SENTRY_ROBOT_ID = 7;
    const uint8_t BLUE_SENTRY_ROBOT_ID = 107;

    inline bool isBlueRobotId(int robot_id) {
        return robot_id == static_cast<int>(BLUE_SENTRY_ROBOT_ID);
    }
}

#endif
