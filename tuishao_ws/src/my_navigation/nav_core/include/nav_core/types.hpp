// nav_core/include/nav_core/types.hpp
// 导航系统核心类型定义

#pragma once
#include <cstdint>

namespace nav_core {

// 主状态机状态
enum class NavState : uint8_t {
    IDLE = 0,        // 空闲
    PLANNING,        // 规划中
    CONTROLLING,     // 控制中
    ESCAPING,        // 脱困中（起点在障碍物中，强制发送速度指令）
    RECOVERY,        // 恢复中
    SUCCEEDED,       // 成功
    FAILED           // 失败
};

// 恢复触发原因
enum class RecoveryTrigger : uint8_t {
    NONE = 0,
    PLANNING_FAILED,    // 规划失败
    CONTROL_FAILED,     // 控制失败
    STUCK,              // 卡住
    TIMEOUT             // 超时
};

// 恢复行为状态
enum class RecoveryStatus : uint8_t {
    IDLE = 0,
    RUNNING,
    SUCCEEDED,
    FAILED
};

inline const char* toString(NavState s) {
    switch(s) {
        case NavState::IDLE: return "IDLE";
        case NavState::PLANNING: return "PLANNING";
        case NavState::CONTROLLING: return "CONTROLLING";
        case NavState::ESCAPING: return "ESCAPING";
        case NavState::RECOVERY: return "RECOVERY";
        case NavState::SUCCEEDED: return "SUCCEEDED";
        case NavState::FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}

}  // namespace nav_core
