// nav_core/include/nav_core/special_terrain_controller.hpp
// 特殊地形控制器抽象接口（阶段A：结构解耦）

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nav_core/map_interface.hpp"
#include "nav_core/types.hpp"

namespace nav_core {

enum class TerrainControlDecision {
    PASS_THROUGH = 0,
    OVERRIDE_CMD,
    REQUEST_REPLAN,
    REQUEST_RECOVERY,
    REQUEST_TEMP_GOAL
};

struct TerrainControlContext {
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::PoseStamped goal_pose;
    nav_msgs::msg::Path current_path;
    double control_rate_hz{20.0};
    double goal_tolerance{0.1};
    bool temporary_terrain_goal_active{false};
};

class SpecialTerrainController {
public:
    virtual ~SpecialTerrainController() = default;

    virtual void initialize(rclcpp::Node* node) = 0;
    virtual void setMap(MapInterface::Ptr map) { map_ = std::move(map); }

    virtual TerrainControlDecision update(
        const TerrainControlContext& context,
        const geometry_msgs::msg::Twist& base_cmd,
        geometry_msgs::msg::Twist& out_cmd) = 0;

    virtual bool consumeTemporaryGoal(geometry_msgs::msg::PoseStamped& goal) {
        (void)goal;
        return false;
    }

    virtual void onNavigationTaskStarted() {}
    virtual void onNavStateChanged(NavState state) = 0;
    virtual bool controlProgressTimeoutOverrideActive() const { return false; }
    virtual double controlProgressTimeoutSec() const { return 0.0; }

protected:
    MapInterface::Ptr map_;
};

}  // namespace nav_core
