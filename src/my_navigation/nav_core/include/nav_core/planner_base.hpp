// nav_core/include/nav_core/planner_base.hpp
// 规划器基类

#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include "nav_core/map_interface.hpp"

namespace nav_core {

class PlannerBase {
public:
    virtual ~PlannerBase() = default;
    
    virtual void initialize(rclcpp::Node* node) = 0;
    
    // 设置地图接口（统一接口，支持OccupancyGrid/ESDF/高程图等）
    virtual void setMap(MapInterface::Ptr map) { map_ = map; }
    
    virtual bool plan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        nav_msgs::msg::Path& path) = 0;

protected:
    MapInterface::Ptr map_;
};

}  // namespace nav_core
