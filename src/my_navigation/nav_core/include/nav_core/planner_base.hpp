// nav_core/include/nav_core/planner_base.hpp
// 规划器基类

#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include "nav_core/map_interface.hpp"

namespace nav_core {

class PlannerBase {
public:
    virtual ~PlannerBase() = default;
    
    virtual void initialize(rclcpp::Node* node) = 0;
    
    // 方式1: 直接设置OccupancyGrid（简单场景）
    virtual void setMap(nav_msgs::msg::OccupancyGrid::SharedPtr map) = 0;
    
    // 方式2: 设置地图接口（支持ESDF/高程图等）
    virtual void setMapInterface(MapInterface::Ptr map) { map_interface_ = map; }
    
    virtual bool plan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        nav_msgs::msg::Path& path) = 0;

protected:
    MapInterface::Ptr map_interface_;
};

}  // namespace nav_core
