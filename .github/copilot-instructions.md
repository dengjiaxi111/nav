# Navigation2026 - AI Coding Agent Instructions

ROS2 navigation system with LiDAR-inertial SLAM and probabilistic occupancy mapping.

## Core Values (八荣八耻)

- 以瞎猜接口为耻，以认真查询为荣（Read before assuming）
- 以模糊执行为耻，以寻求确认为荣（Seek confirmation）
- 以臆想业务为耻，以复用现有为荣（Reuse existing code）
- 以创造接口为耻，以主动测试为荣（Test proactively）
- 以跳过验证为耻，以人类确认为荣（Verify with humans）
- 以破坏架构为耻，以遵循规范为荣（Follow architecture）
- 以假装理解为耻，以诚实无知为荣（Admit uncertainty）
- 以盲目修改为耻，以谨慎重构为荣（Refactor carefully）

## Architecture & Data Flow

```
src/
├── localization/
│   ├── livox_ros_driver2/   # 🔒 Official Livox driver (do not modify)
│   ├── small_point_lio/     # 🔒 SLAM submodule (read-only)
│   └── lio_3se/             # ❌ DEPRECATED
├── mapping/
│   ├── rog_map/             # ✅ Core probabilistic mapping library
│   └── rog_map_ros2_node/   # ✅ ROS2 wrapper
```

**Topic Flow** (verified from source):
```
livox_ros_driver2 → /livox/lidar_* + /livox/imu_*
         ↓
small_point_lio → /cloud_registered (PointCloud2)
                → /Odometry (nav_msgs/Odometry)
         ↓
rog_map_ros2_node → occupancy grid + frontier points
```

**Key Principle**: Libraries (`rog_map`) = pure C++; Wrappers (`rog_map_ros2_node`) = ROS2 integration.

## Build & Verification

```bash
# Build single package
colcon build --packages-select <package_name> --symlink-install

# Verify success
colcon build --packages-select <pkg> 2>&1 | grep -E "ERROR|built package"

# Source and test
source install/setup.bash
ros2 run rog_map_ros2_node rog_map_node --ros-args -p config_file:=/path/to/config.yaml
```

**Required CMake definitions** (in `target_compile_definitions`):
- `USE_ROS2` - enables ROS2 code paths
- `ORIGIN_AT_CORNER` - ROG-Map grid coordinate system
- `ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"` - config file search path

## Configuration Pattern

YAML keys must **exactly match** `loader.LoadParam()` calls in `rog_map/include/rog_map/rog_map_core/config.hpp`:

```yaml
rog_map:
  resolution: 0.05
  ros_callback:
    enable: true
    cloud_topic: "/cloud_registered"  # From small_point_lio
    odom_topic: "/Odometry"           # From small_point_lio
  visualization:
    enable: true
    frame_id: "odom"
```

## Integration Launch

Use `integration.launch.py` to run full pipeline:
```bash
ros2 launch rog_map_ros2_node integration.launch.py
# Launches: small_point_lio + rog_map_node + RViz
```

## Code Style (Google C++ for ROS2)

- Files: `snake_case.cpp/.hpp`
- Classes: `PascalCase`, private members with `_` suffix
- Functions: `camelCase`, callbacks: `snake_case`
- 4 spaces indent, 100 char line limit

## Before Editing - MUST READ

1. **Headers**: Check `.hpp` for interfaces, constructors, namespaces
2. **Macros**: Search `#ifdef`, `#error` for compile requirements
3. **Config loading**: Match YAML to `loader.LoadParam()` key paths
4. **Topic names**: Never hardcode - read from config

## Critical DON'Ts

❌ Modify `small_point_lio/` (submodule) or `livox_ros_driver2/` (official)
❌ Use `lio_3se/` (deprecated)
❌ Add ROS deps directly to `rog_map/` (use wrapper)
❌ Skip `USE_ROS2`/`ORIGIN_AT_CORNER` compile definitions
❌ Hardcode paths - use `ROOT_DIR` macro or parameters
❌ Write excessive docs - max ONE summary per feature

## ROS2 Node Pattern Reference

```cpp
// Modern component-based node pattern (from small_point_lio_node.cpp)
class MyNode : public rclcpp::Node {
public:
    MyNode(const rclcpp::NodeOptions &options) : Node("node_name", options) {
        // Declare parameters
        auto param = declare_parameter<std::string>("param_name");
        
        // Create publishers/subscribers
        publisher_ = create_publisher<MsgType>("topic", 10);
        subscriber_ = create_subscription<MsgType>("topic", rclcpp::SensorDataQoS(),
            [this](const MsgType &msg) { /* callback */ });
        
        // TF2 integration
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    }
private:
    rclcpp::Publisher<MsgType>::SharedPtr publisher_;
    rclcpp::Subscription<MsgType>::SharedPtr subscriber_;
};

// Register as composable node
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(namespace::MyNode)
```

**Two patterns in this project**:
- `small_point_lio`: Component-based (inherits `rclcpp::Node`, composable)
- `rog_map_node`: Standalone wrapper (creates node, passes to library class)

## Key File Locations

| Purpose | Path |
|---------|------|
| ROG-Map config loader | `rog_map/include/rog_map/rog_map_core/config.hpp` |
| ROS2 wrapper node | `rog_map_ros2_node/src/rog_map_node.cpp` |
| Integration launch | `rog_map_ros2_node/launch/integration.launch.py` |
| YAML config | `rog_map_ros2_node/config/rog_map_config.yaml` |

---
**Last updated**: 2025-11-30
