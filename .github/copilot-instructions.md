# Navigation2026 - AI Coding Agent Instructions

This ROS2 navigation system integrates LiDAR-inertial SLAM and probabilistic occupancy mapping. Help developers maintain consistent patterns across the codebase.

## Claude Code 八荣八耻 (Core Values)

这些原则是编码助手的核心行为准则：

- 以瞎猜接口为耻，以认真查询为荣。
- 以模糊执行为耻，以寻求确认为荣。
- 以臆想业务为耻，以复用现有为荣。
- 以创造接口为耻，以主动测试为荣。
- 以跳过验证为耻，以人类确认为荣。
- 以破坏架构为耻，以遵循规范为荣。
- 以假装理解为耻，以诚实无知为荣。
- 以盲目修改为耻，以谨慎重构为荣。

## 1. Architecture Overview

**Project Structure**: Two-tier component design (early stage)
```
src/
├── localization/              # SLAM front-end
│   ├── livox_ros_driver2/   # Official Livox LiDAR driver (do not modify)
│   └── small_point_lio/      # ⭐ Primary SLAM (submodule, read-only)
├── mapping/                   # Occupancy mapping backend
│   ├── rog_map/             # Core probabilistic mapping library
│   └── rog_map_ros2_node/   # ROS2 integration wrapper
```

**Project Status**:
- 🚀 **Early stage repo** - currently contains only localization + mapping modules
- 📦 **Future packages**: Planning to add navigation, control, planning modules incrementally
- ⚠️ **Key constraints**:
  - `lio_3se/` is **deprecated** - do not use
  - `livox_ros_driver2/` is **official driver** - modifications discouraged
  - `small_point_lio/` is **submodule** - cannot be modified in this repo, point upstream for changes

**Key Architectural Principle**: **Separation of concerns**
- **Libraries** (`rog_map`, `small_point_lio`): Pure C++ algorithms, minimal ROS dependencies
- **Wrappers** (`rog_map_ros2_node`): Thin ROS2 integration layers that instantiate and manage library objects
- **Drivers** (`livox_ros_driver2`): Official hardware abstraction for sensor input (maintain as-is)

**Data Flow**: 
```
LiDAR Points → livox_ros_driver2 → /points_raw (topic)
                                      ↓
                        small_point_lio (SLAM) → /odom
                                      ↓
                              ROGMapROS processes
                            (subscribes in constructor)
                                      ↓
                         Publishes /occupancy_grid, /frontier_points
                            (multiple map layers)
```

## 1.5 Project Maturity & Development Roadmap

**Current State (Early Stage)**:
- ✅ Localization module: `small_point_lio` (SLAM, via git submodule)
- ✅ Mapping module: `rog_map` + ROS2 wrapper (`rog_map_ros2_node`)
- ⏳ Hardware abstraction: `livox_ros_driver2` (official driver, included)

**Planned Future Modules** (incremental additions):
- Navigation stack (path planning, trajectory generation)
- Motion control and actuation interface
- Additional SLAM options or fusion methods
- Higher-level planning and decision-making

**Development Philosophy**:
- Add new packages to `src/` directory as needed
- Maintain clear module boundaries
- Each package is independently buildable
- Document integration points when adding new modules

**Current Integration Example**: small_point_lio → rog_map_ros2_node
```yaml
# small_point_lio publishes:
#   /cloud_registered (PointCloud2)  → Input to ROG-Map
#   /odom (Odometry)                  → Robot state for mapping

# rog_map_ros2_node processes both topics and publishes:
#   /rog_map/occ (OccupancyGrid)
#   /rog_map/unk (PointCloud2)
#   /rog_map/frontier (PointCloud2)
```

## 2. Critical Build & Compilation Workflow

**Build system**: ROS2 + Colcon + CMake 3.16+

**Standard build command**:
```bash
cd /home/lehan/navigation2026
colcon build --packages-select <package_name> --symlink-install
```

**Key compile definitions** (required for cross-platform code):
```cmake
# In CMakeLists.txt target_compile_definitions()
USE_ROS2          # Enables ROS2 code paths in headers
ORIGIN_AT_CORNER  # ROG-Map specific: grid coordinate system
ROOT_DIR="..."    # Config file search path
```

**Dependency declaration rules** (strict ROS2 standards):
- ✅ CORRECT: `find_package(pkg_name REQUIRED)` + `ament_target_dependencies(target pkg_name)`
- ❌ WRONG: Direct `target_link_libraries()` with absolute paths

**Verification after edit**:
```bash
colcon build --packages-select <pkg> 2>&1 | tail -20
# Look for "Built target" and "built package" success messages
```

## 3. ROS2 Integration Patterns

### Package Structure Template
Every ROS2 package must have:
```
package_name/
├── package.xml          # Dependencies: <depend>, <buildtool_depend>
├── CMakeLists.txt       # Build configuration
├── src/
│   └── node_name.cpp    # Executable implementation
├── config/
│   └── config_name.yaml # Parameter files
├── launch/
│   └── name.launch.py   # Launch scripts (Python 3)
└── include/package_name/
    └── header.hpp       # Public headers
```

### ROS2 Node Initialization Pattern (from `rog_map_node.cpp`)
```cpp
// 1. Create ROS2 node
auto node = std::make_shared<rclcpp::Node>("node_name");

// 2. Declare and retrieve parameters
node->declare_parameter("param_name", default_value);
auto param_value = node->get_parameter("param_name").as_string();

// 3. Instantiate library class (handles subscriptions/publishers internally)
auto map_ros = std::make_unique<rog_map::ROGMapROS>(node, config_file_path);

// 4. Spin node
rclcpp::spin(node);
rclcpp::shutdown();
```

### Configuration File Integration (YAML → Code)
- YAML structure must **exactly match** the config loader's key path expectations
- ROG-Map uses nested YAML: `rog_map.ros_callback.cloud_topic` = `/points_raw`
- **Always verify** loader calls in source code before editing YAML

Example (`rog_map_config.yaml`):
```yaml
rog_map:
  resolution: 0.1
  ros_callback:
    enable: true
    cloud_topic: "/points_raw"
    odom_topic: "/odom"
    odom_timeout: 0.05
  visualization:
    enable: true
    pub_unknown_map_en: true
```

## 4. Project-Specific Conventions

### Topic Names (ROS2 Best Practices)
- Points from hardware: `/points_raw` (processed by livox_ros_driver2)
- Odometry/state: `/odom` (from lio_3se or small_point_lio)
- Map layers published by ROG-Map: `/occupancy_grid`, `/frontier_points`, `/esdf_map`, etc.

### Frame IDs
- World/map frame: `world` or `map`
- Robot/ego frame: `drone` or `base_link` (from odom messages)
- Configured in RViz visualization: `frame_id: "map"`

### Parameter Naming (`package.xml` <depend> vs file content)
- Always list ROS packages in `package.xml`
- Parameter yaml files should use `snake_case` keys
- Command-line parameter override: `--ros-args -p config_file:=/path/to/file.yaml`

## 5. Testing & Validation Commands

After any modification:

**1. Verify compilation**:
```bash
colcon build --packages-select affected_package 2>&1 | grep -E "ERROR|built package"
```

**2. Source environment**:
```bash
source install/setup.bash
```

**3. Test node startup** (example):
```bash
ros2 run rog_map_ros2_node rog_map_node \
  --ros-args -p config_file:=/path/to/rog_map_config.yaml
```

**4. Monitor topic communication**:
```bash
ros2 topic list                    # See all active topics
ros2 topic echo /points_raw        # Monitor incoming point cloud
ros2 topic info /occupancy_grid    # Check publisher/subscriber count
```

**5. Inspect ROS2 lifecycle**:
```bash
ros2 node list
ros2 param list /node_name
```

## 6. Common Patterns & Anti-patterns

### ✅ DO: Multi-threaded callback handling
ROG-Map uses callback groups for concurrent point cloud + odometry processing:
```cpp
// Multiple callback groups prevent blocking on slow subscriptions
rclcpp::CallbackGroup::SharedPtr cloud_cbk_group = 
    node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
```

### ✅ DO: Shared pointer management
All ROS2 entities use `SharedPtr`:
```cpp
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;
```

### ❌ DON'T: Hardcoded topic/frame names
Always make them configurable or at minimum centralize in constants:
```cpp
// Bad: hardcoded
cloud_sub = node->create_subscription<...>("/points_raw", 10, ...);
// Good: from config
std::string cloud_topic = config.load("ros_callback.cloud_topic");
```

### ❌ DON'T: Blocking operations in callbacks
Callbacks must not call `sleep()` or heavy compute—defer work to timer callbacks or separate threads.

## 7. When to Read Code Before Modifying

**MANDATORY**: Read these files before editing:
1. **Interface definitions**: `.hpp` headers (constructors, function signatures, namespaces)
2. **Compile requirements**: Search for `#ifdef`, `#error`, `#define` macros
3. **Configuration loading**: Look for `loader.LoadParam()` calls to match YAML structure
4. **Topic subscriptions**: Check callback method signatures in ROS integration classes

**Tool usage order**:
```
read_file (for .hpp, .cpp)
  → grep_search (for specific function references)
    → semantic_search (for understanding intent)
      → THEN edit_file or replace_string_in_file
```

## 8. Code Style (Google C++ adapted for ROS2)

**File naming**: `snake_case.cpp` / `snake_case.hpp`

**Classes**: `PascalCase` with private members suffixed `_`
```cpp
class ROGMapROS {
private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
public:
    void processPointCloud(...);
};
```

**Functions/methods**: `camelCase` for most, `snake_case` for callbacks
```cpp
void updateMap();              // regular method
void cloudCallback(...);       // ROS callback
```

**Variables**: `snake_case` (local, member with `_` suffix)
```cpp
std::vector<Eigen::Vector3f> point_buffer_;
double grid_resolution = 0.1;
```

**Constants**: `kPascalCase` prefix
```cpp
static constexpr double kDefaultResolution = 0.1;
```

**Indentation**: 4 spaces (never tabs), line limit 100 chars

**Header guards**: `PROJECT_SUBDIR_FILENAME_HPP_`
```cpp
#ifndef ROG_MAP_ROS_ROG_MAP_ROS2_HPP_
#define ROG_MAP_ROS_ROG_MAP_ROS2_HPP_
// content
#endif
```

**Include order**:
1. Project headers (`#include "...hpp"`)
2. Standard library (`#include <vector>`)
3. Third-party (`#include <rclcpp/...>`)

## 9. File Locations Reference

| Purpose | Path | Status |
|---------|------|--------|
| Core ROG-Map library (keep untouched) | `src/mapping/rog_map/` | ✅ Modifiable |
| ROS2 wrapper for ROG-Map | `src/mapping/rog_map_ros2_node/` | ✅ Modifiable |
| Launch scripts | `src/mapping/rog_map_ros2_node/launch/` | ✅ Modifiable |
| Map config (YAML) | `src/mapping/rog_map_ros2_node/config/` | ✅ Modifiable |
| RViz visualization config | `src/mapping/rog_map_ros2_node/rviz/` | ✅ Modifiable |
| Primary SLAM front-end | `src/localization/small_point_lio/` | 🔒 Submodule (read-only) |
| LiDAR driver | `src/localization/livox_ros_driver2/` | 🔒 Official (maintain as-is) |
| Deprecated SLAM | `src/localization/lio_3se/` | ❌ Do not use |
| Build artifacts | `build/`, `install/`, `log/` | 🚫 Ignore |

## 10. Documentation Policy

**⚠️ CRITICAL RULE**:
- **NO excessive documentation!** 
- Each feature implementation = **maximum ONE simple changelog/summary document**
- Keep documentation minimal and action-focused
- Do NOT create: multiple guides, tutorials, detailed explanations, FAQ sections, etc.
- Example of acceptable documentation:
  ```markdown
  # Implementation: Feature X
  
  - Modified: file_a.yaml (changed param from X to Y)
  - Added: launch_script.py (integrates modules A and B)
  - Verified: `colcon build` successful
  ```

## 11. Critical "DO NOT" List

❌ Modify `small_point_lio/` - it's a submodule (submit changes upstream instead)  
❌ Use `lio_3se/` - it's deprecated  
❌ Modify `livox_ros_driver2/` - use official driver as-is  
❌ Add ROS dependencies directly to `rog_map/` (use wrapper instead)  
❌ Commit build artifacts or `.clang-format` style violations  
❌ Use blocking operations (`std::this_thread::sleep_for`) in callbacks  
❌ Assume topic names—always source from config  
❌ Skip CMakeLists.txt compile definitions needed for feature flags  
❌ Hardcode paths—use `ROOT_DIR` macro or ros parameter passing  
❌ **Write excessive documentation** - keep it minimal, action-focused, one doc per feature max  

---

**Last updated**: 2025-11-11  
**For questions**: Refer to existing READMEs in each package subdirectory
