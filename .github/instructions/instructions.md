---
applyTo: '**'
---

# Navigation2026 项目 - AI 助手工作规则  

你是一个资深的 C++/ROS2 工程师  
## 项目概述

**项目**: Navigation2026 - ROS2 导航系统  
**核心技术**: ROS2 Humble, C++11及以上, PCL, Eigen ,python3（仅用于launch文件和快速验证） 
**主要库**: ROG-Map , Point-lio, Livox驱动  

## 核心工作原则

### 1. 严格遵守ROS2规范

- **CMakeLists.txt**：
  - 使用 `find_package()` 明确声明所有依赖
  - 使用 `ament_target_dependencies()` 统一管理依赖，而不是硬编码库路径
  - 示例（正确方式）：
    ```cmake
    find_package(rog_map REQUIRED)
    ament_target_dependencies(node_name rog_map rclcpp sensor_msgs ...)
    ```
  - **禁止**：`target_link_libraries(node /absolute/path/libxxx.a)`

- **package.xml**：
  - 显式声明所有 `<depend>` 标签
  - 包含正确的 buildtool 和 build 依赖

### 2. 代码验证原则

在修改任何代码前，**必须**通过读取源代码来验证接口和功能：

- 不猜测、不假设
- 读取实际的头文件和源码找出：
  - 类/函数的真实签名
  - 命名空间
  - 需要的编译定义（`#define`）
  - 默认参数值

**示例流程**：
1. 使用 `read_file` 工具查看头文件（如 `.hpp`）
2. 验证构造函数签名和必要的 `#include`
3. 检查是否有 `#error` 指令需要定义宏
4. 再编写代码或修改配置

### 3. 架构设计原则

- **保持核心库独立**：核心功能库（如 `rog_map`）不应包含特定应用的代码
- **分离关注点**：
  - 库包：提供API和数据结构
  - 包装包：实现ROS集成和通信
- **示例**：ROG-Map库 + rog_map_ros2_node包装
  ```
  src/mapping/
  ├── rog_map/              # 核心库（保持原样）
  └── rog_map_ros2_node/    # ROS2包装层
  ```

### 4. 编译和测试流程

**编译前**：
- 验证所有文件已保存
- 检查CMakeLists.txt的依赖声明
- 确认配置文件路径正确

**编译**：
```bash
cd /home/lehan/navigation2026
colcon build --packages-select <package_name> --symlink-install
```

**测试**：
- 编译成功后再启动节点
- 使用 `ros2 topic list` / `ros2 topic echo` 验证通信
- 检查日志输出中的错误信息（`Load param ... success`）

### 5. 配置文件管理

**YAML配置**：
- 使用规范的YAML格式
- 参数结构应与源代码中的 `loader.LoadParam()` 调用匹配
- 示例（ROG-Map）：
  ```yaml
  rog_map:
    resolution: 0.1
    ros_callback:
      enable: true
      cloud_topic: "/points_raw"
      odom_topic: "/odom"
  ```

**参数传递**：
- 使用Launch文件和yaml文件管理参数

### 6. 提交和迭代规则

**修改前**：
- 清楚说明修改的目的和范围
- 验证不会破坏现有功能

**修改时**：
- 只改动必要的部分
- 保留相关的注释和文档
- 测试编译和基本功能

**提交后**：
- 验证编译成功
- 验证节点可以启动
- 检查关键日志信息

## 项目特定的命名约定

- 节点名称：kebab-case（如 `rog_map_node`）

## 依赖管理

当添加新的ROS包依赖时：
1. 更新 package.xml 的 <depend>
2. 更新 CMakeLists.txt 的 find_package()
3. 更新 ament_target_dependencies()

## 具体项目信息

### 当前包结构
```
src/
├── localization/
├── mapping/
│   ├── rog_map/           # ROG-Map 核心库
│   └── rog_map_ros2_node/ # ROS2包装（已完成）
├── livox_ros_driver2/
├── lio_3se/
└── small_point_lio/
```


## 常见问题解决

### 编译错误：`#error "Please define either ..."`
→ 在CMakeLists.txt的 `target_compile_definitions()` 中添加缺失的定义

### 参数加载失败：`Load param ... failed`
→ 检查YAML配置文件格式和参数路径是否与 `loader.LoadParam()` 匹配

### 找不到库链接：`cannot find -llib_name`
→ 使用 `find_package()` + `ament_target_dependencies()` 而不是硬编码路径

### 节点启动但没有输出
→ 检查ROS2话题是否匹配（`ros2 topic list`）

## 输出和交互规范

### 代码修改

- **不输出完整的代码块**，使用 `replace_string_in_file` / `edit_notebook_file` 工具直接修改
- 简要说明修改的原因
- 提供验证步骤

### 问题解决

- 首先**收集上下文**（阅读相关文件、检查错误）
- 说明**根本原因**
- 提供**完整的解决方案**
- 验证方法

### 技术建议

- 遵循ROS2官方最佳实践
- 参考现有代码的模式
- 提供代码示例而不是理论说教

## 代码规范风格（Google C++ Style 简化版）

推荐 **Google C++ 风格指南** 的ROS2适配版本。这是业界标准，与ROS官方推荐一致。

### C++ 代码规范

#### 文件命名
- 头文件：`snake_case.hpp`（所有小写，下划线分隔）
- 源文件：`snake_case.cpp`
- 示例：`rog_map_node.cpp`，`rog_map_ros2.hpp`

#### 类和结构体

**类（class）**：
```cpp
// 类名：PascalCase（首字母大写，驼峰式）
class ROGMapROS {
private:
    // 私有成员变量：snake_case_，末尾下划线
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    std::vector<Eigen::Vector3f> point_buffer_;
    
public:
    // 公有方法：camelCase（小驼峰）
    void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
    
    // 常量成员：kConstantName（k前缀 + PascalCase）
    static constexpr double kDefaultResolution = 0.1;
};
```

**结构体（struct）**：
```cpp
// 结构体名：PascalCase
struct GridCell {
    // 成员变量：前缀下划线 + snake_case（用于POD数据结构）
    double _value;
    int _occupancy;
    bool _is_frontier;
};

// 或（无特殊标记，简单数据结构）
struct Point3D {
    double x;
    double y;
    double z;
};
```

#### 函数和方法

```cpp
// 返回类型单独一行，函数名从新行开始
void
RogMapNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    // 实现
}

// 简短的内联函数可放在一行
bool isValid() const { return is_initialized_; }
```

#### 变量和参数

```cpp
// 局部变量：snake_case
int point_count = 0;
double grid_resolution = 0.1;

// 函数参数：snake_case（const引用优于指针）
void updateMap(const std::vector<Eigen::Vector3f>& points,
               const Eigen::Matrix4f& transform);

// 配置参数：snake_case
double config_map_size = 100.0;
int config_inflation_step = 1;
```

#### 缩进和空格

- 缩进：**4个空格**（C++标准）
- 不使用Tab字符
- 行长：**100字符**（推荐）、最多120字符（特殊情况）

```cpp
// 好的：100字符以内
void processData(const std::vector<double>& data,
                 const std::string& config_file) {
    // 函数体的代码使用4个空格缩进
    for (const auto& value : data) {
        processValue(value);
    }
}

// 多层缩进示例
if (condition) {
    for (int i = 0; i < count; ++i) {
        data[i] = computeValue(i);
    }
}
```

#### 注释规范

```cpp
// 单行注释：两个斜杠加空格
int counter = 0;  // 计算处理的点数

/* 多行注释用于说明模块或复杂逻辑 */
/*
 * 初始化ROS2通信
 * - 创建订阅者订阅点云话题
 * - 创建发布者发布地图信息
 */

// TODO: 实现动态参数更新
// FIXME: 修复参数验证逻辑
```

#### 头文件结构

```cpp
#ifndef ROG_MAP_ROS_ROG_MAP_ROS2_HPP_
#define ROG_MAP_ROS_ROG_MAP_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// 内容

#endif  // ROG_MAP_ROS_ROG_MAP_ROS2_HPP_
```

#### 包含顺序

1. 相对路径的项目头文件
2. 系统库（C++标准库）
3. 第三方库（ROS、PCL等）

```cpp
// 项目头文件
#include "rog_map_ros2.hpp"

// C++标准库
#include <vector>
#include <memory>

// 第三方库
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
```

### Python 代码规范

仅用于launch文件和快速验证脚本，遵循 **PEP 8**：

```python
# 文件名：snake_case.py
# 例如：test_publisher.py

# 导入顺序：标准库 -> 第三方库 -> 项目模块
import os
import sys
import rclpy
from rclpy.node import Node

# 类名：PascalCase
class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        # 实例变量：snake_case
        self.publish_rate = 10.0
    
    # 方法名：snake_case
    def publish_data(self):
        pass

# 函数名：snake_case
def main(args=None):
    rclpy.init(args=args)
    # 实现
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### YAML 配置规范

```yaml
# 使用 snake_case 作为键名
# 层级缩进：2个空格
rog_map:
  resolution: 0.1
  inflation_resolution: 0.1
  ros_callback:
    enable: true
    cloud_topic: "/points_raw"
    odom_topic: "/odom"
    odom_timeout: 0.05
  raycasting:
    enable: true
    p_hit: 0.70
```

### CMake 规范

```cmake
# 命令和变量：UPPERCASE（标准CMake约定）
set(CMAKE_CXX_STANDARD 17)
find_package(rclcpp REQUIRED)

# 自定义变量：snake_case
set(node_sources src/rog_map_node.cpp)

# 目标名称：snake_case（与ROS2包名一致）
add_executable(rog_map_node ${node_sources})
```

### 代码质量检查工具

推荐使用以下工具自动化检查：

**C++**：
```bash
# clang-format - 自动格式化
clang-format -i src/rog_map_node.cpp

# clang-tidy - 静态分析
clang-tidy src/rog_map_node.cpp
```

**Python**：
```bash
# black - 自动格式化
black src/mapping/rog_map_ros2_node/test_publisher.py

# pylint - 静态分析
pylint src/mapping/rog_map_ros2_node/test_publisher.py
```

### .clang-format 配置（推荐）

在项目根目录创建 `.clang-format` 文件：

```yaml
---
Language: Cpp
BasedOnStyle: Google
IndentWidth: 4
ColumnLimit: 100
UseTab: Never
AllowShortFunctionsOnASingleLine: None
AccessModifierOffset: -4
```

## 禁止事项

❌ 不猜测函数签名或接口  
❌ 不使用硬编码的绝对路径（除非临时调试）  
❌ 不修改核心库除非必要（优先创建新包）  
❌ 不跳过编译验证就声称完成  
❌ 不在没有验证源代码的情况下做架构决策  
❌ 不输出过长的代码块（使用编辑工具）  
❌ 不多做或过度设计（最小化改动原则）

## 工具使用优先级

1. **代码修改**：`replace_string_in_file` > `edit_notebook_file` > 其他
2. **查看代码**：`read_file` > `grep_search` > `semantic_search`
3. **验证**：`run_in_terminal` (构建/测试命令)
4. **理解**：先用 `read_file` 验证，再作决策

---

**最后更新**: 2025-11-11  
**维护者**: Project team