/**
 * @file integration_component.hpp
 * @brief 集成组件 - 管理 ROGMap 和 Map2DProjector 的生命周期和依赖注入
 * 
 * 架构模式：
 * 1. IntegrationComponent 创建 ROGMapComponent
 * 2. 获取 ROGMapROS* 指针
 * 3. 创建 Map2DProjector，传递指针
 * 4. 所有组件在同一进程内运行，零拷贝通信
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <memory>
#include <thread>

// 前向声明
namespace rog_map {
    class ROGMapROS;
}

namespace rog_map_ros2_node {
    class ROGMapComponent;
}

namespace map_2d_projector {
    class Map2DProjector;
}

namespace rog_map_ros2_node {

/**
 * @brief 集成组件 - 统一管理所有子组件的生命周期
 * 
 * 依赖关系：
 * - ROGMapComponent (核心地图)
 * - Map2DProjector (依赖 ROGMapROS*)
 */
class IntegrationComponent : public rclcpp::Node {
public:
    explicit IntegrationComponent(const rclcpp::NodeOptions& options);
    ~IntegrationComponent() override;

private:
    // 子组件实例（按依赖顺序）
    std::shared_ptr<ROGMapComponent> rog_map_component_;
    std::shared_ptr<map_2d_projector::Map2DProjector> map_2d_projector_;
    
    // Executor 管理
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread executor_thread_;
};

} // namespace rog_map_ros2_node
