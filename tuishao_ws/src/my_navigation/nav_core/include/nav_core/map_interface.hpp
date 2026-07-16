// nav_core/include/nav_core/map_interface.hpp
// 地图接口 - 抽象不同地图类型的统一查询接口

#pragma once
#include <memory>

namespace nav_core {

// 地图类型枚举
enum class MapType {
    OCCUPANCY_GRID,  // nav_msgs/OccupancyGrid
    ESDF,            // 欧氏符号距离场
    ELEVATION,       // 高程图
    COSTMAP,         // 代价地图
    UNKNOWN
};

// 地图查询结果
struct MapQuery {
    bool valid = false;      // 查询是否有效（在地图范围内）
    bool occupied = false;   // 是否被占据
    double cost = 0.0;       // 代价值 [0,1]
    double distance = 0.0;   // 到最近障碍物距离（ESDF用）
};

// 地图接口基类
class MapInterface {
public:
    using Ptr = std::shared_ptr<MapInterface>;
    virtual ~MapInterface() = default;
    
    // 地图类型
    virtual MapType type() const = 0;
    bool isOccupancyGrid() const { return type() == MapType::OCCUPANCY_GRID; }
    bool isEsdf() const { return type() == MapType::ESDF; }
    
    // 2D点查询
    virtual MapQuery query(double x, double y) const = 0;
    
    // 3D点查询（默认调用2D）
    virtual MapQuery query(double x, double y, double z) const {
        (void)z;
        return query(x, y);
    }
    
    // 便捷方法
    bool isOccupied(double x, double y) const {
        auto q = query(x, y);
        return !q.valid || q.occupied;
    }
    
    double getDistance(double x, double y) const { return query(x, y).distance; }
    double getCost(double x, double y) const { return query(x, y).cost; }
    
    // 地图信息
    virtual double resolution() const = 0;
    virtual void getBounds(double& min_x, double& min_y, 
                          double& max_x, double& max_y) const = 0;
    virtual bool is3D() const { return false; }
    virtual bool hasDistance() const { return false; }
    
    // 地图更新（由调度器独立调用）
    virtual void update() {}
};

}  // namespace nav_core
