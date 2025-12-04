// nav_core/include/nav_core/map_interface.hpp
// 地图接口 - 抽象不同地图类型的统一查询接口

#pragma once
#include <memory>

namespace nav_core {

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
    
    // === 基本查询 ===
    
    // 2D点查询（适用于2D栅格地图）
    virtual MapQuery query(double x, double y) const = 0;
    
    // 3D点查询（适用于3D地图/高程图，默认调用2D）
    virtual MapQuery query(double x, double y, double z) const {
        (void)z;
        return query(x, y);
    }
    
    // === 便捷方法 ===
    
    bool isOccupied(double x, double y) const {
        auto q = query(x, y);
        return !q.valid || q.occupied;
    }
    
    bool isOccupied(double x, double y, double z) const {
        auto q = query(x, y, z);
        return !q.valid || q.occupied;
    }
    
    double getDistance(double x, double y) const {
        return query(x, y).distance;
    }
    
    double getCost(double x, double y) const {
        return query(x, y).cost;
    }
    
    // === 地图信息 ===
    
    virtual double resolution() const = 0;
    virtual void getBounds(double& min_x, double& min_y, 
                          double& max_x, double& max_y) const = 0;
    
    // 是否支持3D查询
    virtual bool is3D() const { return false; }
    
    // 是否支持ESDF距离查询
    virtual bool hasDistance() const { return false; }
    
    // === 动态更新（可选实现） ===
    
    // 添加临时障碍物（用于动态避障）
    virtual void addObstacle(double x, double y, double radius) {
        (void)x; (void)y; (void)radius;
    }
    
    // 清除临时障碍物
    virtual void clearDynamicObstacles() {}
    
    // 标记区域需要更新
    virtual void markForUpdate(double x, double y, double radius) {
        (void)x; (void)y; (void)radius;
    }
};

}  // namespace nav_core
