#ifndef REGION_MANAGER_HPP
#define REGION_MANAGER_HPP

#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <string>

// 多边形结构体
struct Polygon {
    std::vector<geometry_msgs::msg::Point> vertices;
};

class RegionManager {
public:
    RegionManager();
    
    // 区域检查函数
    bool isInRedHighland(double x, double y) const;
    bool isInRedRegion(double x, double y) const;
    bool isInCentralRegion(double x, double y) const;
    bool isInBlueRegion(double x, double y) const;
    bool isInBlueHighland(double x, double y) const;
    bool isInHeroDeployZone(double x, double y) const;
    
    // 获取区域名称
    std::string getRegionName(double x, double y) const;
    
    // 计算正六边形的六个点（距离2m=200cm）
    static std::vector<geometry_msgs::msg::Point> calculateHexagonPoints(double center_x, double center_y, double radius = 200.0);
    
    // 找到同区域的六边形点
    geometry_msgs::msg::Point findSameRegionHexPoint(double target_x, double target_y, 
                                                    double robot_x, double robot_y) const;
    
    // 工具函数：检查点是否在多边形内（静态函数）
    static bool isPointInPolygon(const geometry_msgs::msg::Point& point, const Polygon& polygon);
    
private:
    // 地图区域定义
    Polygon red_highland_;
    Polygon red_region_;
    Polygon central_region_;
    Polygon blue_region_;
    Polygon blue_highland_;
    Polygon hero_deploy_zone_;
};

#endif // REGION_MANAGER_HPP
