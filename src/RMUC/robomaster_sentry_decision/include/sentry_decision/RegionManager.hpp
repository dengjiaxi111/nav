#ifndef REGION_MANAGER_HPP
#define REGION_MANAGER_HPP

#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <string>

struct Polygon {
    std::vector<geometry_msgs::msg::Point> vertices;
};

class RegionManager {
public:
    RegionManager();

    bool isInRedHighland(double x, double y) const;
    bool isInRedRegion(double x, double y) const;
    bool isInCentralRegion(double x, double y) const;
    bool isInBlueRegion(double x, double y) const;
    bool isInBlueHighland(double x, double y) const;
    bool isInHeroDeployZone(double x, double y) const;

    // 比赛许可区域
    bool isInsideAllowedRegion(double x, double y) const;
    geometry_msgs::msg::Point clampPointToAllowedRegion(const geometry_msgs::msg::Point& point) const;

    std::string getRegionName(double x, double y) const;

    static std::vector<geometry_msgs::msg::Point> calculateHexagonPoints(double center_x, double center_y, double radius = 200.0);
    geometry_msgs::msg::Point findSameRegionHexPoint(double target_x, double target_y, double robot_x, double robot_y) const;

private:
    Polygon red_highland_, red_region_, central_region_, blue_region_, blue_highland_, hero_deploy_zone_, allowed_region_;
    static bool isPointInPolygon(const geometry_msgs::msg::Point& point, const Polygon& polygon);
};

#endif // REGION_MANAGER_HPP
