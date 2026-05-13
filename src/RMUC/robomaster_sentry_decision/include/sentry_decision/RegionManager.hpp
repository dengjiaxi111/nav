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
    bool isInHeroDeployZone(double x, double y) const;   // 蓝方英雄部署区（红方视角的敌方部署区）

    // 判断点是否在敌方英雄部署区（根据机器人的ID自动选择对应区域）
    bool isInEnemyHeroDeployZone(double x, double y, int robot_id) const;

    // 判断点是否在敌方工程取矿区域内
    bool isInEnemyEngineerMiningZone(double x, double y, int robot_id) const;

    bool isInsideAllowedRegion(double x, double y) const;
    geometry_msgs::msg::Point clampPointToAllowedRegion(const geometry_msgs::msg::Point& point) const;

    std::string getRegionName(double x, double y) const;

    // 禁用区域判断
    bool isInForbiddenZone(double x, double y) const;

    static std::vector<geometry_msgs::msg::Point> calculateHexagonPoints(double center_x, double center_y, double radius = 80.0);
    geometry_msgs::msg::Point findSameRegionHexPoint(double target_x, double target_y, double robot_x, double robot_y) const;

private:
    Polygon red_highland_, red_region_, central_region_, blue_region_, blue_highland_, hero_deploy_zone_, allowed_region_;
    Polygon red_hero_deploy_zone_;       // 红方英雄部署区（蓝方视角的敌方部署区）
    Polygon red_engineer_mining_zone_;    // 蓝方工程取矿区（红方视角的敌方蓝方工程取矿区）
    Polygon blue_engineer_mining_zone_;   // 红方工程取矿区（蓝方视角的敌方红方工程取矿区）
    std::vector<Polygon> forbidden_zones_;
    static bool isPointInPolygon(const geometry_msgs::msg::Point& point, const Polygon& polygon);
};

#endif // REGION_MANAGER_HPP
