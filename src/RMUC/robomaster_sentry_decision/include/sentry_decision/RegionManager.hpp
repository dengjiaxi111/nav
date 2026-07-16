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

    bool isInRedRegion(double x, double y) const;
    bool isInBlueRegion(double x, double y) const;

    // 判断我方英雄是否在部署区（robot_id决定颜色）
    bool isOurHeroInDeployZone(double hero_x, double hero_y, int robot_id) const;

    // 将点限制在己方区域内（红方用红区，蓝方用蓝区），并避开禁止区域
    geometry_msgs::msg::Point clampToTeamRegion(const geometry_msgs::msg::Point& point,
                                                int robot_id,
                                                double robot_x = 0, double robot_y = 0) const;

    // 在敌人附近找一个同区域的目标点（同原国赛逻辑，但只用红/蓝区）
    geometry_msgs::msg::Point findSameTeamHexPoint(double target_x, double target_y,
                                                   double robot_x, double robot_y,
                                                   int robot_id) const;

    std::string getRegionName(double x, double y) const;

private:
    Polygon red_region_, blue_region_;
    Polygon red_hero_deploy_zone_, blue_hero_deploy_zone_;
    std::vector<Polygon> forbidden_zones_;

    static bool isPointInPolygon(const geometry_msgs::msg::Point& point, const Polygon& polygon);
    bool isInForbiddenZone(double x, double y) const;
    geometry_msgs::msg::Point adjustPointOutOfForbidden(const geometry_msgs::msg::Point& point,
                                                        double robot_x, double robot_y) const;
    static std::vector<geometry_msgs::msg::Point> calculateHexagonPoints(double center_x, double center_y, double radius = 200.0);
};

#endif
