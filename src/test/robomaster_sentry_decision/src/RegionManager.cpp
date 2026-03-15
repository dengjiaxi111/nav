#include "sentry_decision/RegionManager.hpp"
#include <cmath>
#include <algorithm>

RegionManager::RegionManager() {
    geometry_msgs::msg::Point point;
    
    // 初始化红方高地区域
    red_highland_.vertices.clear();
    
    point.x = 328; point.y = 984; point.z = 0;
    red_highland_.vertices.push_back(point);
    
    point.x = 334; point.y = 1444; point.z = 0;
    red_highland_.vertices.push_back(point);
    
    point.x = 1336; point.y = 1440; point.z = 0;
    red_highland_.vertices.push_back(point);
    
    point.x = 1368; point.y = 1342; point.z = 0;
    red_highland_.vertices.push_back(point);
    
    point.x = 1048; point.y = 1326; point.z = 0;
    red_highland_.vertices.push_back(point);
    
    point.x = 800; point.y = 992; point.z = 0;
    red_highland_.vertices.push_back(point);
    
    // 初始化红方区域
    red_region_.vertices.clear();
    
    point.x = 54; point.y = 996; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 800; point.y = 992; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 1048; point.y = 1326; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 1368; point.y = 1342; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 1366; point.y = 1270; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 1262; point.y = 1246; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 926; point.y = 814; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 938; point.y = 628; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 1056; point.y = 284; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 1024; point.y = 306; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 754; point.y = 310; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 708; point.y = 408; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 394; point.y = 414; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 380; point.y = 50; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 192; point.y = 56; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 198; point.y = 316; point.z = 0;
    red_region_.vertices.push_back(point);
    
    point.x = 52; point.y = 328; point.z = 0;
    red_region_.vertices.push_back(point);
    
    // 初始化中央区域
    central_region_.vertices.clear();
    
    point.x = 1368; point.y = 1342; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 1366; point.y = 1270; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 1262; point.y = 1246; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 926; point.y = 814; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 938; point.y = 628; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 1080; point.y = 374; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 996; point.y = 236; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 790; point.y = 224; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 704; point.y = 390; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 424; point.y = 382; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 430; point.y = 56; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 1250; point.y = 76; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 1252; point.y = 240; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 1536; point.y = 256; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 1760; point.y = 566; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 1768; point.y = 966; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 1690; point.y = 1104; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 1800; point.y = 1256; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 2020; point.y = 1254; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 2112; point.y = 1098; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 2402; point.y = 1108; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 2404; point.y = 1444; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 1552; point.y = 1444; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 1544; point.y = 1256; point.z = 0;
    central_region_.vertices.push_back(point);
    
    point.x = 1272; point.y = 1252; point.z = 0;
    central_region_.vertices.push_back(point);
    
    // 初始化蓝方区域
    blue_region_.vertices.clear();
    
    point.x = 2744; point.y = 1164; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 2616; point.y = 1148; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 2600; point.y = 1438; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 2450; point.y = 1436; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 2446; point.y = 1064; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 2098; point.y = 1066; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 2020; point.y = 1178; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 1794; point.y = 1164; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 1762; point.y = 1096; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 1888; point.y = 808; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 1890; point.y = 666; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 1576; point.y = 218; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 1572; point.y = 160; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 1754; point.y = 166; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 2004; point.y = 500; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    point.x = 2746; point.y = 506; point.z = 0;
    blue_region_.vertices.push_back(point);
    
    // 初始化蓝方高地区域
    blue_highland_.vertices.clear();
    
    point.x = 2746; point.y = 506; point.z = 0;
    blue_highland_.vertices.push_back(point);
    
    point.x = 2020; point.y = 488; point.z = 0;
    blue_highland_.vertices.push_back(point);
    
    point.x = 1786; point.y = 142; point.z = 0;
    blue_highland_.vertices.push_back(point);
    
    point.x = 1564; point.y = 136; point.z = 0;
    blue_highland_.vertices.push_back(point);
    
    point.x = 1570; point.y = 46; point.z = 0;
    blue_highland_.vertices.push_back(point);
    
    point.x = 2570; point.y = 38; point.z = 0;
    blue_highland_.vertices.push_back(point);
    
    // 初始化英雄部署区域 (矩形)
    hero_deploy_zone_.vertices.clear();
    
    point.x = 2752; point.y = 942; point.z = 0;
    hero_deploy_zone_.vertices.push_back(point);
    
    point.x = 2756; point.y = 48; point.z = 0;
    hero_deploy_zone_.vertices.push_back(point);
    
    point.x = 2194; point.y = 48; point.z = 0;
    hero_deploy_zone_.vertices.push_back(point);
    
    point.x = 2194; point.y = 942; point.z = 0;
    hero_deploy_zone_.vertices.push_back(point);
}

bool RegionManager::isPointInPolygon(const geometry_msgs::msg::Point& point, const Polygon& polygon) {
    // 静态成员函数，不能有const限定符
    int n = polygon.vertices.size();
    if (n < 3) return false;
    
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((polygon.vertices[i].y > point.y) != (polygon.vertices[j].y > point.y)) &&
            (point.x < (polygon.vertices[j].x - polygon.vertices[i].x) * 
                      (point.y - polygon.vertices[i].y) / 
                      (polygon.vertices[j].y - polygon.vertices[i].y) + 
                      polygon.vertices[i].x)) {
            inside = !inside;
        }
    }
    
    return inside;
}

bool RegionManager::isInRedHighland(double x, double y) const {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    return isPointInPolygon(p, red_highland_);
}

bool RegionManager::isInRedRegion(double x, double y) const {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    return isPointInPolygon(p, red_region_);
}

bool RegionManager::isInCentralRegion(double x, double y) const {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    return isPointInPolygon(p, central_region_);
}

bool RegionManager::isInBlueRegion(double x, double y) const {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    return isPointInPolygon(p, blue_region_);
}

bool RegionManager::isInBlueHighland(double x, double y) const {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    return isPointInPolygon(p, blue_highland_);
}

bool RegionManager::isInHeroDeployZone(double x, double y) const {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    return isPointInPolygon(p, hero_deploy_zone_);
}

std::string RegionManager::getRegionName(double x, double y) const {
    if (isInRedHighland(x, y)) return "red_highland";
    if (isInRedRegion(x, y)) return "red_region";
    if (isInCentralRegion(x, y)) return "central_region";
    if (isInBlueRegion(x, y)) return "blue_region";
    if (isInBlueHighland(x, y)) return "blue_highland";
    if (isInHeroDeployZone(x, y)) return "hero_deploy_zone";  // 新增英雄部署区检查
    return "unknown";
}

std::vector<geometry_msgs::msg::Point> RegionManager::calculateHexagonPoints(double center_x, double center_y, double radius) {
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(6);
    
    for (int i = 0; i < 6; i++) {
        double angle = 60.0 * i * M_PI / 180.0;
        geometry_msgs::msg::Point p;
        p.x = center_x + radius * std::cos(angle);
        p.y = center_y + radius * std::sin(angle);
        p.z = 0.0;
        points.push_back(p);
    }
    
    return points;
}

geometry_msgs::msg::Point RegionManager::findSameRegionHexPoint(double target_x, double target_y, 
                                                               double robot_x, double robot_y) const {
    auto hex_points = calculateHexagonPoints(target_x, target_y, 200.0);
    std::string target_region = getRegionName(target_x, target_y);
    
    std::vector<geometry_msgs::msg::Point> same_region_points;
    
    for (const auto& point : hex_points) {
        if (getRegionName(point.x, point.y) == target_region) {
            same_region_points.push_back(point);
        }
    }
    
    if (!same_region_points.empty()) {
        // 从同区域点中选择距离机器人最近的点
        geometry_msgs::msg::Point best_point = same_region_points[0];
        double min_distance = std::sqrt(std::pow(robot_x - best_point.x, 2) + std::pow(robot_y - best_point.y, 2));
        
        for (size_t i = 1; i < same_region_points.size(); i++) {
            double distance = std::sqrt(std::pow(robot_x - same_region_points[i].x, 2) + 
                                       std::pow(robot_y - same_region_points[i].y, 2));
            if (distance < min_distance) {
                min_distance = distance;
                best_point = same_region_points[i];
            }
        }
        
        return best_point;
    }
    
    // 如果没有同区域点，选择距离机器人最近的点
    geometry_msgs::msg::Point best_point = hex_points[0];
    double min_distance = std::sqrt(std::pow(robot_x - best_point.x, 2) + std::pow(robot_y - best_point.y, 2));
    
    for (size_t i = 1; i < hex_points.size(); i++) {
        double distance = std::sqrt(std::pow(robot_x - hex_points[i].x, 2) + std::pow(robot_y - hex_points[i].y, 2));
        if (distance < min_distance) {
            min_distance = distance;
            best_point = hex_points[i];
        }
    }
    
    return best_point;
}
