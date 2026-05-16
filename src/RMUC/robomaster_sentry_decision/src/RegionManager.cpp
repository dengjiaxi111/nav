#include "sentry_decision/RegionManager.hpp"
#include <cmath>
#include <algorithm>

RegionManager::RegionManager() {
    geometry_msgs::msg::Point point;

    // ========== 原有区域初始化（保持不变） ==========
    // 红方高地区域
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

    // 红方区域
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

    // 中央区域
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

    // 蓝方区域
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

    // 蓝方高地区域
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

    // 英雄部署区域 (蓝方英雄部署区，红方视角的敌方部署区)
    hero_deploy_zone_.vertices.clear();
    point.x = 2752; point.y = 942; point.z = 0;
    hero_deploy_zone_.vertices.push_back(point);
    point.x = 2756; point.y = 48; point.z = 0;
    hero_deploy_zone_.vertices.push_back(point);
    point.x = 2194; point.y = 48; point.z = 0;
    hero_deploy_zone_.vertices.push_back(point);
    point.x = 2194; point.y = 942; point.z = 0;
    hero_deploy_zone_.vertices.push_back(point);

    // 红方英雄部署区 (蓝方视角的敌方部署区)
    red_hero_deploy_zone_.vertices.clear();
    point.x = 48; point.y = 1454; point.z = 0;
    red_hero_deploy_zone_.vertices.push_back(point);
    point.x = 622; point.y = 1434; point.z = 0;
    red_hero_deploy_zone_.vertices.push_back(point);
    point.x = 614; point.y = 544; point.z = 0;
    red_hero_deploy_zone_.vertices.push_back(point);
    point.x = 56; point.y = 546; point.z = 0;
    red_hero_deploy_zone_.vertices.push_back(point);

    // 允许区域
    allowed_region_.vertices.clear();
    point.x = 84; point.y = 972; point.z = 0;
    allowed_region_.vertices.push_back(point);
    point.x = 374; point.y = 974; point.z = 0;
    allowed_region_.vertices.push_back(point);
    point.x = 382; point.y = 1428; point.z = 0;
    allowed_region_.vertices.push_back(point);
    point.x = 2390; point.y = 1434; point.z = 0;
    allowed_region_.vertices.push_back(point);
    point.x = 2448; point.y = 1032; point.z = 0;
    allowed_region_.vertices.push_back(point);
    point.x = 2714; point.y = 1072; point.z = 0;
    allowed_region_.vertices.push_back(point);
    point.x = 2718; point.y = 542; point.z = 0;
    allowed_region_.vertices.push_back(point);
    point.x = 2428; point.y = 528; point.z = 0;
    allowed_region_.vertices.push_back(point);
    point.x = 2414; point.y = 84; point.z = 0;
    allowed_region_.vertices.push_back(point);
    point.x = 442; point.y = 90; point.z = 0;
    allowed_region_.vertices.push_back(point);
    point.x = 432; point.y = 466; point.z = 0;
    allowed_region_.vertices.push_back(point);
    point.x = 88; point.y = 466; point.z = 0;
    allowed_region_.vertices.push_back(point);

    // ========== 新增：工程取矿区 ==========
    // 蓝方工程取矿区（红方视角的敌方蓝方工程取矿区）
    blue_engineer_mining_zone_.vertices.clear();
    point.x = 1520; point.y = 764; point.z = 0;
    blue_engineer_mining_zone_.vertices.push_back(point);
    point.x = 1596; point.y = 698; point.z = 0;
    blue_engineer_mining_zone_.vertices.push_back(point);
    point.x = 1496; point.y = 596; point.z = 0;
    blue_engineer_mining_zone_.vertices.push_back(point);
    point.x = 1382; point.y = 636; point.z = 0;
    blue_engineer_mining_zone_.vertices.push_back(point);

    // 红方工程取矿区（蓝方视角的敌方红方工程取矿区）
    red_engineer_mining_zone_.vertices.clear();
    point.x = 1426; point.y = 848; point.z = 0;
    red_engineer_mining_zone_.vertices.push_back(point);
    point.x = 1314; point.y = 890; point.z = 0;
    red_engineer_mining_zone_.vertices.push_back(point);
    point.x = 1220; point.y = 792; point.z = 0;
    red_engineer_mining_zone_.vertices.push_back(point);
    point.x = 1304; point.y = 712; point.z = 0;
    red_engineer_mining_zone_.vertices.push_back(point);

    // ========== 禁止区域（7个多边形，改用 push_back 逐个添加） ==========
    forbidden_zones_.clear();

    // 1
    {
        Polygon poly;
        poly.vertices.clear();
        point.x = 1298; point.y = 948; point.z = 0; poly.vertices.push_back(point);
        point.x = 1472; point.y = 892; point.z = 0; poly.vertices.push_back(point);
        point.x = 1648; point.y = 712; point.z = 0; poly.vertices.push_back(point);
        point.x = 1652; point.y = 672; point.z = 0; poly.vertices.push_back(point);
        point.x = 1512; point.y = 540; point.z = 0; poly.vertices.push_back(point);
        point.x = 1346; point.y = 590; point.z = 0; poly.vertices.push_back(point);
        point.x = 1162; point.y = 770; point.z = 0; poly.vertices.push_back(point);
        point.x = 1158; point.y = 808; point.z = 0; poly.vertices.push_back(point);
        forbidden_zones_.push_back(poly);
    }
    // 2
    {
        Polygon poly;
        poly.vertices.clear();
        point.x = 1334; point.y = 232; point.z = 0; poly.vertices.push_back(point);
        point.x = 1542; point.y = 236; point.z = 0; poly.vertices.push_back(point);
        point.x = 1566; point.y = 38;  point.z = 0; poly.vertices.push_back(point);
        point.x = 1266; point.y = 38;  point.z = 0; poly.vertices.push_back(point);
        point.x = 1270; point.y = 148; point.z = 0; poly.vertices.push_back(point);
        point.x = 1336; point.y = 156; point.z = 0; poly.vertices.push_back(point);
        forbidden_zones_.push_back(poly);
    }
    // 3
    {
        Polygon poly;
        poly.vertices.clear();
        point.x = 1262; point.y = 1242; point.z = 0; poly.vertices.push_back(point);
        point.x = 1254; point.y = 1442; point.z = 0; poly.vertices.push_back(point);
        point.x = 1542; point.y = 1446; point.z = 0; poly.vertices.push_back(point);
        point.x = 1544; point.y = 1342; point.z = 0; poly.vertices.push_back(point);
        point.x = 1474; point.y = 1334; point.z = 0; poly.vertices.push_back(point);
        point.x = 1480; point.y = 1254; point.z = 0; poly.vertices.push_back(point);
        forbidden_zones_.push_back(poly);
    }
    // 4
    {
        Polygon poly;
        poly.vertices.clear();
        point.x = 270; point.y = 832; point.z = 0; poly.vertices.push_back(point);
        point.x = 216; point.y = 794; point.z = 0; poly.vertices.push_back(point);
        point.x = 216; point.y = 696; point.z = 0; poly.vertices.push_back(point);
        point.x = 270; point.y = 652; point.z = 0; poly.vertices.push_back(point);
        point.x = 358; point.y = 680; point.z = 0; poly.vertices.push_back(point);
        point.x = 366; point.y = 798; point.z = 0; poly.vertices.push_back(point);
        point.x = 274; point.y = 850; point.z = 0; poly.vertices.push_back(point);
        forbidden_zones_.push_back(poly);
    }
    // 5
    {
        Polygon poly;
        poly.vertices.clear();
        point.x = 2526; point.y = 838; point.z = 0; poly.vertices.push_back(point);
        point.x = 2438; point.y = 812; point.z = 0; poly.vertices.push_back(point);
        point.x = 2432; point.y = 694; point.z = 0; poly.vertices.push_back(point);
        point.x = 2530; point.y = 648; point.z = 0; poly.vertices.push_back(point);
        point.x = 2594; point.y = 694; point.z = 0; poly.vertices.push_back(point);
        point.x = 2592; point.y = 794; point.z = 0; poly.vertices.push_back(point);
        forbidden_zones_.push_back(poly);
    }
    // 6
    {
        Polygon poly;
        poly.vertices.clear();
        point.x = 1756; point.y = 142; point.z = 0; poly.vertices.push_back(point);
        point.x = 2004; point.y = 492; point.z = 0; poly.vertices.push_back(point);
        point.x = 2280; point.y = 496; point.z = 0; poly.vertices.push_back(point);
        point.x = 2290; point.y = 516; point.z = 0; poly.vertices.push_back(point);
        point.x = 2336; point.y = 508; point.z = 0; poly.vertices.push_back(point);
        point.x = 2342; point.y = 440; point.z = 0; poly.vertices.push_back(point);
        point.x = 2282; point.y = 432; point.z = 0; poly.vertices.push_back(point);
        point.x = 2244; point.y = 334; point.z = 0; poly.vertices.push_back(point);
        point.x = 2338; point.y = 326; point.z = 0; poly.vertices.push_back(point);
        point.x = 2238; point.y = 310; point.z = 0; poly.vertices.push_back(point);
        point.x = 2202; point.y = 310; point.z = 0; poly.vertices.push_back(point);
        point.x = 2248; point.y = 440; point.z = 0; poly.vertices.push_back(point);
        point.x = 2032; point.y = 440; point.z = 0; poly.vertices.push_back(point);
        point.x = 1820; point.y = 138; point.z = 0; poly.vertices.push_back(point);
        forbidden_zones_.push_back(poly);
    }
    // 7
    {
        Polygon poly;
        poly.vertices.clear();
        point.x = 1064; point.y = 1346; point.z = 0; poly.vertices.push_back(point);
        point.x = 806;  point.y = 988;  point.z = 0; poly.vertices.push_back(point);
        point.x = 466;  point.y = 974;  point.z = 0; poly.vertices.push_back(point);
        point.x = 462;  point.y = 1052; point.z = 0; poly.vertices.push_back(point);
        point.x = 524;  point.y = 1050; point.z = 0; poly.vertices.push_back(point);
        point.x = 564;  point.y = 1152; point.z = 0; poly.vertices.push_back(point);
        point.x = 466;  point.y = 1152; point.z = 0; poly.vertices.push_back(point);
        point.x = 472;  point.y = 1180; point.z = 0; poly.vertices.push_back(point);
        point.x = 606;  point.y = 1176; point.z = 0; poly.vertices.push_back(point);
        point.x = 566;  point.y = 1050; point.z = 0; poly.vertices.push_back(point);
        point.x = 774;  point.y = 1046; point.z = 0; poly.vertices.push_back(point);
        point.x = 994;  point.y = 1354; point.z = 0; poly.vertices.push_back(point);
        forbidden_zones_.push_back(poly);
    }
}


bool RegionManager::isPointInPolygon(const geometry_msgs::msg::Point& point, const Polygon& polygon) {
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
    p.x = x; p.y = y; p.z = 0.0;
    return isPointInPolygon(p, red_highland_);
}

bool RegionManager::isInRedRegion(double x, double y) const {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = 0.0;
    return isPointInPolygon(p, red_region_);
}

bool RegionManager::isInCentralRegion(double x, double y) const {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = 0.0;
    return isPointInPolygon(p, central_region_);
}

bool RegionManager::isInBlueRegion(double x, double y) const {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = 0.0;
    return isPointInPolygon(p, blue_region_);
}

bool RegionManager::isInBlueHighland(double x, double y) const {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = 0.0;
    return isPointInPolygon(p, blue_highland_);
}

bool RegionManager::isInHeroDeployZone(double x, double y) const {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = 0.0;
    return isPointInPolygon(p, hero_deploy_zone_);
}

bool RegionManager::isInEnemyHeroDeployZone(double x, double y, int robot_id) const {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = 0.0;
    if (robot_id == 1) {  // 蓝方机器人，敌方为红方，使用红方部署区
        return isPointInPolygon(p, red_hero_deploy_zone_);
    } else {              // 红方机器人，敌方为蓝方，使用蓝方部署区
        return isPointInPolygon(p, hero_deploy_zone_);
    }
}

bool RegionManager::isInEnemyEngineerMiningZone(double x, double y, int robot_id) const {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = 0.0;
    if (robot_id == 1) {  // 蓝方，敌方工程为红方工程，使用红方取矿区
        return isPointInPolygon(p, red_engineer_mining_zone_);
    } else {              // 红方，敌方工程为蓝方工程，使用蓝方取矿区
        return isPointInPolygon(p, blue_engineer_mining_zone_);
    }
}

bool RegionManager::isInsideAllowedRegion(double x, double y) const {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = 0.0;
    return isPointInPolygon(p, allowed_region_);
}

bool RegionManager::isInForbiddenZone(double x, double y) const {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = 0.0;
    for (const auto& zone : forbidden_zones_) {
        if (isPointInPolygon(p, zone)) return true;
    }
    return false;
}

geometry_msgs::msg::Point RegionManager::adjustPointOutOfForbidden(const geometry_msgs::msg::Point& point,
                                                                   double robot_x, double robot_y) const {
    if (!isInForbiddenZone(point.x, point.y)) return point;

    const double step = 5.0; // cm
    double dx = point.x - robot_x;
    double dy = point.y - robot_y;
    double dist = std::hypot(dx, dy);
    if (dist < 0.001) {
        // 点与机器人重合，给出一个默认安全点（地图中心附近）
        geometry_msgs::msg::Point safe;
        safe.x = 1400.0; safe.y = 750.0;
        return safe;
    }
    double ux = dx / dist;
    double uy = dy / dist;
    geometry_msgs::msg::Point test = point;
    for (double d = 0; d <= dist; d += step) {
        test.x = point.x - ux * d;
        test.y = point.y - uy * d;
        if (!isInForbiddenZone(test.x, test.y)) {
            return test;
        }
    }
    // 如果整个线段都在禁止区，返回机器人位置（机器人理论上不在禁止区）
    geometry_msgs::msg::Point robot_pt;
    robot_pt.x = robot_x; robot_pt.y = robot_y;
    return robot_pt;
}

geometry_msgs::msg::Point RegionManager::clampPointToAllowedRegion(const geometry_msgs::msg::Point& point,
                                                                   double robot_x, double robot_y) const {
    geometry_msgs::msg::Point clamped = point;

    // 首先确保在允许区域内
    if (!isInsideAllowedRegion(point.x, point.y)) {
        geometry_msgs::msg::Point best_proj;
        double min_dist = 1e9;
        const auto& verts = allowed_region_.vertices;
        int n = verts.size();
        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            const auto& A = verts[i];
            const auto& B = verts[j];
            double dx = B.x - A.x;
            double dy = B.y - A.y;
            double len_sq = dx*dx + dy*dy;
            if (len_sq == 0) {
                double d = std::hypot(point.x - A.x, point.y - A.y);
                if (d < min_dist) { min_dist = d; best_proj = A; }
                continue;
            }
            double t = ((point.x - A.x)*dx + (point.y - A.y)*dy) / len_sq;
            t = std::max(0.0, std::min(1.0, t));
            geometry_msgs::msg::Point proj;
            proj.x = A.x + t * dx;
            proj.y = A.y + t * dy;
            double d = std::hypot(point.x - proj.x, point.y - proj.y);
            if (d < min_dist) {
                min_dist = d;
                best_proj = proj;
            }
        }
        clamped = best_proj;
    }

    // 其次确保不在禁止区域内
    if (isInForbiddenZone(clamped.x, clamped.y)) {
        clamped = adjustPointOutOfForbidden(clamped, robot_x, robot_y);
    }
    return clamped;
}

std::string RegionManager::getRegionName(double x, double y) const {
    if (isInRedHighland(x, y)) return "red_highland";
    if (isInRedRegion(x, y)) return "red_region";
    if (isInCentralRegion(x, y)) return "central_region";
    if (isInBlueRegion(x, y)) return "blue_region";
    if (isInBlueHighland(x, y)) return "blue_highland";
    if (isInHeroDeployZone(x, y)) return "hero_deploy_zone";
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

    auto pickNearest = [&](const std::vector<geometry_msgs::msg::Point>& points) -> geometry_msgs::msg::Point {
        if (points.empty()) return geometry_msgs::msg::Point();
        geometry_msgs::msg::Point best = points[0];
        double min_dist = std::hypot(robot_x - best.x, robot_y - best.y);
        for (size_t i = 1; i < points.size(); ++i) {
            double d = std::hypot(robot_x - points[i].x, robot_y - points[i].y);
            if (d < min_dist) { min_dist = d; best = points[i]; }
        }
        return best;
    };

    if (!same_region_points.empty()) {
        return pickNearest(same_region_points);
    }
    return pickNearest(hex_points);
}
