#include "sentry_decision/RegionManager.hpp"
#include "sentry_decision/GameConstants.hpp"
#include <cmath>
#include <algorithm>

using namespace GameConstants;

RegionManager::RegionManager() {
    geometry_msgs::msg::Point point;

    // 红方区域（保持原多边形）
    red_region_.vertices.clear();
    point.x = 54; point.y = 996; red_region_.vertices.push_back(point);
    point.x = 800; point.y = 992; red_region_.vertices.push_back(point);
    point.x = 1048; point.y = 1326; red_region_.vertices.push_back(point);
    point.x = 1368; point.y = 1342; red_region_.vertices.push_back(point);
    point.x = 1366; point.y = 1270; red_region_.vertices.push_back(point);
    point.x = 1262; point.y = 1246; red_region_.vertices.push_back(point);
    point.x = 926; point.y = 814; red_region_.vertices.push_back(point);
    point.x = 938; point.y = 628; red_region_.vertices.push_back(point);
    point.x = 1056; point.y = 284; red_region_.vertices.push_back(point);
    point.x = 1024; point.y = 306; red_region_.vertices.push_back(point);
    point.x = 754; point.y = 310; red_region_.vertices.push_back(point);
    point.x = 708; point.y = 408; red_region_.vertices.push_back(point);
    point.x = 394; point.y = 414; red_region_.vertices.push_back(point);
    point.x = 380; point.y = 50; red_region_.vertices.push_back(point);
    point.x = 192; point.y = 56; red_region_.vertices.push_back(point);
    point.x = 198; point.y = 316; red_region_.vertices.push_back(point);
    point.x = 52; point.y = 328; red_region_.vertices.push_back(point);

    // 蓝方区域
    blue_region_.vertices.clear();
    point.x = 2744; point.y = 1164; blue_region_.vertices.push_back(point);
    point.x = 2616; point.y = 1148; blue_region_.vertices.push_back(point);
    point.x = 2600; point.y = 1438; blue_region_.vertices.push_back(point);
    point.x = 2450; point.y = 1436; blue_region_.vertices.push_back(point);
    point.x = 2446; point.y = 1064; blue_region_.vertices.push_back(point);
    point.x = 2098; point.y = 1066; blue_region_.vertices.push_back(point);
    point.x = 2020; point.y = 1178; blue_region_.vertices.push_back(point);
    point.x = 1794; point.y = 1164; blue_region_.vertices.push_back(point);
    point.x = 1762; point.y = 1096; blue_region_.vertices.push_back(point);
    point.x = 1888; point.y = 808; blue_region_.vertices.push_back(point);
    point.x = 1890; point.y = 666; blue_region_.vertices.push_back(point);
    point.x = 1576; point.y = 218; blue_region_.vertices.push_back(point);
    point.x = 1572; point.y = 160; blue_region_.vertices.push_back(point);
    point.x = 1754; point.y = 166; blue_region_.vertices.push_back(point);
    point.x = 2004; point.y = 500; blue_region_.vertices.push_back(point);
    point.x = 2746; point.y = 506; blue_region_.vertices.push_back(point);

    // 红方英雄部署区
    red_hero_deploy_zone_.vertices.clear();
    point.x = 350; point.y = 1000; red_hero_deploy_zone_.vertices.push_back(point);
    point.x = 792; point.y = 972;  red_hero_deploy_zone_.vertices.push_back(point);
    point.x = 762; point.y = 1320; red_hero_deploy_zone_.vertices.push_back(point);
    point.x = 354; point.y = 1318; red_hero_deploy_zone_.vertices.push_back(point);

    // 蓝方英雄部署区
    blue_hero_deploy_zone_.vertices.clear();
    point.x = 2012; point.y = 518; blue_hero_deploy_zone_.vertices.push_back(point);
    point.x = 2478; point.y = 518; blue_hero_deploy_zone_.vertices.push_back(point);
    point.x = 2472; point.y = 174; blue_hero_deploy_zone_.vertices.push_back(point);
    point.x = 2018; point.y = 202; blue_hero_deploy_zone_.vertices.push_back(point);

    // 禁止区域（保留原国赛的7个）
    forbidden_zones_.clear();
    {
        Polygon poly;
        point.x = 1298; point.y = 948; poly.vertices.push_back(point);
        point.x = 1472; point.y = 892; poly.vertices.push_back(point);
        point.x = 1648; point.y = 712; poly.vertices.push_back(point);
        point.x = 1652; point.y = 672; poly.vertices.push_back(point);
        point.x = 1512; point.y = 540; poly.vertices.push_back(point);
        point.x = 1346; point.y = 590; poly.vertices.push_back(point);
        point.x = 1162; point.y = 770; poly.vertices.push_back(point);
        point.x = 1158; point.y = 808; poly.vertices.push_back(point);
        forbidden_zones_.push_back(poly);
    }
    {
        Polygon poly;
        point.x = 1334; point.y = 232; poly.vertices.push_back(point);
        point.x = 1542; point.y = 236; poly.vertices.push_back(point);
        point.x = 1566; point.y = 38;  poly.vertices.push_back(point);
        point.x = 1266; point.y = 38;  poly.vertices.push_back(point);
        point.x = 1270; point.y = 148; poly.vertices.push_back(point);
        point.x = 1336; point.y = 156; poly.vertices.push_back(point);
        forbidden_zones_.push_back(poly);
    }
    {
        Polygon poly;
        point.x = 1262; point.y = 1242; poly.vertices.push_back(point);
        point.x = 1254; point.y = 1442; poly.vertices.push_back(point);
        point.x = 1542; point.y = 1446; poly.vertices.push_back(point);
        point.x = 1544; point.y = 1342; poly.vertices.push_back(point);
        point.x = 1474; point.y = 1334; poly.vertices.push_back(point);
        point.x = 1480; point.y = 1254; poly.vertices.push_back(point);
        forbidden_zones_.push_back(poly);
    }
    {
        Polygon poly;
        point.x = 270; point.y = 832; poly.vertices.push_back(point);
        point.x = 216; point.y = 794; poly.vertices.push_back(point);
        point.x = 216; point.y = 696; poly.vertices.push_back(point);
        point.x = 270; point.y = 652; poly.vertices.push_back(point);
        point.x = 358; point.y = 680; poly.vertices.push_back(point);
        point.x = 366; point.y = 798; poly.vertices.push_back(point);
        point.x = 274; point.y = 850; poly.vertices.push_back(point);
        forbidden_zones_.push_back(poly);
    }
    {
        Polygon poly;
        point.x = 2526; point.y = 838; poly.vertices.push_back(point);
        point.x = 2438; point.y = 812; poly.vertices.push_back(point);
        point.x = 2432; point.y = 694; poly.vertices.push_back(point);
        point.x = 2530; point.y = 648; poly.vertices.push_back(point);
        point.x = 2594; point.y = 694; poly.vertices.push_back(point);
        point.x = 2592; point.y = 794; poly.vertices.push_back(point);
        forbidden_zones_.push_back(poly);
    }
    {
        Polygon poly;
        point.x = 1756; point.y = 142; poly.vertices.push_back(point);
        point.x = 2004; point.y = 492; poly.vertices.push_back(point);
        point.x = 2280; point.y = 496; poly.vertices.push_back(point);
        point.x = 2290; point.y = 516; poly.vertices.push_back(point);
        point.x = 2336; point.y = 508; poly.vertices.push_back(point);
        point.x = 2342; point.y = 440; poly.vertices.push_back(point);
        point.x = 2282; point.y = 432; poly.vertices.push_back(point);
        point.x = 2244; point.y = 334; poly.vertices.push_back(point);
        point.x = 2338; point.y = 326; poly.vertices.push_back(point);
        point.x = 2238; point.y = 310; poly.vertices.push_back(point);
        point.x = 2202; point.y = 310; poly.vertices.push_back(point);
        point.x = 2248; point.y = 440; poly.vertices.push_back(point);
        point.x = 2032; point.y = 440; poly.vertices.push_back(point);
        point.x = 1820; point.y = 138; poly.vertices.push_back(point);
        forbidden_zones_.push_back(poly);
    }
    {
        Polygon poly;
        point.x = 1064; point.y = 1346; poly.vertices.push_back(point);
        point.x = 806;  point.y = 988;  poly.vertices.push_back(point);
        point.x = 466;  point.y = 974;  poly.vertices.push_back(point);
        point.x = 462;  point.y = 1052; poly.vertices.push_back(point);
        point.x = 524;  point.y = 1050; poly.vertices.push_back(point);
        point.x = 564;  point.y = 1152; poly.vertices.push_back(point);
        point.x = 466;  point.y = 1152; poly.vertices.push_back(point);
        point.x = 472;  point.y = 1180; poly.vertices.push_back(point);
        point.x = 606;  point.y = 1176; poly.vertices.push_back(point);
        point.x = 566;  point.y = 1050; poly.vertices.push_back(point);
        point.x = 774;  point.y = 1046; poly.vertices.push_back(point);
        point.x = 994;  point.y = 1354; poly.vertices.push_back(point);
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

bool RegionManager::isInRedRegion(double x, double y) const {
    geometry_msgs::msg::Point p; p.x = x; p.y = y;
    return isPointInPolygon(p, red_region_);
}

bool RegionManager::isInBlueRegion(double x, double y) const {
    geometry_msgs::msg::Point p; p.x = x; p.y = y;
    return isPointInPolygon(p, blue_region_);
}

bool RegionManager::isOurHeroInDeployZone(double hero_x, double hero_y, int robot_id) const {
    geometry_msgs::msg::Point p; p.x = hero_x; p.y = hero_y;
    if (isBlueRobotId(robot_id)) {  // 蓝方机器人 -> 蓝方英雄部署区
        return isPointInPolygon(p, blue_hero_deploy_zone_);
    } else {              // 红方机器人 -> 红方英雄部署区
        return isPointInPolygon(p, red_hero_deploy_zone_);
    }
}

bool RegionManager::isInForbiddenZone(double x, double y) const {
    geometry_msgs::msg::Point p; p.x = x; p.y = y;
    for (const auto& zone : forbidden_zones_) {
        if (isPointInPolygon(p, zone)) return true;
    }
    return false;
}

// 将点投影到多边形边界上最近点
static geometry_msgs::msg::Point projectToPolygon(const geometry_msgs::msg::Point& point, const Polygon& polygon) {
    geometry_msgs::msg::Point best;
    double min_dist = 1e9;
    const auto& verts = polygon.vertices;
    int n = verts.size();
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        double dx = verts[j].x - verts[i].x;
        double dy = verts[j].y - verts[i].y;
        double len_sq = dx*dx + dy*dy;
        if (len_sq == 0.0) {
            double d = std::hypot(point.x - verts[i].x, point.y - verts[i].y);
            if (d < min_dist) { min_dist = d; best = verts[i]; }
            continue;
        }
        double t = ((point.x - verts[i].x)*dx + (point.y - verts[i].y)*dy) / len_sq;
        t = std::max(0.0, std::min(1.0, t));
        geometry_msgs::msg::Point proj;
        proj.x = verts[i].x + t * dx;
        proj.y = verts[i].y + t * dy;
        double d = std::hypot(point.x - proj.x, point.y - proj.y);
        if (d < min_dist) { min_dist = d; best = proj; }
    }
    return best;
}

geometry_msgs::msg::Point RegionManager::adjustPointOutOfForbidden(const geometry_msgs::msg::Point& point,
                                                                   double robot_x, double robot_y) const {
    if (!isInForbiddenZone(point.x, point.y)) return point;
    const double step = 5.0;
    double dx = point.x - robot_x;
    double dy = point.y - robot_y;
    double dist = std::hypot(dx, dy);
    if (dist < 0.001) {
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
        if (!isInForbiddenZone(test.x, test.y)) return test;
    }
    geometry_msgs::msg::Point robot_pt; robot_pt.x = robot_x; robot_pt.y = robot_y;
    return robot_pt;
}

geometry_msgs::msg::Point RegionManager::clampToTeamRegion(const geometry_msgs::msg::Point& point,
                                                           int robot_id,
                                                           double robot_x, double robot_y) const {
    geometry_msgs::msg::Point clamped = point;
    const Polygon& team_region = isBlueRobotId(robot_id) ? blue_region_ : red_region_;
    // 若点不在团队区域内，投影到边界
    if (!isPointInPolygon(clamped, team_region)) {
        clamped = projectToPolygon(clamped, team_region);
    }
    // 避开禁止区域
    if (isInForbiddenZone(clamped.x, clamped.y)) {
        clamped = adjustPointOutOfForbidden(clamped, robot_x, robot_y);
    }
    return clamped;
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

geometry_msgs::msg::Point RegionManager::findSameTeamHexPoint(double target_x, double target_y,
                                                             double robot_x, double robot_y,
                                                             int robot_id) const {
    auto hex_points = calculateHexagonPoints(target_x, target_y, 200.0);
    const Polygon& team_region = isBlueRobotId(robot_id) ? blue_region_ : red_region_;

    std::vector<geometry_msgs::msg::Point> same_region_points;
    for (const auto& point : hex_points) {
        if (isPointInPolygon(point, team_region)) {
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

std::string RegionManager::getRegionName(double x, double y) const {
    if (isInRedRegion(x, y)) return "red_region";
    if (isInBlueRegion(x, y)) return "blue_region";
    return "unknown";
}
