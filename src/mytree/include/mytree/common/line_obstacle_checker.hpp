#pragma once

#include <geometry_msgs/msg/point.hpp>

namespace myBT
{

// 将 world 坐标转换为地图索引（成功返回 true）
inline bool worldToMap(
  double wx, double wy,
  const nav_msgs::msg::OccupancyGrid & map,
  int & mx, int & my)
{
  double origin_x = map.info.origin.position.x;
  double origin_y = map.info.origin.position.y;
  double resolution = map.info.resolution;

  if (wx < origin_x || wy < origin_y) {
    return false;
  }

  mx = static_cast<int>((wx - origin_x) / resolution);
  my = static_cast<int>((wy - origin_y) / resolution);

  if (mx >= 0 && mx < static_cast<int>(map.info.width) &&
      my >= 0 && my < static_cast<int>(map.info.height)) {
    return true;
  }

  return false;
}

// 获取某栅格索引对应的代价值（0~100, -1）
inline int getCost(
  const nav_msgs::msg::OccupancyGrid & map,
  int mx, int my)
{
  int index = my * map.info.width + mx;
  return map.data[index];
}


// 检查从 start 到 end 的直线路径上是否存在障碍物
inline bool isLineObstacleFree(
  const std::pair<double,double> & start,
  const std::pair<double,double> & end,
  const nav_msgs::msg::OccupancyGrid &costmap,
  unsigned char lethal_cost = 99)
{
    int x0, y0, x1, y1;
    if (!worldToMap(start.first, start.second, costmap, x0, y0) ||
        !worldToMap(end.first, end.second, costmap, x1, y1))
    {
        return false;
    }

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int x = x0;
    int y = y0;

    while (true)
    {
        int cost = getCost(costmap, x, y);
        if (cost >= lethal_cost && cost != -1) {
        return false;
        }

        if (x == x1 && y == y1) break;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx)  { err += dx; y += sy; }
    }

    return true;
}

}  
