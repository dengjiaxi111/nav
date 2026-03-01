// nav_components/include/nav_components/static_map_loader.hpp
// 静态地图加载器 - 读取 yaml + pgm 格式的地图文件

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "nav_components/map_image_io.hpp"
#include <string>

namespace nav_components {

class StaticMapLoader {
public:
    // 从 yaml 文件加载地图
    static nav_msgs::msg::OccupancyGrid::SharedPtr load(
        const std::string& yaml_path,
        rclcpp::Logger logger = rclcpp::get_logger("map_loader"))
    {
        auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();

        MapImageData map_img;
        if (!MapImageIO::loadYamlAndPGM(yaml_path, map_img, logger)) {
            return nullptr;
        }

        // 填充 OccupancyGrid
        map->header.frame_id = "map";
        map->header.stamp = rclcpp::Clock().now();
        map->info.resolution = map_img.resolution;
        map->info.width = map_img.width;
        map->info.height = map_img.height;
        map->info.origin.position.x = map_img.origin_x;
        map->info.origin.position.y = map_img.origin_y;
        map->info.origin.position.z = 0.0;
        
        // 转换像素到占据值
        map->data.resize(map_img.width * map_img.height);
        for (int y = 0; y < map_img.height; y++) {
            for (int x = 0; x < map_img.width; x++) {
                // PGM 坐标系 y 轴翻转
                int pgm_idx = (map_img.height - 1 - y) * map_img.width + x;
                int map_idx = y * map_img.width + x;
                
                double pixel = map_img.pixels[pgm_idx] / static_cast<double>(map_img.max_val);
                if (map_img.negate) pixel = 1.0 - pixel;
                
                // PGM: 白色(高值)=free, 黑色(低值)=occupied
                // OccupancyGrid: 0=free, 100=occupied
                // 所以：pixel 高 -> free(0), pixel 低 -> occupied(100)
                if (pixel <= map_img.free_thresh) {
                    map->data[map_idx] = 100;  // 占据 (黑色像素)
                } else if (pixel >= map_img.occupied_thresh) {
                    map->data[map_idx] = 0;    // 空闲 (白色像素)
                } else {
                    map->data[map_idx] = -1;   // 未知 (灰色像素)
                }
            }
        }
        
        RCLCPP_INFO(logger, "地图加载成功: %dx%d, 分辨率: %.3f",
                    map_img.width, map_img.height, map_img.resolution);
        return map;
    }
};

}  // namespace nav_components
