// nav_components/include/nav_components/static_map_loader.hpp
// 静态地图加载器 - 读取 yaml + pgm 格式的地图文件

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace nav_components {

class StaticMapLoader {
public:
    // 从 yaml 文件加载地图
    static nav_msgs::msg::OccupancyGrid::SharedPtr load(
        const std::string& yaml_path,
        rclcpp::Logger logger = rclcpp::get_logger("map_loader"))
    {
        auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        
        // 解析 yaml
        YAML::Node config;
        try {
            config = YAML::LoadFile(yaml_path);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger, "无法加载 yaml: %s", e.what());
            return nullptr;
        }
        
        // 获取图片路径（相对于yaml文件）
        std::string image_path = config["image"].as<std::string>();
        if (image_path[0] != '/') {
            size_t pos = yaml_path.find_last_of('/');
            if (pos != std::string::npos) {
                image_path = yaml_path.substr(0, pos + 1) + image_path;
            }
        }
        
        double resolution = config["resolution"].as<double>();
        auto origin = config["origin"].as<std::vector<double>>();
        double occupied_thresh = config["occupied_thresh"].as<double>(0.65);
        double free_thresh = config["free_thresh"].as<double>(0.196);
        bool negate = config["negate"].as<int>(0) != 0;
        
        // 加载 PGM 图片
        int width, height, max_val;
        std::vector<uint8_t> pixels;
        if (!loadPGM(image_path, width, height, max_val, pixels, logger)) {
            return nullptr;
        }
        
        // 填充 OccupancyGrid
        map->header.frame_id = "map";
        map->header.stamp = rclcpp::Clock().now();
        map->info.resolution = resolution;
        map->info.width = width;
        map->info.height = height;
        map->info.origin.position.x = origin[0];
        map->info.origin.position.y = origin[1];
        map->info.origin.position.z = 0.0;
        
        // 转换像素到占据值
        map->data.resize(width * height);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                // PGM 坐标系 y 轴翻转
                int pgm_idx = (height - 1 - y) * width + x;
                int map_idx = y * width + x;
                
                double pixel = pixels[pgm_idx] / static_cast<double>(max_val);
                if (negate) pixel = 1.0 - pixel;
                
                // PGM: 白色(高值)=free, 黑色(低值)=occupied
                // OccupancyGrid: 0=free, 100=occupied
                // 所以：pixel 高 -> free(0), pixel 低 -> occupied(100)
                if (pixel <= free_thresh) {
                    map->data[map_idx] = 100;  // 占据 (黑色像素)
                } else if (pixel >= occupied_thresh) {
                    map->data[map_idx] = 0;    // 空闲 (白色像素)
                } else {
                    map->data[map_idx] = -1;   // 未知 (灰色像素)
                }
            }
        }
        
        RCLCPP_INFO(logger, "地图加载成功: %dx%d, 分辨率: %.3f", width, height, resolution);
        return map;
    }

private:
    static bool loadPGM(const std::string& path, int& width, int& height, 
                        int& max_val, std::vector<uint8_t>& data,
                        rclcpp::Logger logger)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file) {
            RCLCPP_ERROR(logger, "无法打开图片: %s", path.c_str());
            return false;
        }
        
        std::string magic;
        file >> magic;
        if (magic != "P5") {
            RCLCPP_ERROR(logger, "仅支持 PGM P5 格式，当前: %s", magic.c_str());
            return false;
        }
        
        // 跳过注释
        char c;
        file.get(c);
        while (file.peek() == '#') {
            std::string comment;
            std::getline(file, comment);
        }
        
        file >> width >> height >> max_val;
        file.get(c);  // 跳过换行
        
        data.resize(width * height);
        file.read(reinterpret_cast<char*>(data.data()), width * height);
        
        return true;
    }
};

}  // namespace nav_components
