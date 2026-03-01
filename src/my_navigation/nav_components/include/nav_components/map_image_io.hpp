// nav_components/include/nav_components/map_image_io.hpp
// 地图图像读取工具 - 统一解析 yaml + pgm(P5)

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace nav_components {

struct MapImageData {
    std::string yaml_path;
    std::string image_path;

    double resolution = 0.05;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_z = 0.0;

    double occupied_thresh = 0.65;
    double free_thresh = 0.196;
    bool negate = false;

    int width = 0;
    int height = 0;
    int max_val = 255;
    std::vector<uint8_t> pixels;
};

class MapImageIO {
public:
    static bool loadYamlAndPGM(const std::string& yaml_path,
                               MapImageData& out,
                               rclcpp::Logger logger = rclcpp::get_logger("map_image_io")) {
        YAML::Node config;
        try {
            config = YAML::LoadFile(yaml_path);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger, "无法加载 yaml: %s", e.what());
            return false;
        }

        if (!config["image"] || !config["resolution"] || !config["origin"]) {
            RCLCPP_ERROR(logger, "yaml 缺少必要字段(image/resolution/origin): %s", yaml_path.c_str());
            return false;
        }

        std::string image_path = config["image"].as<std::string>();
        image_path = resolveImagePath(yaml_path, image_path);

        auto origin = config["origin"].as<std::vector<double>>();
        if (origin.size() < 2) {
            RCLCPP_ERROR(logger, "origin 至少需要2个元素: %s", yaml_path.c_str());
            return false;
        }

        int width = 0;
        int height = 0;
        int max_val = 255;
        std::vector<uint8_t> pixels;
        if (!loadPGM(image_path, width, height, max_val, pixels, logger)) {
            return false;
        }

        out.yaml_path = yaml_path;
        out.image_path = image_path;
        out.resolution = config["resolution"].as<double>();
        out.origin_x = origin[0];
        out.origin_y = origin[1];
        out.origin_z = origin.size() > 2 ? origin[2] : 0.0;

        out.occupied_thresh = config["occupied_thresh"].as<double>(0.65);
        out.free_thresh = config["free_thresh"].as<double>(0.196);
        out.negate = config["negate"].as<int>(0) != 0;

        out.width = width;
        out.height = height;
        out.max_val = max_val;
        out.pixels = std::move(pixels);
        return true;
    }

private:
    static std::string resolveImagePath(const std::string& yaml_path,
                                        const std::string& image_path_raw) {
        if (!image_path_raw.empty() && image_path_raw[0] == '/') {
            return image_path_raw;
        }

        size_t pos = yaml_path.find_last_of('/');
        if (pos == std::string::npos) {
            return image_path_raw;
        }
        return yaml_path.substr(0, pos + 1) + image_path_raw;
    }

    static bool loadPGM(const std::string& path, int& width, int& height,
                        int& max_val, std::vector<uint8_t>& data,
                        rclcpp::Logger logger) {
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

        char c;
        file.get(c);
        while (file.peek() == '#') {
            std::string comment;
            std::getline(file, comment);
        }

        file >> width >> height >> max_val;
        file.get(c);

        if (width <= 0 || height <= 0 || max_val <= 0 || max_val > 255) {
            RCLCPP_ERROR(logger, "PGM 头信息非法: w=%d h=%d max=%d", width, height, max_val);
            return false;
        }

        data.resize(static_cast<size_t>(width * height));
        file.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(width * height));
        if (!file) {
            RCLCPP_ERROR(logger, "读取 PGM 像素失败: %s", path.c_str());
            return false;
        }

        return true;
    }
};

}  // namespace nav_components
