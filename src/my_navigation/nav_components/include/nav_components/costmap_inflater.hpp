// nav_components/include/nav_components/costmap_inflater.hpp
// Costmap 膨胀器 - 基于欧氏距离的代价膨胀

#pragma once
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

namespace nav_components {

// 衰减类型
enum class DecayType {
    LINEAR,      // 线性衰减
    EXPONENTIAL  // 指数衰减
};

// 膨胀参数
struct InflationParams {
    double inflation_radius = 0.5;   // 膨胀半径(m)
    double inscribed_radius = 0.2;   // 内切圆半径(m)
    double cost_scaling = 3.0;       // 指数衰减因子
    DecayType decay_type = DecayType::EXPONENTIAL;
};

class CostmapInflater {
public:
    // 方式1：从 ESDF 距离场生成（最高效，推荐）
    static nav_msgs::msg::OccupancyGrid::SharedPtr fromEsdf(
        const std::vector<float>& esdf,
        int width, int height, double resolution,
        double origin_x, double origin_y,
        const InflationParams& params)
    {
        auto costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        costmap->header.frame_id = "map";
        costmap->info.resolution = resolution;
        costmap->info.width = width;
        costmap->info.height = height;
        costmap->info.origin.position.x = origin_x;
        costmap->info.origin.position.y = origin_y;
        
        costmap->data.resize(width * height);
        for (int i = 0; i < width * height; i++) {
            costmap->data[i] = computeCost(esdf[i], params);
        }
        return costmap;
    }
    
    // 方式2：从 OccupancyGrid 独立计算（使用 Meijster EDT，O(W×H)）
    static nav_msgs::msg::OccupancyGrid::SharedPtr inflate(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& source,
        const InflationParams& params,
        int8_t obstacle_threshold = 50)
    {
        if (!source || source->data.empty()) return nullptr;
        
        int width = source->info.width;
        int height = source->info.height;
        double resolution = source->info.resolution;
        
        // 计算欧氏距离场
        std::vector<float> dist(width * height);
        computeEDT(source->data, width, height, resolution, obstacle_threshold, dist);
        
        // 转换为 costmap
        auto costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        costmap->header = source->header;
        costmap->info = source->info;
        costmap->data.resize(width * height);
        
        for (int i = 0; i < width * height; i++) {
            costmap->data[i] = computeCost(dist[i], params);
        }
        return costmap;
    }

private:
    // Meijster EDT 算法（O(W×H) 真正欧氏距离）
    static void computeEDT(const std::vector<int8_t>& occ, int w, int h,
                           double res, int8_t thresh, std::vector<float>& dist)
    {
        const float INF = 1e9f;
        std::vector<float> temp(w * h);
        
        // Pass 1: 沿 Y 方向
        for (int x = 0; x < w; x++) {
            for (int y = 0; y < h; y++) {
                int idx = y * w + x;
                temp[idx] = (occ[idx] >= thresh || occ[idx] < 0) ? 0.0f : INF;
            }
            for (int y = 1; y < h; y++) {
                int idx = y * w + x;
                temp[idx] = std::min(temp[idx], temp[idx - w] + 1.0f);
            }
            for (int y = h - 2; y >= 0; y--) {
                int idx = y * w + x;
                temp[idx] = std::min(temp[idx], temp[idx + w] + 1.0f);
            }
        }
        
        // Pass 2: 沿 X 方向（抛物线下包络）
        std::vector<int> v(w);
        std::vector<float> z(w + 1);
        
        for (int y = 0; y < h; y++) {
            int k = 0;
            v[0] = 0;
            z[0] = -INF;
            z[1] = INF;
            
            for (int x = 1; x < w; x++) {
                float fx = temp[y * w + x];
                float fv = temp[y * w + v[k]];
                float s = ((fx * fx + x * x) - (fv * fv + v[k] * v[k])) / (2.0f * (x - v[k]));
                
                while (s <= z[k]) {
                    k--;
                    fv = temp[y * w + v[k]];
                    s = ((fx * fx + x * x) - (fv * fv + v[k] * v[k])) / (2.0f * (x - v[k]));
                }
                k++;
                v[k] = x;
                z[k] = s;
                z[k + 1] = INF;
            }
            
            k = 0;
            for (int x = 0; x < w; x++) {
                while (z[k + 1] < x) k++;
                float dx = x - v[k];
                float dy = temp[y * w + v[k]];
                dist[y * w + x] = std::sqrt(dx * dx + dy * dy) * res;
            }
        }
    }
    
    static int8_t computeCost(float distance, const InflationParams& params) {
        if (distance <= 0.0f) return 100;
        if (distance <= params.inscribed_radius) return 99;
        if (distance >= params.inflation_radius) return 0;
        
        double ratio = (distance - params.inscribed_radius) / 
                       (params.inflation_radius - params.inscribed_radius);
        double cost = (params.decay_type == DecayType::LINEAR) 
            ? 98.0 * (1.0 - ratio)
            : 98.0 * std::exp(-params.cost_scaling * ratio);
        
        return static_cast<int8_t>(std::clamp(cost, 1.0, 98.0));
    }
};

}  // namespace nav_components
