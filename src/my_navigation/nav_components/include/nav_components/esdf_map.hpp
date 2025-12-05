// nav_components/include/nav_components/esdf_map.hpp
// ESDF (Euclidean Signed Distance Field) 地图实现

#pragma once
#include <nav_core/map_interface.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

namespace nav_components {

class EsdfMap : public nav_core::MapInterface {
public:
    // 从 OccupancyGrid 构建 ESDF
    void buildFromOccupancy(const nav_msgs::msg::OccupancyGrid::SharedPtr& grid,
                            int8_t obstacle_threshold = 50) {
        if (!grid || grid->data.empty()) return;
        
        width_ = grid->info.width;
        height_ = grid->info.height;
        resolution_ = grid->info.resolution;
        origin_x_ = grid->info.origin.position.x;
        origin_y_ = grid->info.origin.position.y;
        
        // 分配距离场（米为单位）
        distance_.resize(width_ * height_);
        
        // 使用 Meijster EDT 算法计算距离场
        computeEDT(grid->data, obstacle_threshold);
    }
    
    // MapInterface 接口实现
    nav_core::MapType type() const override { return nav_core::MapType::ESDF; }
    double resolution() const override { return resolution_; }
    bool hasDistance() const override { return true; }
    
    void getBounds(double& min_x, double& min_y, 
                   double& max_x, double& max_y) const override {
        min_x = origin_x_;
        min_y = origin_y_;
        max_x = origin_x_ + width_ * resolution_;
        max_y = origin_y_ + height_ * resolution_;
    }
    
    nav_core::MapQuery query(double x, double y) const override {
        nav_core::MapQuery q;
        
        int ix = static_cast<int>((x - origin_x_) / resolution_);
        int iy = static_cast<int>((y - origin_y_) / resolution_);
        
        if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_) {
            q.valid = false;
            return q;
        }
        
        q.valid = true;
        q.distance = distance_[iy * width_ + ix];
        q.occupied = (q.distance <= 0.0);
        q.cost = distanceToCost(q.distance);
        return q;
    }
    
    // 直接获取距离值（高性能路径，避免构造 MapQuery）
    float getDistanceFast(int ix, int iy) const {
        if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_) {
            return -std::numeric_limits<float>::max();
        }
        return distance_[iy * width_ + ix];
    }
    
    // 双线性插值获取距离（平滑版本）
    double getDistanceInterp(double x, double y) const {
        double fx = (x - origin_x_) / resolution_;
        double fy = (y - origin_y_) / resolution_;
        
        int ix = static_cast<int>(fx);
        int iy = static_cast<int>(fy);
        
        if (ix < 0 || ix >= width_ - 1 || iy < 0 || iy >= height_ - 1) {
            // 边界返回最近格子的值
            ix = std::clamp(ix, 0, width_ - 1);
            iy = std::clamp(iy, 0, height_ - 1);
            return static_cast<double>(distance_[iy * width_ + ix]);
        }
        
        // 双线性插值
        double tx = fx - ix;
        double ty = fy - iy;
        
        double d00 = distance_[iy * width_ + ix];
        double d10 = distance_[iy * width_ + (ix + 1)];
        double d01 = distance_[(iy + 1) * width_ + ix];
        double d11 = distance_[(iy + 1) * width_ + (ix + 1)];
        
        return (1 - tx) * (1 - ty) * d00 + tx * (1 - ty) * d10 
             + (1 - tx) * ty * d01 + tx * ty * d11;
    }
    
    // 二次插值获取平滑距离和梯度（解决峡谷中心梯度无效化问题）
    // 参考: 2×3 方格三点拟合二次函数
    // 返回: distance, grad_x, grad_y
    bool getDistanceAndGradient(double x, double y, 
                                double& dist, double& grad_x, double& grad_y) const {
        // 转换到栅格坐标（连续）
        double fx = (x - origin_x_) / resolution_;
        double fy = (y - origin_y_) / resolution_;
        
        int ix = static_cast<int>(fx);
        int iy = static_cast<int>(fy);
        
        // 边界检查（需要额外1格用于插值）
        if (ix < 1 || ix >= width_ - 1 || iy < 1 || iy >= height_ - 1) {
            dist = 0.0;
            grad_x = grad_y = 0.0;
            return false;
        }
        
        // 相对于 cell 中心的偏移 [-0.5, 0.5]
        double dx = fx - ix - 0.5;
        double dy = fy - iy - 0.5;
        
        // X 方向：三点二次插值 f(x) = ax² + bx + c
        float d_l = distance_[iy * width_ + (ix - 1)];  // left
        float d_c = distance_[iy * width_ + ix];        // center
        float d_r = distance_[iy * width_ + (ix + 1)];  // right
        
        // 二次拟合: a = (d_l + d_r)/2 - d_c, b = (d_r - d_l)/2, c = d_c
        double ax = 0.5 * (d_l + d_r) - d_c;
        double bx = 0.5 * (d_r - d_l);
        double dist_x = ax * dx * dx + bx * dx + d_c;
        grad_x = (2.0 * ax * dx + bx) / resolution_;
        
        // Y 方向：三点二次插值
        float d_b = distance_[(iy - 1) * width_ + ix];  // bottom
        float d_t = distance_[(iy + 1) * width_ + ix];  // top
        
        double ay = 0.5 * (d_b + d_t) - d_c;
        double by = 0.5 * (d_t - d_b);
        double dist_y = ay * dy * dy + by * dy + d_c;
        grad_y = (2.0 * ay * dy + by) / resolution_;
        
        // 距离取两个方向的平均
        dist = 0.5 * (dist_x + dist_y);
        return true;
    }
    
    // 简化接口：仅获取梯度（用于轨迹优化）
    bool getGradient(double x, double y, double& grad_x, double& grad_y) const {
        double dist;
        return getDistanceAndGradient(x, y, dist, grad_x, grad_y);
    }

    // 生成可视化用的 OccupancyGrid
    nav_msgs::msg::OccupancyGrid::SharedPtr toOccupancyGrid(
        double max_vis_dist = 2.0) const {
        
        auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        grid->header.frame_id = "map";
        grid->info.resolution = resolution_;
        grid->info.width = width_;
        grid->info.height = height_;
        grid->info.origin.position.x = origin_x_;
        grid->info.origin.position.y = origin_y_;
        
        grid->data.resize(width_ * height_);
        for (size_t i = 0; i < distance_.size(); i++) {
            float d = distance_[i];
            if (d <= 0.0f) {
                grid->data[i] = 100;  // 障碍物内部
            } else {
                // 距离映射: 0=近障碍物(深色), 0=远(浅色)
                int val = static_cast<int>(100.0 * (1.0 - std::min(d / max_vis_dist, 1.0)));
                grid->data[i] = std::clamp(val, 0, 99);
            }
        }
        return grid;
    }
    
    int width() const { return width_; }
    int height() const { return height_; }

private:
    // Meijster 二维 EDT 算法（O(n) 复杂度）
    // 参考: A. Meijster et al., "A General Algorithm for Computing Distance Transforms"
    void computeEDT(const std::vector<int8_t>& occ, int8_t threshold) {
        const float INF = 1e9f;
        std::vector<float> temp(width_ * height_);
        
        // 第一遍：沿 Y 方向（列）计算一维距离
        for (int x = 0; x < width_; x++) {
            // 初始化：障碍物=0，空闲=INF
            for (int y = 0; y < height_; y++) {
                int idx = y * width_ + x;
                int8_t val = occ[idx];
                temp[idx] = (val >= threshold || val < 0) ? 0.0f : INF;
            }
            
            // 前向扫描
            for (int y = 1; y < height_; y++) {
                int idx = y * width_ + x;
                if (temp[idx] > temp[idx - width_] + 1.0f) {
                    temp[idx] = temp[idx - width_] + 1.0f;
                }
            }
            
            // 后向扫描
            for (int y = height_ - 2; y >= 0; y--) {
                int idx = y * width_ + x;
                if (temp[idx] > temp[idx + width_] + 1.0f) {
                    temp[idx] = temp[idx + width_] + 1.0f;
                }
            }
        }
        
        // 第二遍：沿 X 方向（行）计算欧氏距离
        std::vector<int> s(width_);    // 抛物线位置
        std::vector<float> t(width_);  // 抛物线交点
        
        for (int y = 0; y < height_; y++) {
            int q = 0;
            s[0] = 0;
            t[0] = -INF;
            
            // 构建下包络线
            for (int x = 1; x < width_; x++) {
                float fy_x = temp[y * width_ + x];
                while (q >= 0) {
                    float fy_s = temp[y * width_ + s[q]];
                    float sep = computeSep(s[q], fy_s * fy_s, x, fy_x * fy_x);
                    if (sep > t[q]) {
                        break;
                    }
                    q--;
                }
                q++;
                s[q] = x;
                t[q] = computeSep(s[q - 1], temp[y * width_ + s[q - 1]] * temp[y * width_ + s[q - 1]],
                                  x, fy_x * fy_x);
            }
            t[q + 1] = INF;
            
            // 填充结果
            int k = 0;
            for (int x = 0; x < width_; x++) {
                while (t[k + 1] < static_cast<float>(x)) k++;
                float dx = static_cast<float>(x - s[k]);
                float fy = temp[y * width_ + s[k]];
                distance_[y * width_ + x] = std::sqrt(dx * dx + fy * fy) * resolution_;
            }
        }
    }
    
    // 计算两个抛物线的交点 x 坐标
    static float computeSep(int p, float fp2, int q, float fq2) {
        return (fq2 - fp2 + static_cast<float>(q * q - p * p)) / 
               (2.0f * static_cast<float>(q - p));
    }
    
    // 距离到代价的映射（用于规划）
    static double distanceToCost(double dist) {
        if (dist <= 0.0) return 1.0;           // 障碍物
        if (dist >= 1.0) return 0.0;           // 安全区
        return 1.0 - dist;                     // 线性衰减
    }
    
    std::vector<float> distance_;  // 距离场（米）
    int width_ = 0, height_ = 0;
    double resolution_ = 0.0;
    double origin_x_ = 0.0, origin_y_ = 0.0;
};

}  // namespace nav_components
