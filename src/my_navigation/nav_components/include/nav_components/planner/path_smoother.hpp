// nav_components/include/nav_components/planner/path_smoother.hpp
// 路径平滑器 - 基于 B样条的路径平滑
// 参考: Fast-Planner (HKUST)，适配 2D 差速底盘

#pragma once
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nav_components/planner/bspline.hpp"
#include "nav_components/planner/bspline_optimizer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>
#include <chrono>

namespace nav_components {

// 平滑参数
struct SmoothParams {
    double resample_interval = 0.3;  // 均匀重采样间隔 (m)，控制点密度
    int bspline_order = 3;           // B样条阶数 (固定为3，三次B样条)
    double output_resolution = 0.05; // 输出路径分辨率 (m)
    
    // 优化参数
    bool enable_optimization = true; // 是否启用 B样条优化
    double lambda_smooth = 10.0;     // 平滑项权重
    double lambda_collision = 5.0;   // 障碍物项权重
    double safe_dist = 0.3;          // 安全距离阈值 (m)
    int opt_max_iter = 50;           // 优化最大迭代次数
    
    // 曲率自适应（转角处离障碍物更远）
    bool curvature_adaptive = true;  // 启用曲率自适应
    double curvature_weight = 2.0;   // 曲率加权系数
    
    // 窄通道中心吸引（进出窄通道时走中间）
    bool narrow_passage_align = true;   // 启用窄通道中心吸引
    double narrow_passage_thresh = 0.6; // 窄通道判定阈值 (m)
    double lambda_align = 3.0;          // 中心吸引权重
    int align_entry_points = 2;         // 入口/出口各优化几个点 (1-3)
    int narrow_min_consecutive = 3;     // 最少连续几个点才算窄通道

    // 台阶段方向约束
    bool stair_segment_align = true;
    double lambda_stair_align = 6.0;
    int stair_align_expand_points = 2;
};

class PathSmoother {
public:
    PathSmoother() = default;
    explicit PathSmoother(const SmoothParams& params) : params_(params) {
        updateOptimizerParams();
    }
    
    void setParams(const SmoothParams& params) { 
        params_ = params; 
        updateOptimizerParams();
    }
    
    void setESDFCallback(ESDFCallback cb) { 
        optimizer_.setESDFCallback(cb); 
    }

    void setStairNormalCallback(StairNormalCallback cb) {
        optimizer_.setStairNormalCallback(cb);
    }
    
    // 平滑路径
    // 输入: A* 或其他规划器产生的原始路径
    // 输出: 平滑后的路径
    bool smooth(const nav_msgs::msg::Path& input, nav_msgs::msg::Path& output) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        if (input.poses.size() < 3) {
            output = input;
            return true;
        }
        
        // Step 1: 沿原始路径均匀重采样
        auto t1 = std::chrono::high_resolution_clock::now();
        std::vector<Eigen::Vector2d> waypoints = resamplePath(input, params_.resample_interval);
        auto t2 = std::chrono::high_resolution_clock::now();
        
        if (waypoints.size() < 4) {
            output = input;
            return true;
        }
        
        // Step 2: 参数化为 B样条控制点
        Eigen::MatrixXd ctrl_pts;
        double interval;
        if (!parameterizeToBspline(waypoints, ctrl_pts, interval)) {
            output = input;
            return true;
        }
        auto t3 = std::chrono::high_resolution_clock::now();
        
        // Step 2.5: B样条优化（可选）
        auto t_opt_start = std::chrono::high_resolution_clock::now();
        if (params_.enable_optimization) {
            optimizer_.optimize(ctrl_pts, params_.bspline_order);
        }
        auto t_opt_end = std::chrono::high_resolution_clock::now();
        
        // 保存控制点用于调试可视化
        last_ctrl_pts_.clear();
        last_ctrl_pts_.reserve(ctrl_pts.rows());
        for (int i = 0; i < ctrl_pts.rows(); ++i) {
            last_ctrl_pts_.emplace_back(ctrl_pts(i, 0), ctrl_pts(i, 1));
        }
        
        // Step 3: 构造 B样条并采样
        BSpline2D bspline(ctrl_pts, params_.bspline_order, interval);
        if (!bspline.isValid()) {
            output = input;
            return true;
        }
        
        double duration = bspline.getDuration();
        if (duration <= 0) {
            output = input;
            return true;
        }
        
        auto sampled = bspline.sample(params_.output_resolution / std::max(0.5, duration));
        auto t4 = std::chrono::high_resolution_clock::now();
        if (sampled.empty()) {
            output = input;
            return true;
        }
        
        // Step 4: 转换为 Path 消息
        output.header = input.header;
        output.poses.clear();
        output.poses.reserve(sampled.size());
        
        for (size_t i = 0; i < sampled.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = input.header;
            pose.pose.position.x = sampled[i].x();
            pose.pose.position.y = sampled[i].y();
            pose.pose.position.z = 0.0;
            
            // 计算航向角（基于前后点方向）
            if (i < sampled.size() - 1) {
                Eigen::Vector2d dir = sampled[i + 1] - sampled[i];
                double yaw = std::atan2(dir.y(), dir.x());
                pose.pose.orientation.z = std::sin(yaw / 2);
                pose.pose.orientation.w = std::cos(yaw / 2);
            } else if (i > 0) {
                // 最后一个点用前一个点的方向
                pose.pose.orientation = output.poses.back().pose.orientation;
            } else {
                pose.pose.orientation.w = 1.0;
            }
            
            output.poses.push_back(pose);
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        
        // 输出计时信息
        auto resample_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        auto interp_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
        auto opt_us = std::chrono::duration_cast<std::chrono::microseconds>(t_opt_end - t_opt_start).count();
        auto sample_us = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t_opt_end).count();
        auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        
        if (params_.enable_optimization) {
            RCLCPP_INFO(rclcpp::get_logger("path_smoother"),
                "Path smoothing: %zu pts -> %zu ctrl -> %zu out | "
                "resample: %.2fms, interp: %.2fms, opt: %.2fms, sample: %.2fms, total: %.2fms",
                input.poses.size(), waypoints.size(), output.poses.size(),
                resample_us / 1000.0, interp_us / 1000.0, opt_us / 1000.0, 
                sample_us / 1000.0, total_us / 1000.0);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("path_smoother"),
                "Path smoothing: %zu pts -> %zu ctrl -> %zu out | "
                "resample: %.2fms, interp: %.2fms, sample: %.2fms, total: %.2fms",
                input.poses.size(), waypoints.size(), output.poses.size(),
                resample_us / 1000.0, interp_us / 1000.0, sample_us / 1000.0, total_us / 1000.0);
        }
        
        return true;
    }
    
    // 获取最近一次平滑后的控制点（调试用）
    const std::vector<Eigen::Vector2d>& getLastControlPoints() const {
        return last_ctrl_pts_;
    }

private:
    void updateOptimizerParams() {
        BSplineOptParams opt_params;
        opt_params.lambda_smooth = params_.lambda_smooth;
        opt_params.lambda_collision = params_.lambda_collision;
        opt_params.safe_dist = params_.safe_dist;
        opt_params.max_iterations = params_.opt_max_iter;
        opt_params.curvature_adaptive = params_.curvature_adaptive;
        opt_params.curvature_weight = params_.curvature_weight;
        opt_params.narrow_passage_align = params_.narrow_passage_align;
        opt_params.narrow_passage_thresh = params_.narrow_passage_thresh;
        opt_params.lambda_align = params_.lambda_align;
        opt_params.align_entry_points = params_.align_entry_points;
        opt_params.narrow_min_consecutive = params_.narrow_min_consecutive;
        opt_params.stair_segment_align = params_.stair_segment_align;
        opt_params.lambda_stair_align = params_.lambda_stair_align;
        opt_params.stair_expand_points = params_.stair_align_expand_points;
        optimizer_.setParams(opt_params);
    }
    
    // 沿路径均匀重采样
    // 关键函数：将不均匀的 A* 路径点转换为均匀分布的控制点
    std::vector<Eigen::Vector2d> resamplePath(const nav_msgs::msg::Path& path, double interval) {
        std::vector<Eigen::Vector2d> result;
        if (path.poses.empty()) return result;
        
        // 提取所有点
        std::vector<Eigen::Vector2d> points;
        points.reserve(path.poses.size());
        for (const auto& pose : path.poses) {
            points.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }
        
        // 计算累计弧长
        std::vector<double> arc_length(points.size());
        arc_length[0] = 0;
        for (size_t i = 1; i < points.size(); ++i) {
            arc_length[i] = arc_length[i-1] + (points[i] - points[i-1]).norm();
        }
        double total_length = arc_length.back();
        
        if (total_length < interval) {
            // 路径太短，直接返回首尾两点
            result.push_back(points.front());
            result.push_back(points.back());
            return result;
        }
        
        // 均匀采样
        result.push_back(points.front());  // 起点
        
        double target_s = interval;
        size_t seg_idx = 0;  // 当前线段索引
        
        while (target_s < total_length - 1e-6) {
            // 找到包含 target_s 的线段
            while (seg_idx < points.size() - 1 && arc_length[seg_idx + 1] < target_s) {
                ++seg_idx;
            }
            
            if (seg_idx >= points.size() - 1) break;
            
            // 在线段上插值
            double seg_start = arc_length[seg_idx];
            double seg_end = arc_length[seg_idx + 1];
            double seg_length = seg_end - seg_start;
            
            if (seg_length > 1e-9) {
                double t = (target_s - seg_start) / seg_length;
                Eigen::Vector2d pt = points[seg_idx] * (1 - t) + points[seg_idx + 1] * t;
                result.push_back(pt);
            }
            
            target_s += interval;
        }
        
        result.push_back(points.back());  // 终点
        return result;
    }
    
    // 将路径点参数化为 B样条控制点
    // 使用三次 B样条插值：给定 K 个点，计算 K+2 个控制点使曲线穿过这些点
    bool parameterizeToBspline(const std::vector<Eigen::Vector2d>& points,
                               Eigen::MatrixXd& ctrl_pts,
                               double& interval) {
        int K = static_cast<int>(points.size());
        if (K < 4) {
            return false;
        }
        
        // 计算总弧长
        double total_length = 0;
        for (int i = 1; i < K; ++i) {
            total_length += (points[i] - points[i-1]).norm();
        }
        if (total_length < 1e-6) {
            return false;
        }
        
        // 节点间隔
        interval = total_length / (K - 1);
        
        int N = K + 2;  // 控制点数量
        
        // 构建系数矩阵 A (N x N)
        // 边界条件: P''(0) = 0 → P_0 - 2*P_1 + P_2 = 0
        // 内部点约束: (1/6)*P_{i} + (4/6)*P_{i+1} + (1/6)*P_{i+2} = Q_i
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N, N);
        Eigen::MatrixXd b = Eigen::MatrixXd::Zero(N, 2);
        
        // 第一行: 自然边界条件 P''(0) = 0
        A(0, 0) = 1.0;
        A(0, 1) = -2.0;
        A(0, 2) = 1.0;
        b.row(0).setZero();
        
        // 内部行 (1 到 K): B样条插值约束
        for (int i = 0; i < K; ++i) {
            A(i + 1, i) = 1.0 / 6.0;
            A(i + 1, i + 1) = 4.0 / 6.0;
            A(i + 1, i + 2) = 1.0 / 6.0;
            b.row(i + 1) = points[i].transpose();
        }
        
        // 最后一行: 自然边界条件 P''(end) = 0
        A(N - 1, N - 3) = 1.0;
        A(N - 1, N - 2) = -2.0;
        A(N - 1, N - 1) = 1.0;
        b.row(N - 1).setZero();
        
        // 使用 PartialPivLU 求解（比 ColPivHouseholderQR 快，适合方阵）
        ctrl_pts = A.partialPivLu().solve(b);
        
        return true;
    }
    
    SmoothParams params_;
    BSplineOptimizer optimizer_;
    std::vector<Eigen::Vector2d> last_ctrl_pts_;  // 调试用：最近一次的控制点
};

}  // namespace nav_components
