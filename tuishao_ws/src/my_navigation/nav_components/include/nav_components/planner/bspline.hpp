// nav_components/include/nav_components/planner/bspline.hpp
// 2D 均匀 B样条曲线 - 参考 Fast-Planner 简化实现

#pragma once
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

namespace nav_components {

// 2D 均匀三次 B样条
// 参考: Fast-Planner (HKUST) NonUniformBspline，简化为2D版本
class BSpline2D {
public:
    BSpline2D() = default;
    
    // 从控制点和节点间隔构造
    // points: N×2 矩阵，每行一个控制点 (x, y)
    // order: B样条阶数（通常为3，三次B样条）
    // interval: 节点间隔 Δt
    BSpline2D(const Eigen::MatrixXd& points, int order, double interval) {
        setUniformBspline(points, order, interval);
    }
    
    void setUniformBspline(const Eigen::MatrixXd& points, int order, double interval) {
        if (points.rows() < 2 || points.cols() < 2) {
            valid_ = false;
            return;
        }
        
        control_points_ = points;
        p_ = order;
        interval_ = interval;
        n_ = static_cast<int>(points.rows()) - 1;  // n+1 个控制点
        m_ = n_ + p_ + 1;                          // m+1 个节点
        
        // 需要至少 p+1 个控制点
        if (n_ < p_) {
            valid_ = false;
            return;
        }
        
        // 构造均匀节点向量
        u_ = Eigen::VectorXd::Zero(m_ + 1);
        for (int i = 0; i <= m_; ++i) {
            u_(i) = (i - p_) * interval_;
        }
        
        valid_ = true;
    }
    
    bool isValid() const { return valid_; }
    
    // De Boor 算法求值（参数 u ∈ [u_p, u_{m-p}]）
    Eigen::Vector2d evaluateDeBoor(double u) const {
        if (!valid_ || control_points_.rows() == 0) {
            return Eigen::Vector2d::Zero();
        }
        
        // 限制参数范围
        double u_min = u_(p_);
        double u_max = u_(m_ - p_);
        u = std::max(u_min, std::min(u_max, u));
        
        // 找到 u 所在的节点区间 [u_k, u_{k+1})
        int k = p_;
        while (k < m_ - p_ && u_(k + 1) <= u) {
            k++;
        }
        k = std::min(k, m_ - p_ - 1);  // 确保不越界
        
        // De Boor 递归
        std::vector<Eigen::Vector2d> d(p_ + 1);
        for (int i = 0; i <= p_; ++i) {
            int idx = k - p_ + i;
            if (idx >= 0 && idx < control_points_.rows()) {
                d[i] = control_points_.row(idx).head<2>();
            } else {
                d[i] = Eigen::Vector2d::Zero();
            }
        }
        
        for (int r = 1; r <= p_; ++r) {
            for (int i = p_; i >= r; --i) {
                int u_idx1 = k - p_ + i;
                int u_idx2 = k + 1 + i - r;
                if (u_idx1 >= 0 && u_idx1 <= m_ && u_idx2 >= 0 && u_idx2 <= m_) {
                    double denom = u_(u_idx2) - u_(u_idx1);
                    if (std::abs(denom) > 1e-10) {
                        double alpha = (u - u_(u_idx1)) / denom;
                        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
                    }
                }
            }
        }
        
        return d[p_];
    }
    
    // 按时间参数求值（t ∈ [0, duration]）
    Eigen::Vector2d evaluateT(double t) const {
        if (!valid_) {
            return Eigen::Vector2d::Zero();
        }
        double u = u_(p_) + t;
        return evaluateDeBoor(u);
    }
    
    // 获取曲线持续时间
    double getDuration() const {
        if (!valid_) {
            return 0.0;
        }
        return u_(m_ - p_) - u_(p_);
    }
    
    // 获取控制点
    const Eigen::MatrixXd& getControlPoints() const { return control_points_; }
    double getInterval() const { return interval_; }
    int getOrder() const { return p_; }
    
    // 采样曲线点（使用稳定的 De Boor 算法）
    std::vector<Eigen::Vector2d> sample(double dt = 0.05) const {
        std::vector<Eigen::Vector2d> points;
        
        if (!valid_) {
            return points;
        }
        
        double duration = getDuration();
        if (duration <= 0) {
            return points;
        }
        
        // 确保采样间隔合理
        dt = std::max(dt, duration / 1000.0);
        points.reserve(static_cast<size_t>(duration / dt) + 2);
        
        for (double t = 0; t <= duration; t += dt) {
            points.push_back(evaluateT(t));
        }
        
        // 确保终点被包含
        Eigen::Vector2d end_pt = evaluateT(duration);
        if (points.empty() || (end_pt - points.back()).norm() > 1e-6) {
            points.push_back(end_pt);
        }
        
        return points;
    }

private:
    Eigen::MatrixXd control_points_;  // 控制点 N×2
    Eigen::VectorXd u_;               // 节点向量
    int p_ = 3;                       // 阶数
    int n_ = 0;                       // n+1 个控制点
    int m_ = 0;                       // m+1 个节点
    double interval_ = 0.1;           // 节点间隔
    bool valid_ = false;              // 是否有效
};

}  // namespace nav_components
