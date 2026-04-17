// nav_components/include/nav_components/planner/bspline_optimizer.hpp
// B样条轨迹优化器 - 参考 Fast-Planner (HKUST)
// 简化版：平滑项 + 障碍物项，适配 2D 差速底盘

#pragma once
#include <Eigen/Dense>
#include <vector>
#include <functional>
#include <cmath>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

namespace nav_components {

// 优化参数
struct BSplineOptParams {
    // 代价函数权重
    double lambda_smooth = 10.0;    // 平滑项权重
    double lambda_collision = 5.0;  // 障碍物项权重
    
    // 障碍物参数
    double safe_dist = 0.3;         // 安全距离阈值 (m)
    
    // 曲率自适应（转角处离障碍物更远）
    bool curvature_adaptive = true; // 启用曲率自适应
    double curvature_weight = 2.0;  // 曲率加权系数 (>1 时转角处更远离障碍物)
    
    // 窄通道中心吸引（进出窄通道时走中间）
    bool narrow_passage_align = true;   // 启用窄通道中心吸引
    double narrow_passage_thresh = 0.6; // 窄通道判定阈值 (m)，小于此距离认为在窄区域
    double lambda_align = 3.0;          // 中心吸引权重
    int align_entry_points = 2;         // 入口/出口各优化几个点 (1-3)
    int narrow_min_consecutive = 3;     // 最少连续几个点才算窄通道（防止单点误判）

    // 台阶段方向约束（路径切向尽量与台阶法向一致）
    bool stair_segment_align = true;
    double lambda_stair_align = 6.0;
    // 兼容旧参数：仅在四个弧长窗口参数全部<=0时回退使用
    int stair_expand_points = 2;
    // 新参数：按弧长定义作用窗口，并区分上/下台阶
    double stair_align_up_pre_dist = 0.6;
    double stair_align_up_post_dist = 0.6;
    double stair_align_down_pre_dist = 0.6;
    double stair_align_down_post_dist = 0.6;
    // 曲线采样级优化参数（Phase B）
    std::string stair_align_mode = "curve_sample"; // "control_point"(旧) or "curve_sample"(新)
    double stair_sample_ds = 0.08; // 曲线采样间距 (m)
    // Phase C: 锚点与走廊约束
    double lambda_stair_anchor = 0.0;    // 锚点权重，0=禁用（纯 Phase B）
    double lambda_stair_lateral = 0.0;   // 走廊权重，0=禁用
    double stair_corridor_half_width = 0.30; // 走廊半宽 (m)
    // 使用 A* 命中的台阶点作为锚点中心（减少 A*/B-spline 横向偏移）
    bool use_astar_stair_anchors = true;
    double astar_stair_anchor_match_max_dist = 0.6;
    // 命中点近障碍时沿台阶切向向低代价侧偏移
    double astar_anchor_near_obstacle_dist = 0.18;
    double astar_anchor_tangent_probe_dist = 0.12;
    double astar_anchor_tangent_shift_step = 0.08;
    double astar_anchor_tangent_shift_max = 0.24;
    double astar_anchor_tangent_improve_min = 0.01;
    
    // 优化器参数
    int max_iterations = 100;       // 最大迭代次数
    double init_step_size = 0.1;    // 初始步长
    double min_step_size = 1e-4;    // 最小步长
    double cost_tolerance = 1e-4;   // 代价变化收敛阈值
    double grad_tolerance = 1e-3;   // 梯度范数收敛阈值
};

// 台阶对齐诊断信息（Phase A）
struct StairAlignDiagnostics {
    int num_clusters = 0;
    int num_curve_samples = 0;
    double mean_heading_err_deg = 0.0;
    double max_heading_err_deg = 0.0;
    struct ClusterInfo {
        Eigen::Vector2d center_pos = Eigen::Vector2d::Zero();
        Eigen::Vector2d normal = Eigen::Vector2d::Zero();
        bool is_uphill = true;
        double window_arc_start = 0.0;
        double window_arc_end = 0.0;
        Eigen::Vector2d anchor_pre = Eigen::Vector2d::Zero();
        Eigen::Vector2d anchor_mid = Eigen::Vector2d::Zero();
        Eigen::Vector2d anchor_post = Eigen::Vector2d::Zero();
        bool has_astar_match = false;
        Eigen::Vector2d astar_match_point = Eigen::Vector2d::Zero();
        double astar_match_dist = -1.0;
    };
    std::vector<ClusterInfo> clusters;
    std::vector<Eigen::Vector2d> astar_hit_points;
    struct SampleInfo {
        Eigen::Vector2d position = Eigen::Vector2d::Zero();
        Eigen::Vector2d tangent_unit = Eigen::Vector2d::Zero();
        Eigen::Vector2d normal = Eigen::Vector2d::Zero();
        double heading_err_deg = 0.0;
    };
    std::vector<SampleInfo> samples;
    double mean_lateral_disp = 0.0;
    double max_lateral_disp = 0.0;
};

// ESDF 查询回调类型
// 输入: 2D 位置 (x, y)
// 输出: 距离值 (正=自由空间, 负=障碍物内部)
// 可选输出: 梯度 (指向远离障碍物的方向)
using ESDFCallback = std::function<double(double x, double y, double* grad_x, double* grad_y)>;
using StairNormalCallback = std::function<bool(double x, double y, double* nx, double* ny)>;

class BSplineOptimizer {
public:
    BSplineOptimizer() = default;
    explicit BSplineOptimizer(const BSplineOptParams& params) : params_(params) {}
    
    void setParams(const BSplineOptParams& params) { params_ = params; }
    void setESDFCallback(ESDFCallback cb) { esdf_callback_ = cb; }
    void setStairNormalCallback(StairNormalCallback cb) { stair_normal_callback_ = cb; }
    void setAstarStairHits(const std::vector<Eigen::Vector2d>& hits) {
        astar_stair_hits_ = hits;
    }
    void setInterval(double interval) { bspline_interval_ = interval; }
    const StairAlignDiagnostics& getStairDiagnostics() const { return last_stair_diag_; }
    
    /**
     * 优化 B样条控制点
     * @param ctrl_pts 输入/输出：控制点矩阵 (N x 2)，每行一个控制点 (x, y)
     * @param order B样条阶数（通常为 3）
     * @return 是否成功优化
     * 
     * 注意：首尾各 order 个控制点不参与优化（边界约束）
     */
    bool optimize(Eigen::MatrixXd& ctrl_pts, int order = 3) {
        auto t_start = std::chrono::high_resolution_clock::now();
        
        if (ctrl_pts.rows() < 2 * order + 1) {
            RCLCPP_WARN(rclcpp::get_logger("bspline_opt"), 
                "控制点太少 (%ld)，无法优化", ctrl_pts.rows());
            return false;
        }
        
        order_ = order;
        n_ctrl_ = static_cast<int>(ctrl_pts.rows());
        
        // 优化范围：排除首尾各 order 个点
        opt_start_ = order;
        opt_end_ = n_ctrl_ - order;  // 不包含
        n_opt_ = opt_end_ - opt_start_;
        
        if (n_opt_ <= 0) {
            return true;  // 没有可优化的点
        }
        
        // 提取可优化的控制点作为优化变量
        Eigen::MatrixXd x = ctrl_pts.block(opt_start_, 0, n_opt_, 2);
        
        // 梯度下降优化
        double prev_cost = std::numeric_limits<double>::max();
        double initial_cost = 0.0;
        double step = params_.init_step_size;
        int final_iter = 0;
        
        for (int iter = 0; iter < params_.max_iterations; ++iter) {
            final_iter = iter;
            
            // 将当前优化变量写回完整控制点
            ctrl_pts.block(opt_start_, 0, n_opt_, 2) = x;
            
            // 计算代价和梯度
            Eigen::MatrixXd grad = Eigen::MatrixXd::Zero(n_opt_, 2);
            double cost = computeCostAndGradient(ctrl_pts, grad);
            
            // 记录初始代价
            if (iter == 0) {
                initial_cost = cost;
            }
            
            // 检查收敛
            double grad_norm = grad.norm();
            double cost_change = std::abs(prev_cost - cost);
            
            if (iter % 20 == 0) {
                RCLCPP_DEBUG(rclcpp::get_logger("bspline_opt"),
                    "Iter %d: cost=%.4f, grad_norm=%.4f, step=%.4f",
                    iter, cost, grad_norm, step);
            }
            
            if (cost_change < params_.cost_tolerance && iter > 5) {
                RCLCPP_DEBUG(rclcpp::get_logger("bspline_opt"),
                    "收敛 (代价变化 %.6f < %.6f)", cost_change, params_.cost_tolerance);
                break;
            }
            
            if (grad_norm < params_.grad_tolerance) {
                RCLCPP_DEBUG(rclcpp::get_logger("bspline_opt"),
                    "收敛 (梯度范数 %.6f < %.6f)", grad_norm, params_.grad_tolerance);
                break;
            }
            
            // 自适应步长（简单的 Armijo 线搜索）
            double new_cost = cost;
            Eigen::MatrixXd x_new = x;
            bool step_accepted = false;
            
            for (int ls = 0; ls < 10; ++ls) {
                x_new = x - step * grad;
                
                // 临时写回并计算新代价
                ctrl_pts.block(opt_start_, 0, n_opt_, 2) = x_new;
                Eigen::MatrixXd tmp_grad = Eigen::MatrixXd::Zero(n_opt_, 2);
                new_cost = computeCostAndGradient(ctrl_pts, tmp_grad);
                
                if (new_cost < cost) {
                    // 接受这一步，尝试增大步长
                    step *= 1.2;
                    step = std::min(step, 1.0);
                    step_accepted = true;
                    break;
                } else {
                    // 减小步长重试
                    step *= 0.5;
                    if (step < params_.min_step_size) {
                        step = params_.min_step_size;
                        break;
                    }
                }
            }
            
            // 只有在代价降低时才接受新位置
            if (step_accepted) {
                x = x_new;
                prev_cost = new_cost;
            } else {
                // 无法降低代价，恢复控制点并结束优化
                ctrl_pts.block(opt_start_, 0, n_opt_, 2) = x;
                RCLCPP_DEBUG(rclcpp::get_logger("bspline_opt"),
                    "无法继续降低代价，提前结束优化");
                break;
            }
        }
        
        // 最终结果写回
        ctrl_pts.block(opt_start_, 0, n_opt_, 2) = x;
        
        // 最终台阶对齐诊断（Phase A: 输出角度误差统计）
        if (params_.stair_segment_align && stair_normal_callback_) {
            Eigen::MatrixXd diag_grad = Eigen::MatrixXd::Zero(n_opt_, 2);
            double diag_cost = 0.0;
            computeStairAlignCostAndGrad(ctrl_pts, diag_grad, diag_cost);
            if (last_stair_diag_.num_clusters > 0) {
                RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                    "[StairAlign] mode=%s | clusters=%d, samples=%d | "
                    "heading: mean=%.1f° max=%.1f° | lateral: mean=%.3fm max=%.3fm | cost=%.4f",
                    params_.stair_align_mode.c_str(),
                    last_stair_diag_.num_clusters, last_stair_diag_.num_curve_samples,
                    last_stair_diag_.mean_heading_err_deg,
                    last_stair_diag_.max_heading_err_deg,
                    last_stair_diag_.mean_lateral_disp,
                    last_stair_diag_.max_lateral_disp,
                    diag_cost);
                for (size_t ci = 0; ci < last_stair_diag_.clusters.size(); ++ci) {
                    const auto& cl = last_stair_diag_.clusters[ci];
                    RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                        "  cluster[%zu]: %s, n=(%.2f,%.2f), center=(%.2f,%.2f), "
                        "window=[%.2f, %.2f]m, anchor_pre=(%.2f,%.2f), anchor_post=(%.2f,%.2f), "
                        "astar_match=%s, d_pre=%.3fm",
                        ci, cl.is_uphill ? "UP" : "DOWN",
                        cl.normal.x(), cl.normal.y(),
                        cl.center_pos.x(), cl.center_pos.y(),
                        cl.window_arc_start, cl.window_arc_end,
                        cl.anchor_pre.x(), cl.anchor_pre.y(),
                        cl.anchor_post.x(), cl.anchor_post.y(),
                        cl.has_astar_match ? "yes" : "no",
                        cl.astar_match_dist);
                }
            }
        }
        
        // 计算并输出优化时间统计
        auto t_end = std::chrono::high_resolution_clock::now();
        last_opt_time_ms_ = std::chrono::duration_cast<std::chrono::microseconds>(
            t_end - t_start).count() / 1000.0;
        
        double cost_reduction = (initial_cost > 0) ? 
            (initial_cost - last_cost_) / initial_cost * 100.0 : 0.0;
        
        // 根据是否启用附加约束选择输出格式
        if ((params_.narrow_passage_align && last_align_cost_ > 0.01) ||
            (params_.stair_segment_align && last_stair_align_cost_ > 0.01)) {
            RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                "优化完成: %d次迭代, %.2fms | 代价: %.4f -> %.4f (降低%.1f%%) | "
                "平滑=%.3f, 碰撞=%.3f, 窄通道=%.3f, 台阶=%.3f",
                final_iter + 1, last_opt_time_ms_, initial_cost, last_cost_, cost_reduction,
                last_smooth_cost_, last_collision_cost_, last_align_cost_, last_stair_align_cost_);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                "优化完成: %d次迭代, %.2fms | 代价: %.4f -> %.4f (降低%.1f%%) | "
                "平滑=%.3f, 碰撞=%.3f",
                final_iter + 1, last_opt_time_ms_, initial_cost, last_cost_, cost_reduction,
                last_smooth_cost_, last_collision_cost_);
        }
        
        return true;
    }
    
    // 获取上次优化的统计信息
    double getLastCost() const { return last_cost_; }
    double getLastSmoothCost() const { return last_smooth_cost_; }
    double getLastCollisionCost() const { return last_collision_cost_; }
    double getLastAlignCost() const { return last_align_cost_; }
    double getLastStairAlignCost() const { return last_stair_align_cost_; }
    double getLastOptTimeMs() const { return last_opt_time_ms_; }

private:
    /**
     * 计算总代价和梯度
     * grad 只包含可优化控制点的梯度 (n_opt_ x 2)
     */
    double computeCostAndGradient(const Eigen::MatrixXd& ctrl_pts, 
                                   Eigen::MatrixXd& grad) {
        double cost_smooth = 0.0;
        double cost_collision = 0.0;
        double cost_align = 0.0;
        double cost_stair_align = 0.0;
        
        grad.setZero();
        
        // ===== 1. 平滑项 (Elastic Band) =====
        // f_s = Σ || Q_{i+1} + Q_{i-1} - 2*Q_i ||²
        computeSmoothCostAndGrad(ctrl_pts, grad, cost_smooth);
        
        // ===== 2. 障碍物项 (ESDF-based，带曲率自适应) =====
        computeCollisionCostAndGrad(ctrl_pts, grad, cost_collision);
        
        // ===== 3. 窄通道航向对齐项 =====
        if (params_.narrow_passage_align && esdf_callback_) {
            computeAlignCostAndGrad(ctrl_pts, grad, cost_align);
        }

        // ===== 4. 台阶段方向约束项 =====
        if (params_.stair_segment_align && stair_normal_callback_) {
            computeStairAlignCostAndGrad(ctrl_pts, grad, cost_stair_align);
        }
        
        // 总代价
        double total_cost = params_.lambda_smooth * cost_smooth 
                         + params_.lambda_collision * cost_collision
                         + params_.lambda_align * cost_align
                         + cost_stair_align;
        
        last_cost_ = total_cost;
        last_smooth_cost_ = cost_smooth;
        last_collision_cost_ = cost_collision;
        last_align_cost_ = cost_align;
        last_stair_align_cost_ = cost_stair_align;
        
        return total_cost;
    }
    
    /**
     * 平滑项代价和梯度
     */
    void computeSmoothCostAndGrad(const Eigen::MatrixXd& ctrl_pts,
                                   Eigen::MatrixXd& grad,
                                   double& cost) {
        cost = 0.0;
        
        for (int i = 1; i < n_ctrl_ - 1; ++i) {
            Eigen::Vector2d q_prev = ctrl_pts.row(i - 1).head<2>();
            Eigen::Vector2d q_curr = ctrl_pts.row(i).head<2>();
            Eigen::Vector2d q_next = ctrl_pts.row(i + 1).head<2>();
            
            // 弹性带力
            Eigen::Vector2d force = q_next + q_prev - 2.0 * q_curr;
            cost += force.squaredNorm();
            
            // 梯度分配到相关控制点
            auto addGrad = [&](int idx, const Eigen::Vector2d& g) {
                if (idx >= opt_start_ && idx < opt_end_) {
                    grad.row(idx - opt_start_) += params_.lambda_smooth * g.transpose();
                }
            };
            
            addGrad(i, -4.0 * force);
            addGrad(i - 1, 2.0 * force);
            addGrad(i + 1, 2.0 * force);
        }
    }
    
    /**
     * 障碍物项代价和梯度（带曲率自适应）
     */
    void computeCollisionCostAndGrad(const Eigen::MatrixXd& ctrl_pts,
                                      Eigen::MatrixXd& grad,
                                      double& cost) {
        cost = 0.0;
        if (!esdf_callback_) return;
        
        for (int i = opt_start_; i < opt_end_; ++i) {
            double x = ctrl_pts(i, 0);
            double y = ctrl_pts(i, 1);
            double gx = 0, gy = 0;
            double dist = esdf_callback_(x, y, &gx, &gy);
            
            if (dist >= params_.safe_dist) continue;
            
            double diff = dist - params_.safe_dist;
            cost += diff * diff;
            
            Eigen::Vector2d grad_esdf(gx, gy);
            if (grad_esdf.norm() < 1e-6) continue;
            
            // 曲率自适应权重
            double weight = 1.0;
            if (params_.curvature_adaptive && i > 0 && i < n_ctrl_ - 1) {
                Eigen::Vector2d q_prev = ctrl_pts.row(i - 1).head<2>();
                Eigen::Vector2d q_curr = ctrl_pts.row(i).head<2>();
                Eigen::Vector2d q_next = ctrl_pts.row(i + 1).head<2>();
                double curvature = (q_next + q_prev - 2.0 * q_curr).norm();
                weight = 1.0 + params_.curvature_weight * curvature;
            }
            
            grad.row(i - opt_start_) += weight * params_.lambda_collision * 2.0 * diff * grad_esdf.transpose();
        }
    }
    
    /**
     * 窄通道中心线对齐项 (Narrow Passage Alignment)
     * 
     * 【目的】
     * 在穿越窄通道时，将入口/出口处的控制点拉向中心线延长线，
     * 使机器人能够笔直地进入和离开通道，避免在通道口转弯导致碰撞。
     * 
     * 【算法流程】
     * 1. 宽度检测：对每个控制点，沿垂直于行进方向向两侧射线搜索，测量到障碍物的距离
     * 2. 窄通道判定：两侧都有障碍物 且 总宽度 <= 阈值 且 连续点数 >= min_consecutive
     * 3. 中心线计算：使用所有窄通道点的质心和修正后的端点，构建稳定的中心线
     * 4. 理想位置：入口/出口点的理想位置在中心线延长线上
     * 5. 梯度下降：将点拉向理想位置，cost = ||pt - ideal_pos||²
     * 
     * 【关键参数】
     * - narrow_passage_thresh: 窄通道宽度阈值 (默认0.7m)
     * - narrow_min_consecutive: 连续窄点数要求 (默认3)
     * - align_entry_points: 入口/出口各处理多少个点 (默认3)
     * - lambda_align: 对齐力权重 (默认10.0)
     */
    void computeAlignCostAndGrad(const Eigen::MatrixXd& ctrl_pts,
                                  Eigen::MatrixXd& grad,
                                  double& cost) {
        cost = 0.0;
        
        const int entry_points = params_.align_entry_points;
        const int min_consecutive = params_.narrow_min_consecutive;
        const double narrow_thresh = params_.narrow_passage_thresh;
        const double search_max_dist = 1.5;  // 最大搜索距离 (m)
        
        // ==================== 第一遍：收集每个点的通道宽度信息 ====================
        struct PointInfo {
            double dist_left = 1e9;         // 左侧到障碍物的距离
            double dist_right = 1e9;        // 右侧到障碍物的距离
            double passage_width = 1e9;     // 通道宽度 = dist_left + dist_right
            double center_offset = 0.0;     // 偏离中心量 = (dist_right - dist_left) / 2，正=偏右
            bool is_narrow = false;         // 是否在窄通道内
            Eigen::Vector2d perp_dir = Eigen::Vector2d::Zero();  // 垂直方向（指向左侧）
        };
        std::vector<PointInfo> point_info(n_ctrl_);
        
        for (int i = 1; i < n_ctrl_ - 1; ++i) {
            // 计算局部行进方向（前后控制点连线）
            Eigen::Vector2d p_prev = ctrl_pts.row(i - 1).head<2>();
            Eigen::Vector2d p_curr = ctrl_pts.row(i).head<2>();
            Eigen::Vector2d p_next = ctrl_pts.row(i + 1).head<2>();
            
            Eigen::Vector2d dir = (p_next - p_prev);
            double dir_norm = dir.norm();
            if (dir_norm < 1e-6) continue;
            dir /= dir_norm;
            
            // 垂直方向（左手法则：逆时针旋转90度，指向左侧）
            Eigen::Vector2d perp(-dir.y(), dir.x());
            point_info[i].perp_dir = perp;
            
            double x = p_curr.x();
            double y = p_curr.y();
            
            // 向左侧射线搜索（沿 +perp 方向）
            double dist_left = search_max_dist;
            for (double d = 0.05; d <= search_max_dist; d += 0.05) {
                double check_x = x + perp.x() * d;
                double check_y = y + perp.y() * d;
                double esdf_dist = esdf_callback_(check_x, check_y, nullptr, nullptr);
                if (esdf_dist < 0.05) {  // 碰到障碍物
                    dist_left = d;
                    break;
                }
            }
            
            // 向右侧射线搜索（沿 -perp 方向）
            double dist_right = search_max_dist;
            for (double d = 0.05; d <= search_max_dist; d += 0.05) {
                double check_x = x - perp.x() * d;
                double check_y = y - perp.y() * d;
                double esdf_dist = esdf_callback_(check_x, check_y, nullptr, nullptr);
                if (esdf_dist < 0.05) {
                    dist_right = d;
                    break;
                }
            }
            
            point_info[i].dist_left = dist_left;
            point_info[i].dist_right = dist_right;
            point_info[i].passage_width = dist_left + dist_right;
            point_info[i].center_offset = (dist_right - dist_left) / 2.0;
            
            // 窄通道判定：两侧都有墙 且 宽度 <= 阈值
            bool has_both_walls = (dist_left < search_max_dist - 0.1) && 
                                   (dist_right < search_max_dist - 0.1);
            bool is_narrow_width = point_info[i].passage_width <= narrow_thresh;
            point_info[i].is_narrow = has_both_walls && is_narrow_width;
        }
        
        // ==================== 第二遍：检测连续窄通道段 ====================
        std::vector<std::pair<int, int>> narrow_segments;
        int seg_start = -1;
        int consecutive = 0;
        for (int i = 0; i < n_ctrl_; ++i) {
            if (point_info[i].is_narrow) {
                if (seg_start < 0) seg_start = i;
                consecutive++;
            } else {
                if (consecutive >= min_consecutive) {
                    narrow_segments.push_back({seg_start, i - 1});
                }
                seg_start = -1;
                consecutive = 0;
            }
        }
        if (consecutive >= min_consecutive) {
            narrow_segments.push_back({seg_start, n_ctrl_ - 1});
        }
        
        // 调试输出（每10次调用打印1次）
        static int call_count = 0;
        bool do_log = (++call_count % 10 == 1);
        if (do_log) {
            int narrow_count = 0;
            for (int i = 0; i < n_ctrl_; ++i) if (point_info[i].is_narrow) narrow_count++;
            RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                "[Align] thresh=%.2f, min_consec=%d, entry_pts=%d, segments=%zu, narrow_pts=%d/%d",
                narrow_thresh, min_consecutive, entry_points, narrow_segments.size(), narrow_count, n_ctrl_);
            
            for (int i = 1; i < n_ctrl_ - 1; ++i) {
                if (point_info[i].passage_width < 1.0) {
                    RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                        "  pt[%d]: L=%.2f R=%.2f W=%.2f %s",
                        i, point_info[i].dist_left, point_info[i].dist_right, 
                        point_info[i].passage_width,
                        point_info[i].is_narrow ? "NARROW" : "");
                }
            }
        }
        
        if (narrow_segments.empty()) return;
        
        // ==================== 第三遍：对每个窄通道段施加对齐力 ====================
        for (const auto& seg : narrow_segments) {
            int seg_start_idx = seg.first;
            int seg_end_idx = seg.second;
            int seg_length = seg_end_idx - seg_start_idx + 1;
            
            // 计算通道最窄处的宽度
            double min_width = 1e9;
            for (int i = seg_start_idx; i <= seg_end_idx; ++i) {
                if (point_info[i].passage_width < min_width) {
                    min_width = point_info[i].passage_width;
                }
            }
            
            // ===== 计算稳定的中心线（基于所有窄通道点的质心） =====
            
            // 1. 通道原始方向（用首尾点估算）
            Eigen::Vector2d entry_pos(ctrl_pts(seg_start_idx, 0), ctrl_pts(seg_start_idx, 1));
            Eigen::Vector2d exit_pos(ctrl_pts(seg_end_idx, 0), ctrl_pts(seg_end_idx, 1));
            Eigen::Vector2d raw_dir = exit_pos - entry_pos;
            double passage_len = raw_dir.norm();
            if (passage_len < 1e-6) continue;
            raw_dir /= passage_len;
            
            // 2. 垂直方向（用于将控制点修正到真正的通道中心）
            Eigen::Vector2d perp_raw(-raw_dir.y(), raw_dir.x());
            
            // 3. 修正后的中心线端点
            //    center_offset > 0 表示点偏左（右侧距离大），需要向右移 (-perp)
            Eigen::Vector2d center_start = entry_pos - perp_raw * point_info[seg_start_idx].center_offset;
            Eigen::Vector2d center_end = exit_pos - perp_raw * point_info[seg_end_idx].center_offset;
            
            // 4. 修正后的通道方向
            Eigen::Vector2d passage_dir = center_end - center_start;
            double center_len = passage_dir.norm();
            if (center_len < 1e-6) continue;
            passage_dir /= center_len;
            
            // 5. 计算修正后的中心线质心（所有点修正到中心后取平均）
            Eigen::Vector2d center_centroid(0, 0);
            for (int i = seg_start_idx; i <= seg_end_idx; ++i) {
                Eigen::Vector2d pt(ctrl_pts(i, 0), ctrl_pts(i, 1));
                center_centroid += pt - perp_raw * point_info[i].center_offset;
            }
            center_centroid /= seg_length;
            
            // ===== 过滤误判：长宽比检查 =====
            // 真正的窄通道应该是 "长而窄"，长度至少是宽度的1.2倍
            double length_width_ratio = passage_len / (min_width + 0.01);
            if (length_width_ratio < 1.2) {
                if (do_log) {
                    RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                        "[Align] 跳过可疑段 seg[%d-%d]: len=%.2f, width=%.2f, ratio=%.1f (可能是弯道)",
                        seg_start_idx, seg_end_idx, passage_len, min_width, length_width_ratio);
                }
                continue;
            }
            
            // 垂直于通道方向（用于通道内点的中心对齐）
            Eigen::Vector2d perp_dir(-passage_dir.y(), passage_dir.x());
            
            if (do_log) {
                RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                    "[Align] 通道 seg[%d-%d], len=%.2f, width=%.2f, dir=(%.2f,%.2f)",
                    seg_start_idx, seg_end_idx, passage_len, min_width, passage_dir.x(), passage_dir.y());
                RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                    "  中心线: (%.2f,%.2f) -> (%.2f,%.2f), 质心=(%.2f,%.2f)",
                    center_start.x(), center_start.y(), center_end.x(), center_end.y(),
                    center_centroid.x(), center_centroid.y());
            }
            
            // ===== 处理入口点：拉向中心线反向延长线 =====
            double avg_spacing = passage_len / (seg_length > 1 ? seg_length - 1 : 1);
            double centroid_to_start = (center_start - center_centroid).dot(passage_dir);
            
            for (int k = 1; k <= entry_points; ++k) {
                int idx = seg_start_idx - k;
                if (idx < opt_start_ || idx >= opt_end_) {
                    if (do_log) RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                        "  入口 pt[%d]: 跳过(超出优化范围)", idx);
                    continue;
                }
                
                Eigen::Vector2d pt(ctrl_pts(idx, 0), ctrl_pts(idx, 1));
                
                // 理想位置 = 质心 - passage_dir * (质心到入口距离 + k*间距)
                double target_dist = -centroid_to_start + k * avg_spacing;
                Eigen::Vector2d ideal_pos = center_centroid - passage_dir * target_dist;
                Eigen::Vector2d offset_vec = pt - ideal_pos;
                
                // 安全检查：理想位置不能在障碍物内
                double dist_ideal = esdf_callback_(ideal_pos.x(), ideal_pos.y(), nullptr, nullptr);
                double min_safe = std::min(0.1, min_width * 0.25);
                if (dist_ideal < min_safe) {
                    if (do_log) RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                        "  入口 pt[%d]: 跳过(理想位置距障碍物 %.2f < %.2f)", idx, dist_ideal, min_safe);
                    continue;
                }
                
                // 施加对齐力：cost = ||pt - ideal||², grad = 2 * (pt - ideal)
                double offset_norm = offset_vec.norm();
                if (offset_norm > 0.03) {
                    cost += offset_norm * offset_norm;
                    Eigen::Vector2d grad_term = params_.lambda_align * 2.0 * offset_vec;
                    grad.row(idx - opt_start_) += grad_term.transpose();
                    if (do_log) RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                        "  入口 pt[%d]: offset=%.3f -> 拉向(%.2f, %.2f)", 
                        idx, offset_norm, ideal_pos.x(), ideal_pos.y());
                }
            }
            
            // ===== 处理出口点：拉向中心线正向延长线 =====
            double centroid_to_end = (center_end - center_centroid).dot(passage_dir);
            
            for (int k = 1; k <= entry_points; ++k) {
                int idx = seg_end_idx + k;
                if (idx < opt_start_ || idx >= opt_end_) {
                    if (do_log) RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                        "  出口 pt[%d]: 跳过(超出优化范围)", idx);
                    continue;
                }
                
                Eigen::Vector2d pt(ctrl_pts(idx, 0), ctrl_pts(idx, 1));
                
                // 理想位置 = 质心 + passage_dir * (质心到出口距离 + k*间距)
                double target_dist = centroid_to_end + k * avg_spacing;
                Eigen::Vector2d ideal_pos = center_centroid + passage_dir * target_dist;
                Eigen::Vector2d offset_vec = pt - ideal_pos;
                
                // 安全检查
                double dist_ideal = esdf_callback_(ideal_pos.x(), ideal_pos.y(), nullptr, nullptr);
                double min_safe = std::min(0.1, min_width * 0.25);
                if (dist_ideal < min_safe) {
                    if (do_log) RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                        "  出口 pt[%d]: 跳过(理想位置距障碍物 %.2f < %.2f)", idx, dist_ideal, min_safe);
                    continue;
                }
                
                // 施加对齐力
                double offset_norm = offset_vec.norm();
                if (offset_norm > 0.03) {
                    cost += offset_norm * offset_norm;
                    Eigen::Vector2d grad_term = params_.lambda_align * 2.0 * offset_vec;
                    grad.row(idx - opt_start_) += grad_term.transpose();
                    if (do_log) RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                        "  出口 pt[%d]: 当前(%.2f,%.2f) -> 理想(%.2f,%.2f), offset=%.3f", 
                        idx, pt.x(), pt.y(), ideal_pos.x(), ideal_pos.y(), offset_norm);
                }
            }
            
            // ===== 处理通道内的点：推向通道中心 =====
            for (int idx = seg_start_idx; idx <= seg_end_idx; ++idx) {
                if (idx < opt_start_ || idx >= opt_end_) continue;
                
                double offset = point_info[idx].center_offset;
                Eigen::Vector2d perp = point_info[idx].perp_dir;
                if (perp.norm() < 1e-6) continue;
                
                // 只有偏离中心足够大时才施加力
                if (std::abs(offset) > 0.02) {
                    cost += offset * offset;
                    // offset > 0 表示偏左（右侧空间大），需要向右推（-perp方向）
                    // 梯度 = offset * perp 使得点向中心移动
                    Eigen::Vector2d grad_term = params_.lambda_align * 2.0 * offset * perp;
                    grad.row(idx - opt_start_) += grad_term.transpose();
                }
            }
        }
    }

    void computeStairAlignLegacy(const Eigen::MatrixXd& ctrl_pts,
                                 Eigen::MatrixXd& grad,
                                 double& cost) {
        cost = 0.0;
        last_stair_diag_ = StairAlignDiagnostics{};
        if (!stair_normal_callback_) {
            return;
        }

        std::vector<int> stair_indices;
        std::vector<Eigen::Vector2d> stair_normals;
        std::vector<uint8_t> stair_is_uphill;
        stair_indices.reserve(n_ctrl_);
        stair_normals.reserve(n_ctrl_);
        stair_is_uphill.reserve(n_ctrl_);

        for (int i = 1; i < n_ctrl_ - 1; ++i) {
            double nx = 0.0;
            double ny = 0.0;
            if (!stair_normal_callback_(ctrl_pts(i, 0), ctrl_pts(i, 1), &nx, &ny)) {
                continue;
            }

            Eigen::Vector2d normal(nx, ny);
            double normal_norm = normal.norm();
            if (normal_norm < 1e-6) {
                continue;
            }
            normal /= normal_norm;
            stair_indices.push_back(i);
            stair_normals.push_back(normal);

            Eigen::Vector2d p_prev = ctrl_pts.row(i - 1).head<2>();
            Eigen::Vector2d p_next = ctrl_pts.row(i + 1).head<2>();
            Eigen::Vector2d tangent = p_next - p_prev;
            bool is_uphill = true;
            if (tangent.norm() > 1e-6) {
                is_uphill = (tangent.dot(normal) >= 0.0);
            }
            stair_is_uphill.push_back(is_uphill ? 1u : 0u);
        }

        if (stair_indices.empty()) {
            return;
        }

        std::vector<double> arc_length(n_ctrl_, 0.0);
        for (int i = 1; i < n_ctrl_; ++i) {
            Eigen::Vector2d p_prev = ctrl_pts.row(i - 1).head<2>();
            Eigen::Vector2d p_curr = ctrl_pts.row(i).head<2>();
            arc_length[i] = arc_length[i - 1] + (p_curr - p_prev).norm();
        }

        const double up_pre_dist = std::max(0.0, params_.stair_align_up_pre_dist);
        const double up_post_dist = std::max(0.0, params_.stair_align_up_post_dist);
        const double down_pre_dist = std::max(0.0, params_.stair_align_down_pre_dist);
        const double down_post_dist = std::max(0.0, params_.stair_align_down_post_dist);
        const bool use_arc_window =
            (up_pre_dist > 1e-6 || up_post_dist > 1e-6 ||
             down_pre_dist > 1e-6 || down_post_dist > 1e-6);
        const int expand_points = std::max(0, params_.stair_expand_points);
        std::vector<int> nearest_stair_index(n_ctrl_, -1);
        std::vector<double> nearest_stair_arc_dist(
            n_ctrl_, std::numeric_limits<double>::max());
        std::vector<Eigen::Vector2d> nearest_stair_normal(n_ctrl_, Eigen::Vector2d::Zero());

        for (size_t k = 0; k < stair_indices.size(); ++k) {
            int stair_idx = stair_indices[k];
            const bool is_uphill = (stair_is_uphill[k] != 0u);
            const double pre_dist = is_uphill ? up_pre_dist : down_pre_dist;
            const double post_dist = is_uphill ? up_post_dist : down_post_dist;

            int window_start = stair_idx;
            int window_end = stair_idx;
            if (use_arc_window) {
                double acc_pre = 0.0;
                for (int idx = stair_idx; idx > 1; --idx) {
                    Eigen::Vector2d p_curr = ctrl_pts.row(idx).head<2>();
                    Eigen::Vector2d p_prev = ctrl_pts.row(idx - 1).head<2>();
                    acc_pre += (p_curr - p_prev).norm();
                    if (acc_pre > pre_dist + 1e-6) {
                        break;
                    }
                    window_start = idx - 1;
                }

                double acc_post = 0.0;
                for (int idx = stair_idx; idx < n_ctrl_ - 2; ++idx) {
                    Eigen::Vector2d p_curr = ctrl_pts.row(idx).head<2>();
                    Eigen::Vector2d p_next = ctrl_pts.row(idx + 1).head<2>();
                    acc_post += (p_next - p_curr).norm();
                    if (acc_post > post_dist + 1e-6) {
                        break;
                    }
                    window_end = idx + 1;
                }
            } else {
                window_start = std::max(1, stair_idx - expand_points);
                window_end = std::min(n_ctrl_ - 2, stair_idx + expand_points);
            }

            for (int idx = window_start; idx <= window_end; ++idx) {
                const double arc_dist =
                    std::abs(arc_length[idx] - arc_length[stair_idx]);
                if (nearest_stair_index[idx] < 0 || arc_dist < nearest_stair_arc_dist[idx]) {
                    nearest_stair_index[idx] = stair_idx;
                    nearest_stair_arc_dist[idx] = arc_dist;
                    nearest_stair_normal[idx] = stair_normals[k];
                }
            }
        }

        for (int i = 1; i < n_ctrl_ - 1; ++i) {
            if (nearest_stair_index[i] < 0) {
                continue;
            }

            const Eigen::Vector2d& normal = nearest_stair_normal[i];

            Eigen::Vector2d p_prev = ctrl_pts.row(i - 1).head<2>();
            Eigen::Vector2d p_next = ctrl_pts.row(i + 1).head<2>();
            Eigen::Vector2d tangent = p_next - p_prev;
            if (tangent.norm() < 1e-6) {
                continue;
            }

            Eigen::Vector2d tangent_perp = tangent - tangent.dot(normal) * normal;
            cost += params_.lambda_stair_align * tangent_perp.squaredNorm();

            Eigen::Vector2d grad_tangent = 2.0 * tangent_perp;
            if (i + 1 >= opt_start_ && i + 1 < opt_end_) {
                grad.row(i + 1 - opt_start_) +=
                    (params_.lambda_stair_align * grad_tangent).transpose();
            }
            if (i - 1 >= opt_start_ && i - 1 < opt_end_) {
                grad.row(i - 1 - opt_start_) -=
                    (params_.lambda_stair_align * grad_tangent).transpose();
            }
        }
    }

    // ===== 台阶方向约束：调度函数 =====
    void computeStairAlignCostAndGrad(const Eigen::MatrixXd& ctrl_pts,
                                      Eigen::MatrixXd& grad,
                                      double& cost) {
        if (params_.stair_align_mode == "control_point" || bspline_interval_ < 1e-9) {
            computeStairAlignLegacy(ctrl_pts, grad, cost);
        } else {
            computeStairAlignCurveSample(ctrl_pts, grad, cost);
        }
    }

    // ===== 新版：曲线采样级台阶对齐（Phase B）=====
    void computeStairAlignCurveSample(const Eigen::MatrixXd& ctrl_pts,
                                      Eigen::MatrixXd& grad,
                                      double& cost) {
        cost = 0.0;
        last_stair_diag_ = StairAlignDiagnostics{};
        if (!stair_normal_callback_ || bspline_interval_ < 1e-9) return;

        const int p = order_;
        const double h = bspline_interval_;
        const double duration = static_cast<double>(n_ctrl_ - p) * h;
        if (duration < 1e-6 || p != 3) return;

        const double sample_ds = std::max(0.01, params_.stair_sample_ds);
        const double dt = std::min(h * 0.5, sample_ds);
        const int n_samples = std::max(2,
            static_cast<int>(std::ceil(duration / dt)) + 1);

        // Step 1: 密集采样曲线并检测台阶区域
        struct CurveSample {
            double t;
            Eigen::Vector2d pos;
            Eigen::Vector2d tangent;
            double arc_len;
            int span_k;
            double local_s;
            bool stair_hit = false;
            Eigen::Vector2d normal = Eigen::Vector2d::Zero();
            bool is_uphill = true;
        };

        std::vector<CurveSample> samples;
        samples.reserve(n_samples);
        double arc = 0.0;

        for (int i = 0; i < n_samples; ++i) {
            CurveSample cs;
            cs.t = std::min(static_cast<double>(i) * dt, duration);
            evalCubicBSpline(ctrl_pts, h, cs.t,
                             cs.pos, cs.tangent, cs.span_k, cs.local_s);

            if (!samples.empty()) {
                arc += (cs.pos - samples.back().pos).norm();
            }
            cs.arc_len = arc;

            double nx_val = 0.0, ny_val = 0.0;
            if (stair_normal_callback_(cs.pos.x(), cs.pos.y(),
                                       &nx_val, &ny_val)) {
                Eigen::Vector2d nv(nx_val, ny_val);
                double nn = nv.norm();
                if (nn > 1e-6) {
                    nv /= nn;
                    cs.stair_hit = true;
                    cs.normal = nv;
                    if (cs.tangent.norm() > 1e-6) {
                        cs.is_uphill = (cs.tangent.dot(nv) >= 0.0);
                    }
                }
            }
            samples.push_back(cs);
        }

        if (samples.empty()) return;

        // Step 2: 聚类连续台阶命中点
        struct StairCluster {
            int start_idx = 0, end_idx = 0;
            Eigen::Vector2d normal = Eigen::Vector2d::Zero();
            bool is_uphill = true;
            double center_arc = 0.0;
            Eigen::Vector2d center_pos = Eigen::Vector2d::Zero();
        };

        std::vector<StairCluster> clusters;
        {
            int cl_s = -1;
            Eigen::Vector2d cl_n = Eigen::Vector2d::Zero();
            Eigen::Vector2d cl_p = Eigen::Vector2d::Zero();
            int cl_up = 0, cl_cnt = 0;

            auto flush = [&](int end_i) {
                if (cl_s >= 0 && cl_cnt >= 1) {
                    StairCluster cl;
                    cl.start_idx = cl_s;
                    cl.end_idx = end_i;
                    Eigen::Vector2d avg = cl_n / cl_cnt;
                    cl.normal = (avg.norm() > 1e-6)
                        ? avg.normalized()
                        : Eigen::Vector2d::UnitX();
                    cl.is_uphill = (cl_up * 2 >= cl_cnt);
                    cl.center_arc = (samples[cl_s].arc_len
                                     + samples[end_i].arc_len) * 0.5;
                    cl.center_pos = cl_p / cl_cnt;
                    clusters.push_back(cl);
                }
                cl_s = -1;
                cl_n.setZero();
                cl_p.setZero();
                cl_up = 0;
                cl_cnt = 0;
            };

            for (int i = 0; i < static_cast<int>(samples.size()); ++i) {
                if (samples[i].stair_hit) {
                    if (cl_s < 0) cl_s = i;
                    cl_n += samples[i].normal;
                    cl_p += samples[i].pos;
                    if (samples[i].is_uphill) cl_up++;
                    cl_cnt++;
                } else {
                    flush(i - 1);
                }
            }
            flush(static_cast<int>(samples.size()) - 1);
        }

        if (clusters.empty()) return;

        // Step 3: 定义作用窗、计算锚点、将采样点分配到最近的台阶簇
        const double up_pre = std::max(0.0, params_.stair_align_up_pre_dist);
        const double up_post = std::max(0.0, params_.stair_align_up_post_dist);
        const double down_pre = std::max(0.0, params_.stair_align_down_pre_dist);
        const double down_post = std::max(0.0, params_.stair_align_down_post_dist);

        struct ClusterAnchorInfo {
            Eigen::Vector2d anchor_pre, anchor_mid, anchor_post;
            Eigen::Vector2d t_stair;
            Eigen::Vector2d center_pos;
            double anchor_pre_arc, anchor_mid_arc, anchor_post_arc;
        };
        std::vector<ClusterAnchorInfo> cluster_anchors(clusters.size());
        std::vector<int> sample_cluster(samples.size(), -1);

    last_stair_diag_.astar_hit_points = astar_stair_hits_;

        for (int ci = 0; ci < static_cast<int>(clusters.size()); ++ci) {
            const auto& cl = clusters[ci];
            double pre  = cl.is_uphill ? up_pre  : down_pre;
            double post = cl.is_uphill ? up_post : down_post;
            double w_start = cl.center_arc - pre;
            double w_end   = cl.center_arc + post;

            Eigen::Vector2d center_pos = cl.center_pos;
            bool has_match = false;
            Eigen::Vector2d matched_pt = center_pos;
            double matched_dist = -1.0;

            if (params_.use_astar_stair_anchors && !astar_stair_hits_.empty()) {
                double best_d = std::numeric_limits<double>::max();
                Eigen::Vector2d best_pt = center_pos;
                for (const auto& hp : astar_stair_hits_) {
                    double d = (hp - center_pos).norm();
                    if (d < best_d) {
                        best_d = d;
                        best_pt = hp;
                    }
                }
                if (best_d <= std::max(0.05, params_.astar_stair_anchor_match_max_dist)) {
                    Eigen::Vector2d shifted = best_pt;

                    if (esdf_callback_) {
                        double gx0 = 0.0, gy0 = 0.0;
                        const double d0 = esdf_callback_(best_pt.x(), best_pt.y(), &gx0, &gy0);
                        const double near_thresh =
                            std::max(0.0, params_.astar_anchor_near_obstacle_dist);
                        const double probe_dist =
                            std::max(0.01, params_.astar_anchor_tangent_probe_dist);
                        const double shift_step =
                            std::max(0.01, params_.astar_anchor_tangent_shift_step);
                        const double shift_max =
                            std::max(shift_step, params_.astar_anchor_tangent_shift_max);
                        const double improve_min =
                            std::max(0.0, params_.astar_anchor_tangent_improve_min);

                        if (d0 < near_thresh) {
                            Eigen::Vector2d t_stair(-cl.normal.y(), cl.normal.x());
                            const double tn = t_stair.norm();
                            if (tn > 1e-6) {
                                t_stair /= tn;
                                double gxp = 0.0, gyp = 0.0;
                                double gxm = 0.0, gym = 0.0;
                                const double d_plus = esdf_callback_(
                                    (best_pt + t_stair * probe_dist).x(),
                                    (best_pt + t_stair * probe_dist).y(),
                                    &gxp, &gyp);
                                const double d_minus = esdf_callback_(
                                    (best_pt - t_stair * probe_dist).x(),
                                    (best_pt - t_stair * probe_dist).y(),
                                    &gxm, &gym);

                                const double sign = (d_plus >= d_minus) ? 1.0 : -1.0;
                                const int steps = std::max(
                                    1, static_cast<int>(std::floor(shift_max / shift_step + 1e-6)));
                                double best_local_dist = d0;

                                for (int si = 0; si < steps; ++si) {
                                    const Eigen::Vector2d cand = shifted + sign * t_stair * shift_step;
                                    double gxc = 0.0, gyc = 0.0;
                                    const double d_cand = esdf_callback_(cand.x(), cand.y(), &gxc, &gyc);
                                    if (d_cand > best_local_dist + improve_min) {
                                        shifted = cand;
                                        best_local_dist = d_cand;
                                        if (best_local_dist >= near_thresh) {
                                            break;
                                        }
                                    } else {
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    center_pos = shifted;
                    has_match = true;
                    matched_pt = shifted;
                }
            }

            double dir = cl.is_uphill ? 1.0 : -1.0;
            auto& anc = cluster_anchors[ci];
            anc.t_stair = Eigen::Vector2d(-cl.normal.y(), cl.normal.x());
            anc.center_pos = center_pos;
            anc.anchor_pre  = center_pos - dir * cl.normal * pre;
            anc.anchor_mid  = center_pos;
            anc.anchor_post = center_pos + dir * cl.normal * post;
            anc.anchor_pre_arc  = w_start;
            anc.anchor_mid_arc  = cl.center_arc;
            anc.anchor_post_arc = w_end;

            StairAlignDiagnostics::ClusterInfo ci_diag;
            ci_diag.center_pos = center_pos;
            ci_diag.normal = cl.normal;
            ci_diag.is_uphill = cl.is_uphill;
            ci_diag.window_arc_start = w_start;
            ci_diag.window_arc_end = w_end;
            ci_diag.anchor_pre = anc.anchor_pre;
            ci_diag.anchor_mid = anc.anchor_mid;
            ci_diag.anchor_post = anc.anchor_post;
            ci_diag.has_astar_match = has_match;
            ci_diag.astar_match_point = matched_pt;
            if (has_match) {
                matched_dist = (anc.anchor_mid - matched_pt).norm();
            }
            ci_diag.astar_match_dist = has_match ? matched_dist : -1.0;
            last_stair_diag_.clusters.push_back(ci_diag);

            for (int si = 0; si < static_cast<int>(samples.size()); ++si) {
                if (samples[si].arc_len >= w_start - 1e-6 &&
                    samples[si].arc_len <= w_end + 1e-6 &&
                    sample_cluster[si] < 0) {
                    sample_cluster[si] = ci;
                }
            }
        }

        // Step 4: 计算航向/横向走廊代价与解析梯度
        double sum_err = 0.0, max_err = 0.0;
        double sum_lat = 0.0, max_lat = 0.0;
        int cost_count = 0;
        const double corr_hw = std::max(0.01, params_.stair_corridor_half_width);

        for (int si = 0; si < static_cast<int>(samples.size()); ++si) {
            if (sample_cluster[si] < 0) continue;

            const auto& s = samples[si];
            const int ci = sample_cluster[si];
            const Eigen::Vector2d& n_vec = clusters[ci].normal;
            const auto& anc = cluster_anchors[ci];

            double t_norm = s.tangent.norm();
            if (t_norm < 1e-6) continue;

            Eigen::Vector2d t_hat = s.tangent / t_norm;
            double a = t_hat.dot(n_vec);

            // J_heading: lambda_heading * (1 - (T̂·n)²)
            double c_heading = 1.0 - a * a;
            cost += params_.lambda_stair_align * c_heading;

            double err_deg = std::acos(std::min(1.0, std::abs(a)))
                             * 180.0 / M_PI;
            sum_err += err_deg;
            max_err = std::max(max_err, err_deg);
            cost_count++;

            last_stair_diag_.samples.push_back(
                {s.pos, t_hat, n_vec, err_deg});

            int ctrl_idx[4] = {
                s.span_k - 3, s.span_k - 2, s.span_k - 1, s.span_k};

            // heading 梯度: dc/dP_j = -2a·B'_j(s)/(h·||T||) · (n - a·T̂)
            {
                Eigen::Vector2d v = n_vec - a * t_hat;
                double w_coeff = -2.0 * a / (h * t_norm);
                double dB[4];
                cubicBasisDeriv(s.local_s, dB);
                for (int j = 0; j < 4; ++j) {
                    int idx = ctrl_idx[j];
                    if (idx >= opt_start_ && idx < opt_end_) {
                        Eigen::Vector2d g =
                            (params_.lambda_stair_align * w_coeff * dB[j]) * v;
                        grad.row(idx - opt_start_) += g.transpose();
                    }
                }
            }

            // J_lateral: lambda_lateral * huber(d_t / corridor_half_width)
            if (params_.lambda_stair_lateral > 0.0) {
                double d_t = (s.pos - anc.center_pos).dot(anc.t_stair);
                double abs_d_t = std::abs(d_t);
                sum_lat += abs_d_t;
                max_lat = std::max(max_lat, abs_d_t);

                double u = d_t / corr_hw;
                double abs_u = std::abs(u);
                double huber_val, dhuber_du;
                if (abs_u <= 1.0) {
                    huber_val = 0.5 * u * u;
                    dhuber_du = u;
                } else {
                    huber_val = abs_u - 0.5;
                    dhuber_du = (u > 0.0) ? 1.0 : -1.0;
                }
                cost += params_.lambda_stair_lateral * huber_val;

                double Bvals[4];
                cubicBasis(s.local_s, Bvals);
                double g_scale = params_.lambda_stair_lateral * dhuber_du / corr_hw;
                for (int j = 0; j < 4; ++j) {
                    int idx = ctrl_idx[j];
                    if (idx >= opt_start_ && idx < opt_end_) {
                        Eigen::Vector2d g = (g_scale * Bvals[j]) * anc.t_stair;
                        grad.row(idx - opt_start_) += g.transpose();
                    }
                }
            }
        }

        // Step 5: 锚点代价 J_anchor — 对每个簇找最接近锚点弧长的采样点
        if (params_.lambda_stair_anchor > 0.0) {
            for (int ci = 0; ci < static_cast<int>(clusters.size()); ++ci) {
                const auto& anc = cluster_anchors[ci];
                double target_arcs[3] = {
                    anc.anchor_pre_arc, anc.anchor_mid_arc, anc.anchor_post_arc};
                Eigen::Vector2d targets[3] = {
                    anc.anchor_pre, anc.anchor_mid, anc.anchor_post};

                for (int ai = 0; ai < 3; ++ai) {
                    int best_si = -1;
                    double best_dist = std::numeric_limits<double>::max();
                    for (int si = 0; si < static_cast<int>(samples.size()); ++si) {
                        if (sample_cluster[si] != ci) continue;
                        double d = std::abs(samples[si].arc_len - target_arcs[ai]);
                        if (d < best_dist) {
                            best_dist = d;
                            best_si = si;
                        }
                    }
                    if (best_si < 0) continue;

                    const auto& s = samples[best_si];
                    Eigen::Vector2d err = s.pos - targets[ai];
                    double sq = err.squaredNorm();
                    cost += params_.lambda_stair_anchor * sq;

                    double Bvals[4];
                    cubicBasis(s.local_s, Bvals);
                    int ctrl_idx[4] = {
                        s.span_k - 3, s.span_k - 2, s.span_k - 1, s.span_k};
                    for (int j = 0; j < 4; ++j) {
                        int idx = ctrl_idx[j];
                        if (idx >= opt_start_ && idx < opt_end_) {
                            Eigen::Vector2d g =
                                (2.0 * params_.lambda_stair_anchor * Bvals[j]) * err;
                            grad.row(idx - opt_start_) += g.transpose();
                        }
                    }
                }
            }
        }

        last_stair_diag_.num_clusters =
            static_cast<int>(clusters.size());
        last_stair_diag_.num_curve_samples = cost_count;
        last_stair_diag_.mean_heading_err_deg =
            (cost_count > 0) ? sum_err / cost_count : 0.0;
        last_stair_diag_.max_heading_err_deg = max_err;
        last_stair_diag_.mean_lateral_disp =
            (cost_count > 0) ? sum_lat / cost_count : 0.0;
        last_stair_diag_.max_lateral_disp = max_lat;
    }

    // ===== 均匀三次 B-spline 基函数 =====
    static void cubicBasis(double s, double B[4]) {
        double oms = 1.0 - s;
        double s2 = s * s;
        double s3 = s2 * s;
        B[0] = oms * oms * oms / 6.0;
        B[1] = (3.0 * s3 - 6.0 * s2 + 4.0) / 6.0;
        B[2] = (-3.0 * s3 + 3.0 * s2 + 3.0 * s + 1.0) / 6.0;
        B[3] = s3 / 6.0;
    }

    static void cubicBasisDeriv(double s, double dB[4]) {
        double oms = 1.0 - s;
        double s2 = s * s;
        dB[0] = -oms * oms / 2.0;
        dB[1] = (9.0 * s2 - 12.0 * s) / 6.0;
        dB[2] = (-9.0 * s2 + 6.0 * s + 3.0) / 6.0;
        dB[3] = s2 / 2.0;
    }

    // 求值均匀三次 B-spline 位置与切向
    // t ∈ [0, duration], duration = (n_ctrl_ - order_) * h
    void evalCubicBSpline(const Eigen::MatrixXd& ctrl_pts, double h,
                          double t, Eigen::Vector2d& pos,
                          Eigen::Vector2d& tangent,
                          int& span_k, double& local_s) const {
        double u = std::max(0.0,
            std::min(static_cast<double>(n_ctrl_ - order_) * h, t));
        span_k = static_cast<int>(std::floor(u / h + 1e-10)) + order_;
        span_k = std::max(order_, std::min(span_k, n_ctrl_ - 1));
        local_s = (h > 1e-10)
            ? (u - static_cast<double>(span_k - order_) * h) / h
            : 0.0;
        local_s = std::max(0.0, std::min(1.0, local_s));

        double B[4], dB[4];
        cubicBasis(local_s, B);
        cubicBasisDeriv(local_s, dB);

        int indices[4] = {span_k - 3, span_k - 2, span_k - 1, span_k};
        pos.setZero();
        tangent.setZero();
        for (int j = 0; j < 4; ++j) {
            int ci = std::max(0, std::min(n_ctrl_ - 1, indices[j]));
            Eigen::Vector2d p = ctrl_pts.row(ci).head<2>();
            pos += B[j] * p;
            tangent += dB[j] * p;
        }
        tangent /= h;
    }

    BSplineOptParams params_;
    ESDFCallback esdf_callback_;
    StairNormalCallback stair_normal_callback_;

    
    int order_ = 3;
    int n_ctrl_ = 0;
    int opt_start_ = 0;
    int opt_end_ = 0;
    int n_opt_ = 0;
    
    double last_cost_ = 0;
    double last_smooth_cost_ = 0;
    double last_collision_cost_ = 0;
    double last_align_cost_ = 0;
    double last_stair_align_cost_ = 0;
    double last_opt_time_ms_ = 0;
    double bspline_interval_ = 0.0;
    StairAlignDiagnostics last_stair_diag_;
    std::vector<Eigen::Vector2d> astar_stair_hits_;
};

}  // namespace nav_components
