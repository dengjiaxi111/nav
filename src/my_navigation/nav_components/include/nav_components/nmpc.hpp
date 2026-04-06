// NMPC controller - 基于 acados 的非线性模型预测控制器
// 作为 ControllerBase 插件集成到导航框架

#pragma once
#include <nav_core/controller_base.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <vector>
#include <mutex>

// 前向声明 acados solver (使用正确的类型名称)
extern "C" {
    struct wheelleg_nmpc_solver_capsule;
}

// 前向声明地图接口
namespace nav_components {
    class LayeredMapManager;
}

namespace nav_components {

/**
 * @brief NMPC 路径跟踪控制器
 * 
 * 特性:
 * - 差速轮腿机器人非线性动力学模型
 * - 实时优化求解 (50Hz+)
 * - 运行时参数可调 (通过 YAML 配置)
 * - ESDF 障碍物代价 (通过 acados 运行时参数 p 注入)
 */
class NMPC : public nav_core::ControllerBase {
public:
    NMPC();
    ~NMPC() override;
    
    // ========== ControllerBase 接口实现 ==========
    void initialize(rclcpp::Node* node) override;
    void setMap(nav_core::MapInterface::Ptr map) override;
    void setPath(const nav_msgs::msg::Path& path) override;
    nav_core::ControlResult computeVelocity(
        const geometry_msgs::msg::PoseStamped& current_pose,
        geometry_msgs::msg::Twist& cmd_vel) override;
    void setTolerance(double xy_tol, double yaw_tol) override;
    void reset() override;

private:
    // ========== 核心算法 ==========
    /**
     * @brief 求解 NMPC 优化问题
      * @param x0 当前状态 [x, y, theta, v, omega, v_cmd, omega_cmd]
     * @param yref 参考轨迹序列 (N+1个点)
      * @param u_opt 输出最优控制 [a_cmd, alpha_cmd]
     * @return 求解状态 (0=成功)
     */
    int solveNMPC(const std::vector<double>& x0,
                  const std::vector<std::vector<double>>& yref,
                  std::vector<double>& u_opt);
    
    /**
     * @brief 从全局路径提取局部参考轨迹
     * @param current_pose 当前位姿
     * @param horizon_length 提取长度 (米)
     * @return 参考轨迹序列 [[x,y,theta,v,omega], ...]
     */
    std::vector<std::vector<double>> extractLocalReference(
        const geometry_msgs::msg::Pose& current_pose,
        double horizon_length);
    
    /**
     * @brief 查找路径上距离当前位置最近的点
     */
    int findNearestPathPoint(const geometry_msgs::msg::Pose& pose);
    
    /**
     * @brief 更新 NMPC 运行时参数 (从 YAML 热重载)
     */
    void updateNMPCParameters();
    
    /**
     * @brief 发布预测轨迹到 RViz
     * @param header 消息头
     * @param x0 当前状态
     */
    void publishPredictedPath(const std_msgs::msg::Header& header, 
                             const std::vector<double>& x0);

    /**
     * @brief 从 solver 预测轨迹中采样前瞻时刻的命令状态
     * @param preview_time 前瞻时间 (秒)
     * @param v_cmd 输出线速度命令
     * @param omega_cmd 输出角速度命令
     * @return true 表示采样成功
     */
    bool samplePreviewCommand(double preview_time, double& v_cmd, double& omega_cmd);

    /**
     * @brief 为每个 shooting node 查询 ESDF 并注入到 acados 参数 p
     * 参数格式 p = [xref(7), d_esdf, weight_scale,
     *               q_pos, q_theta, q_vel, r_lin, r_ang,
     *               esdf_weight, esdf_safe_dist, contouring_weight,
     *               vel_lag_tau, omega_lag_tau, q_omega]
     * @param yref 参考轨迹（用于查询位置）
     */
    void injectEsdfParameters(const std::vector<std::vector<double>>& yref,
                              const std::vector<double>& theta_adjusted);

    /**
     * @brief 计算路径指定段的最大曲率
     * @param start_idx 起始路径点索引
     * @param num_points 向前检查的路径点数量
     * @return 最大曲率 (1/m)
     */
    double computeLocalMaxCurvature(int start_idx, int num_points) const;
    
    // ========== 状态变量 ==========
    rclcpp::Node* node_;
    nav_msgs::msg::Path global_path_;        // 全局路径
    nav_msgs::msg::Path predicted_path_;     // NMPC 预测轨迹
    
    double xy_tolerance_ = 0.1;
    double yaw_tolerance_ = 0.1;
    int nearest_idx_ = 0;  // 最近路径点索引
    
    // NMPC 状态
    std::vector<double> last_state_;   // 上一次状态 [x,y,theta,v,omega,v_cmd,omega_cmd]
    std::vector<double> last_control_; // 上一次控制 [a_cmd,alpha_cmd]
    bool initialized_ = false;
    
    // 地图接口（ESDF 查询）
    std::shared_ptr<LayeredMapManager> map_manager_;
    
    // 里程计反馈（来自 small_point_lio）
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::Odometry latest_odom_;
    bool odom_received_ = false;
    mutable std::mutex odom_mutex_;
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // ========== acados Solver ==========
    wheelleg_nmpc_solver_capsule* acados_ocp_capsule_ = nullptr;
    int N_horizon_ = 50;
    double T_horizon_ = 1.5;
    
    // 参数总数必须与 model.py 中 self.params 的维度一致（当前 20 = 原19 + q_omega）
    static constexpr int NP_PARAM = 20;
    
    // ========== 参数配置 ==========
    struct NMPCParams {
        // 运动约束
        double max_linear_vel = 2.0;
        double max_angular_vel = 2.0;
        double max_linear_accel = 2.0;
        double max_angular_accel = 3.0;
        bool allow_reverse = false;

        // 代价权重（运行时参数注入）
        double Q_position = 10.0;
        double Q_orientation = 5.0;
        double Q_velocity = 1.0;
        double Q_omega = 5.0;      // 角速度跟踪权重（独立于 Q_velocity）
        double R_linear = 0.1;
        double R_angular = 0.1;

        // ESDF 与轮廓代价权重（运行时参数注入）
        double esdf_weight = 20.0;
        double esdf_safe_dist = 0.5;
        double contouring_weight = 50.0;

        // ESDF 代价开关
        bool enable_esdf_cost = true;      // 启用 ESDF 代价

        // 局部参考轨迹提取
        // horizon_length: 参考轨迹覆盖的实际路径弧长 (米)，与 desired_velocity 解耦
        // desired_velocity: 参考速度基准值，与位置间距无关
        double horizon_length = 2.0;       // 参考覆盖弧长 (米)
        double desired_velocity = 1.0;     // 期望巡航速度 (m/s)
        bool use_omega_ref_from_path = false;

        // 曲率自适应 horizon：弯道时自动缩短覆盖弧长，减少前视距离
        bool enable_curvature_horizon_adapt = true;
        double horizon_kappa_scale = 0.5;  // 曲率对 horizon 的压缩系数
        double horizon_min_length = 0.4;   // horizon 最小值 (米)

        // 终点减速与起步对齐（参考轨迹整形）
        double goal_decel_start_dist = 1.5;      // 距终点开始减速距离 (m)
        double goal_crawl_speed = 0.15;          // 终点前爬行速度下限 (m/s)
        bool enable_goal_speed_guard = true;     // 启用近终点速度安全包络
        double goal_speed_guard_dist_scale = 1.5; // 包络生效距离倍率(相对 goal_decel_start_dist)
        double goal_speed_guard_decel_scale = 1.2; // 安全刹车减速度倍率(相对 max_linear_accel)
        double goal_speed_guard_abs_floor = 0.5; // 安全刹车最小减速度(m/s^2)
        double pivot_turn_heading_thresh = 0.785; // 航向误差大于该阈值时原地转向 (rad)
        double heading_slowdown_start = 0.2;     // 航向误差大于该阈值开始降速 (rad)
        double heading_slowdown_min_factor = 0.1; // 航向降速最小倍率

        // 高曲率路段速度衰减（参考速度整形）
        bool enable_curvature_speed_decay = true;   // 启用曲率速度衰减
        double curvature_decay_kappa_ref = 0.8;     // 曲率参考值 (1/m)
        double curvature_decay_min_factor = 0.45;   // 曲率衰减最小倍率

        // 速度反馈融合（用于缓解物理里程计对指令的拖拽）
        // x0_vel = alpha * odom_vel + (1-alpha) * last_cmd_vel
        // alpha=1.0 为纯闭环里程计；alpha=0.0 为纯指令前馈
        double odom_feedback_alpha = 0.0;

        // 横向误差自适应速度缩减 (图片策略)
        double lateral_error_threshold = 0.15;  // 横向误差阈值 (m)，超过此值启用速度缩减

        // 近端权重递增
        double near_weight_multiplier = 2.0;  // 前 1/4 时域权重倍数

        // 终端权重缩放
        double terminal_multiplier = 2.0;

        // 底层闭环速度一阶滞后模型时间常数（秒）
        double vel_lag_tau = 0.6;
        double omega_lag_tau = 0.6;

        // 输出前瞻时间：从预测轨迹中取未来该时刻的命令作为实际输出
        double output_preview_time = 0.0;
    } params_;
    
    // ========== ROS 接口 ==========
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
    
    // 性能统计
    struct {
        double avg_solve_time_ms = 0.0;
        double max_solve_time_ms = 0.0;
        int solve_count = 0;
        int consecutive_failures = 0;
    } stats_;

    // 终点减速锁存：进入减速区后只允许参考速度上限递减，避免速度回跳
    bool goal_brake_latched_ = false;
    double goal_brake_speed_cap_ = 1e9;
    bool pivot_turn_active_ = false;
    double pivot_turn_heading_error_ = 0.0;
};

}  // namespace nav_components
