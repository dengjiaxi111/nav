// NMPC controller - 基于 acados 的非线性模型预测控制器
// 作为 ControllerBase 插件集成到导航框架

#pragma once
#include <nav_core/controller_base.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
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
     * @param x0 当前状态 [x, y, theta, v, omega]
     * @param yref 参考轨迹序列 (N+1个点)
     * @param u_opt 输出最优控制 [a_lin, alpha_ang]
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
     * @brief 为每个 shooting node 查询 ESDF 并注入到 acados 参数 p
     * 参数格式 p = [d_esdf, grad_x, grad_y, x_query, y_query]
     * @param yref 参考轨迹（用于查询位置）
     */
    void injectEsdfParameters(const std::vector<std::vector<double>>& yref,
                              const std::vector<double>& theta_adjusted);
    
    // ========== 状态变量 ==========
    rclcpp::Node* node_;
    nav_msgs::msg::Path global_path_;        // 全局路径
    nav_msgs::msg::Path predicted_path_;     // NMPC 预测轨迹
    
    double xy_tolerance_ = 0.1;
    double yaw_tolerance_ = 0.1;
    int nearest_idx_ = 0;  // 最近路径点索引
    
    // NMPC 状态
    std::vector<double> last_state_;   // 上一次状态 [x,y,theta,v,omega]
    std::vector<double> last_control_; // 上一次控制 [a,alpha]
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
    int N_horizon_ = 40;
    double T_horizon_ = 2.0;
    
    static constexpr int NP_PARAM = 9;  // [xref(7), d_esdf, weight_scale]
    
    // ========== 参数配置 ==========
    struct NMPCParams {
        // 运动约束
        double max_linear_vel = 2.0;
        double max_angular_vel = 2.0;
        double max_linear_accel = 2.0;
        double max_angular_accel = 3.0;
        bool allow_reverse = false;
        
        // 代价权重
        double Q_position = 10.0;
        double Q_orientation = 5.0;
        double Q_velocity = 1.0;
        double R_linear = 0.1;
        double R_angular = 0.1;
        double terminal_multiplier = 2.0;
        
        // ESDF 障碍物代价 (在 solver 内部使用)
        double esdf_weight = 20.0;         // ESDF 代价权重
        double esdf_safe_dist = 0.5;       // 安全距离 (m)
        bool enable_esdf_cost = true;      // 启用 ESDF 代价
        
        // 局部参考提取
        double horizon_length = 3.0;       // 提取前方路径长度 (米)
        double desired_velocity = 1.0;     // 期望速度
        
        // 横向误差自适应速度缩减 (图片策略)
        double lateral_error_threshold = 0.15;  // 横向误差阈值 (m)，超过此值启用速度缩减
        
        // 近端权重递增
        double near_weight_multiplier = 2.0;  // 前 1/4 时域权重倍数
    } params_;
    
    // ========== ROS 接口 ==========
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_marker_pub_;
    
    // 性能统计
    struct {
        double avg_solve_time_ms = 0.0;
        double max_solve_time_ms = 0.0;
        int solve_count = 0;
        int consecutive_failures = 0;
    } stats_;
};

}  // namespace nav_components
