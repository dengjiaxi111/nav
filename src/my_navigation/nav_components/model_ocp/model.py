"""
差速轮腿机器人非线性动力学模型
状态: [x, y, theta, v, omega, v_cmd, omega_cmd]
控制: [a_cmd, alpha_cmd] (速度命令导数)
运行时参数 p (外部代价版):
    [x_ref, y_ref, theta_ref, v_ref, omega_ref, a_ref, alpha_ref,
     d_esdf, weight_scale,
    q_pos, q_theta, q_vel,
     r_lin, r_ang,
     esdf_weight, esdf_safe_dist, contouring_weight,
     vel_lag_tau, omega_lag_tau]
说明:
    - 跟踪参考 (前7个) 由上层 C++ 在每个 shooting node 注入
    - d_esdf 仅包含 ESDF 距离 (m)
    - weight_scale 用于近端/终端权重缩放 (含 terminal_multiplier)
    - Q/R/ESDF/contouring 权重均支持运行时注入
    - 采用增广状态建模: 通过 (v_cmd, omega_cmd) 表示下发给底层的速度目标
    - 使用一阶滞后模型拟合底层 LQR/轮腿系统的速度响应
    - 控制量保持为加速度，可更自然地约束命令变化率
    - q_vel 用于速度跟踪；在 OCP 代价中对角速度误差施加了缩放惩罚项
"""

import casadi as ca
import numpy as np


# 建模开关: True=包含惯性/滞后环节, False=不包含惯性环节(直接加速度模型)
DEFAULT_ENABLE_LAG_MODEL = True


class WheellegModel:
    def __init__(self, enable_lag_model: bool = DEFAULT_ENABLE_LAG_MODEL):
        self.enable_lag_model = bool(enable_lag_model)

        # 状态变量 (7维)
        self.x = ca.SX.sym('x')          # 位置 x (m)
        self.y = ca.SX.sym('y')          # 位置 y (m)
        self.theta = ca.SX.sym('theta')  # 航向角 (rad)
        self.v = ca.SX.sym('v')          # 线速度 (m/s)
        self.omega = ca.SX.sym('omega')  # 角速度 (rad/s)
        self.v_cmd = ca.SX.sym('v_cmd_state')          # 下发线速度命令 (m/s)
        self.omega_cmd = ca.SX.sym('omega_cmd_state')  # 下发角速度命令 (rad/s)
        
        self.state = ca.vertcat(
            self.x, self.y, self.theta, self.v, self.omega, self.v_cmd, self.omega_cmd
        )
        
        # 控制输入 (2维): 速度命令导数（加速度）
        self.a_lin = ca.SX.sym('a_cmd')      # 线加速度命令 (m/s^2)
        self.alpha_ang = ca.SX.sym('alpha_cmd')  # 角加速度命令 (rad/s^2)

        self.control = ca.vertcat(self.a_lin, self.alpha_ang)
        
        # 运行时参数: 每个 shooting node 独立设置
        self.x_ref = ca.SX.sym('x_ref')
        self.y_ref = ca.SX.sym('y_ref')
        self.theta_ref = ca.SX.sym('theta_ref')
        self.v_ref = ca.SX.sym('v_ref')
        self.omega_ref = ca.SX.sym('omega_ref')
        self.a_ref = ca.SX.sym('a_ref')
        self.alpha_ref = ca.SX.sym('alpha_ref')
        self.d_esdf = ca.SX.sym('d_esdf')                # ESDF 距离 (m)
        self.weight_scale = ca.SX.sym('weight_scale')    # 近端/终端权重缩放

        # 运行时权重
        self.q_pos = ca.SX.sym('q_pos')
        self.q_theta = ca.SX.sym('q_theta')
        self.q_vel = ca.SX.sym('q_vel')
        self.r_lin = ca.SX.sym('r_lin')
        self.r_ang = ca.SX.sym('r_ang')
        self.esdf_weight = ca.SX.sym('esdf_weight')
        self.esdf_safe_dist = ca.SX.sym('esdf_safe_dist')
        self.contouring_weight = ca.SX.sym('contouring_weight')

        # 底层闭环一阶滞后时间常数（运行时注入）
        self.vel_lag_tau = ca.SX.sym('vel_lag_tau')
        self.omega_lag_tau = ca.SX.sym('omega_lag_tau')

        # 角速度跟踪权重（独立于 q_vel，运行时注入）
        self.q_omega = ca.SX.sym('q_omega')
        
        self.params = ca.vertcat(
            self.x_ref, self.y_ref, self.theta_ref, self.v_ref, self.omega_ref,
            self.a_ref, self.alpha_ref,
            self.d_esdf, self.weight_scale,
            self.q_pos, self.q_theta, self.q_vel,
            self.r_lin, self.r_ang,
            self.esdf_weight, self.esdf_safe_dist, self.contouring_weight,
            self.vel_lag_tau, self.omega_lag_tau,
            self.q_omega
        )
        self.np = self.params.shape[0]
        
        self.max_v = 1.5        # 最大线速度 (m/s)
        self.max_omega = 3.0    # 最大角速度 (rad/s): v_diff / track_width
        self.max_a = 2.0        # 最大线加速度
        self.max_alpha = 4.0    # 最大角加速度
        
    def dynamics(self):
        """连续时间动力学模型 dx/dt = f(x, u)"""
        # 运动学部分
        dx = self.v * ca.cos(self.theta)
        dy = self.v * ca.sin(self.theta)
        dtheta = self.omega

        if self.enable_lag_model:
            # 动力学部分(含惯性): 速度命令经过一阶滞后后作用于真实速度
            tau_v = ca.fmax(self.vel_lag_tau, 0.05)
            tau_w = ca.fmax(self.omega_lag_tau, 0.05)
            dv = (self.v_cmd - self.v) / tau_v
            domega = (self.omega_cmd - self.omega) / tau_w
        else:
            # 动力学部分(不含惯性): 真实速度由加速度直接积分
            dv = self.a_lin
            domega = self.alpha_ang

        # 控制输入作用在命令状态（控制量保持加速度）
        dv_cmd = self.a_lin
        domega_cmd = self.alpha_ang
        
        return ca.vertcat(dx, dy, dtheta, dv, domega, dv_cmd, domega_cmd)
    
    @staticmethod
    def angle_diff(a, b):
        """wrap(a - b) to [-pi, pi]"""
        return ca.atan2(ca.sin(a - b), ca.cos(a - b))

    def contouring_error(self):
        """计算相对参考路径的横向偏离（contouring error）
        
        使用参考点的切线方向 theta_ref，计算当前位置的正交偏离距离
        这可以防止 MPC 预测轨迹过度偏离参考路径（"切西瓜"现象）
        
        Returns:
            横向偏离距离的平方 (m²)
        """
        dx = self.x - self.x_ref
        dy = self.y - self.y_ref
        
        # 投影到参考轨迹的法向 (正交方向)
        # 法向单位矢量: [-sin(theta_ref), cos(theta_ref)]
        e_contour = -ca.sin(self.theta_ref) * dx + ca.cos(self.theta_ref) * dy
        
        return e_contour**2

    def esdf_cost_expr(self, safe_dist):
        """ESDF 障碍物排斥违反量 (无梯度线性化)"""
        violation = ca.fmax(0.0, safe_dist - self.d_esdf)
        return violation  # 返回违反量 (标量), 在代价中平方
    
    def get_constraints(self):
        """返回状态和控制约束"""
        # 状态约束
        x_min = np.array([
            -np.inf, -np.inf, -np.inf,
            -self.max_v, -self.max_omega,
            -self.max_v, -self.max_omega
        ])
        x_max = np.array([
            np.inf, np.inf, np.inf,
            self.max_v, self.max_omega,
            self.max_v, self.max_omega
        ])
        
        # 控制约束（加速度幅值）
        u_min = np.array([-self.max_a, -self.max_alpha])
        u_max = np.array([self.max_a, self.max_alpha])
        
        return x_min, x_max, u_min, u_max