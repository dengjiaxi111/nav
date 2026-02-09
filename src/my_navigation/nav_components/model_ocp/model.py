"""
差速轮腿机器人非线性动力学模型
状态: [x, y, theta, v, omega]
控制: [a_lin, alpha_ang] (线加速度, 角加速度)
运行时参数 p (外部代价版):
    [x_ref, y_ref, theta_ref, v_ref, omega_ref, a_ref, alpha_ref, d_esdf, weight_scale]
说明:
    - 跟踪参考 (前7个) 由上层 C++ 在每个 shooting node 注入
    - d_esdf 仅包含 ESDF 距离 (m)，不再需要梯度或查询点
    - weight_scale 用于近端/终端权重缩放 (含 terminal_multiplier)
"""

import casadi as ca
import numpy as np


class WheellegModel:
    def __init__(self):
        # 状态变量 (5维)
        self.x = ca.SX.sym('x')          # 位置 x (m)
        self.y = ca.SX.sym('y')          # 位置 y (m)
        self.theta = ca.SX.sym('theta')  # 航向角 (rad)
        self.v = ca.SX.sym('v')          # 线速度 (m/s)
        self.omega = ca.SX.sym('omega')  # 角速度 (rad/s)
        
        self.state = ca.vertcat(self.x, self.y, self.theta, self.v, self.omega)
        
        # 控制输入 (2维)
        self.a_lin = ca.SX.sym('a_lin')      # 线加速度 (m/s²)
        self.alpha_ang = ca.SX.sym('alpha')  # 角加速度 (rad/s²)
        
        self.control = ca.vertcat(self.a_lin, self.alpha_ang)
        
        # 运行时参数 (9维): 每个 shooting node 独立设置
        self.x_ref = ca.SX.sym('x_ref')
        self.y_ref = ca.SX.sym('y_ref')
        self.theta_ref = ca.SX.sym('theta_ref')
        self.v_ref = ca.SX.sym('v_ref')
        self.omega_ref = ca.SX.sym('omega_ref')
        self.a_ref = ca.SX.sym('a_ref')
        self.alpha_ref = ca.SX.sym('alpha_ref')
        self.d_esdf = ca.SX.sym('d_esdf')        # ESDF 距离 (m)
        self.weight_scale = ca.SX.sym('weight_scale')  # 近端/终端权重缩放
        
        self.params = ca.vertcat(
            self.x_ref, self.y_ref, self.theta_ref, self.v_ref, self.omega_ref,
            self.a_ref, self.alpha_ref,
            self.d_esdf, self.weight_scale
        )
        self.np = self.params.shape[0]  # 9
        
        # 物理参数 (根据URDF: 轮距0.58m, 轮径0.08m, 差速驱动)
        # max_v = wheel_radius * max_wheel_speed
        # 轮子 velocity command 限幅 [-80, 80] rad/s (来自URDF ros2_control)
        # max_v = 0.08 * 80 = 6.4 m/s (理论值，实际受摩擦限制)
        # 实际合理值: ~1.5 m/s (考虑 Gazebo 仿真稳定性)
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
        
        # 动力学部分 (简化为一阶系统)
        dv = self.a_lin
        domega = self.alpha_ang
        
        return ca.vertcat(dx, dy, dtheta, dv, domega)
    
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

    def esdf_cost_expr(self, safe_dist=0.5):
        """ESDF 障碍物排斥违反量 (无梯度线性化)"""
        violation = ca.fmax(0.0, safe_dist - self.d_esdf)
        return violation  # 返回违反量 (标量), 在代价中平方
    
    def get_constraints(self):
        """返回状态和控制约束"""
        # 状态约束
        x_min = np.array([-np.inf, -np.inf, -np.inf, -self.max_v, -self.max_omega])
        x_max = np.array([np.inf, np.inf, np.inf, self.max_v, self.max_omega])
        
        # 控制约束
        u_min = np.array([-self.max_a, -self.max_alpha])
        u_max = np.array([self.max_a, self.max_alpha])
        
        return x_min, x_max, u_min, u_max