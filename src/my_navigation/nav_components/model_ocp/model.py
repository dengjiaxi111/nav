"""
差速轮腿机器人非线性动力学模型
状态: [x, y, theta, v, omega]
控制: [a_lin, alpha_ang] (线加速度, 角加速度)
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
        
        # 物理参数 (根据实际机器人调整)
        self.max_v = 2.0        # 最大线速度 (m/s)
        self.max_omega = 2.0    # 最大角速度 (rad/s)
        self.max_a = 2.0        # 最大线加速度
        self.max_alpha = 3.0    # 最大角加速度
        
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
    
    def get_constraints(self):
        """返回状态和控制约束"""
        # 状态约束
        x_min = np.array([-np.inf, -np.inf, -np.inf, -self.max_v, -self.max_omega])
        x_max = np.array([np.inf, np.inf, np.inf, self.max_v, self.max_omega])
        
        # 控制约束
        u_min = np.array([-self.max_a, -self.max_alpha])
        u_max = np.array([self.max_a, self.max_alpha])
        
        return x_min, x_max, u_min, u_max