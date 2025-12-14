"""
NMPC 配置加载器 - 运行时参数管理
支持从 YAML 文件动态加载权重和约束参数
"""

import numpy as np
import yaml
from pathlib import Path
from typing import Dict, Tuple


class NMPCConfig:
    """NMPC 运行时配置管理器"""
    
    def __init__(self, config_file: str = "nmpc_config.yaml"):
        self.config_file = Path(__file__).parent / config_file
        self.config = self._load_config()
        
    def _load_config(self) -> Dict:
        """加载 YAML 配置文件"""
        if not self.config_file.exists():
            raise FileNotFoundError(f"配置文件不存在: {self.config_file}")
        
        with open(self.config_file, 'r') as f:
            return yaml.safe_load(f)
    
    def reload(self):
        """热重载配置（运行时调用）"""
        self.config = self._load_config()
        print(f"✓ 配置已重载: {self.config_file}")
    
    def get_constraint_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取约束界限
        Returns:
            u_min, u_max: 控制约束数组
        """
        c = self.config['constraints']
        u_min = np.array([
            -c['max_linear_acceleration'],
            -c['max_angular_acceleration']
        ])
        u_max = np.array([
            c['max_linear_acceleration'],
            c['max_angular_acceleration']
        ])
        return u_min, u_max
    
    def get_state_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取状态约束 (只约束速度)"""
        c = self.config['constraints']
        # 返回 [v, omega] 的界限
        x_min = np.array([
            -c['max_linear_velocity'],
            -c['max_angular_velocity']
        ])
        x_max = np.array([
            c['max_linear_velocity'],
            c['max_angular_velocity']
        ])
        return x_min, x_max
    
    def get_cost_matrices(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """获取代价函数矩阵
        Returns:
            W_0, W, W_e: 初始/路径/终端代价矩阵
        """
        w = self.config['cost_weights']
        
        # Q 矩阵 (状态权重)
        Q = np.diag([
            w['Q_position'],    # x
            w['Q_position'],    # y
            w['Q_orientation'], # theta
            w['Q_velocity'],    # v
            w['Q_velocity']     # omega
        ])
        
        # R 矩阵 (控制权重)
        R = np.diag([
            w['R_linear'],      # a_lin
            w['R_angular']      # alpha_ang
        ])
        
        # 组合权重矩阵 (nx+nu) × (nx+nu)
        W = np.block([
            [Q, np.zeros((5, 2))],
            [np.zeros((2, 5)), R]
        ])
        
        # 终端权重
        W_e = Q * w['terminal_weight_multiplier']
        
        return W, W, W_e  # W_0, W, W_e
    
    def get_horizon_params(self) -> Tuple[float, int]:
        """获取时域参数"""
        h = self.config['horizon']
        return h['prediction_time'], h['num_steps']
    
    def print_current_config(self):
        """打印当前配置"""
        print("\n========== NMPC 当前配置 ==========")
        print(f"约束:")
        for k, v in self.config['constraints'].items():
            print(f"  {k}: {v}")
        print(f"\n代价权重:")
        for k, v in self.config['cost_weights'].items():
            print(f"  {k}: {v}")
        print("===================================\n")


if __name__ == "__main__":
    # 测试配置加载
    config = NMPCConfig()
    config.print_current_config()
    
    u_min, u_max = config.get_constraint_bounds()
    print(f"控制约束: [{u_min[0]:.2f}, {u_max[0]:.2f}] m/s²")
    
    W_0, W, W_e = config.get_cost_matrices()
    print(f"路径代价矩阵形状: {W.shape}")
    print(f"终端代价矩阵形状: {W_e.shape}")
