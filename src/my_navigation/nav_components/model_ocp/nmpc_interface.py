"""
NMPC Solver Python 接口 - 运行时参数调整示例
展示如何在不重新生成 solver 的情况下调整参数
"""

import numpy as np
import sys
import os

# 添加 solver 路径
solver_path = os.path.join(os.path.dirname(__file__), '../nmpc_solver')
sys.path.append(solver_path)

try:
    from acados_solver import acados_solver_wheelleg_nmpc as AcadosSolver
    from nmpc_config_loader import NMPCConfig
except ImportError as e:
    print(f"❌ 导入失败: {e}")
    print("请先运行 export_ocp.py 生成 solver")
    sys.exit(1)


class NMPCController:
    """NMPC 控制器 - 支持运行时参数调整"""
    
    def __init__(self, config_file: str = "nmpc_config.yaml"):
        # 加载配置
        self.config = NMPCConfig(config_file)
        self.config.print_current_config()
        
        # 创建 solver
        self.solver = AcadosSolver()
        
        # 应用初始配置
        self.update_solver_parameters()
        
        print("✓ NMPC Controller 初始化完成")
    
    def update_solver_parameters(self):
        """更新 solver 运行时参数"""
        # 1. 更新约束界限
        u_min, u_max = self.config.get_constraint_bounds()
        x_min, x_max = self.config.get_state_bounds()
        
        for i in range(self.solver.N):
            # 更新控制约束
            self.solver.constraints_set(i, "lbu", u_min)
            self.solver.constraints_set(i, "ubu", u_max)
            
            # 更新状态约束 (只约束速度 v, omega)
            self.solver.constraints_set(i, "lbx", x_min)
            self.solver.constraints_set(i, "ubx", x_max)
        
        # 2. 更新代价函数权重
        W_0, W, W_e = self.config.get_cost_matrices()
        
        # 初始阶段
        self.solver.cost_set(0, "W", W_0)
        
        # 中间阶段
        for i in range(1, self.solver.N):
            self.solver.cost_set(i, "W", W)
        
        # 终端阶段
        self.solver.cost_set(self.solver.N, "W", W_e)
        
        print("✓ Solver 参数已更新")
    
    def reload_config(self):
        """热重载配置文件（运行时调用）"""
        self.config.reload()
        self.update_solver_parameters()
    
    def solve(self, x0: np.ndarray, yref: np.ndarray = None) -> dict:
        """
        求解 NMPC 优化问题
        
        Args:
            x0: 初始状态 [x, y, theta, v, omega]
            yref: 参考轨迹 (可选, 默认为零)
        
        Returns:
            dict: {
                'u_opt': 最优控制,
                'x_traj': 预测轨迹,
                'solve_time': 求解时间 (ms),
                'status': 求解状态
            }
        """
        # 设置初始状态
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)
        
        # 设置参考轨迹
        if yref is None:
            yref = np.zeros(7)  # [x, y, theta, v, omega, a_lin, alpha_ang]
        
        for i in range(self.solver.N):
            self.solver.set(i, "yref", yref)
        self.solver.set(self.solver.N, "yref", yref[:5])  # 终端只有状态
        
        # 求解
        import time
        t_start = time.perf_counter()
        status = self.solver.solve()
        solve_time = (time.perf_counter() - t_start) * 1000
        
        # 提取结果
        u_opt = self.solver.get(0, "u")
        x_traj = np.array([self.solver.get(i, "x") for i in range(self.solver.N + 1)])
        
        return {
            'u_opt': u_opt,
            'x_traj': x_traj,
            'solve_time': solve_time,
            'status': status
        }
    
    def __del__(self):
        """清理资源"""
        if hasattr(self, 'solver'):
            del self.solver


def demo_runtime_tuning():
    """演示运行时参数调整"""
    print("\n========== NMPC 运行时调参演示 ==========\n")
    
    # 1. 创建控制器
    controller = NMPCController()
    
    # 2. 模拟初始状态
    x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # 静止在原点
    
    # 3. 第一次求解
    print("\n--- 第一次求解 (默认参数) ---")
    result = controller.solve(x0)
    print(f"求解时间: {result['solve_time']:.3f} ms")
    print(f"最优控制: {result['u_opt']}")
    
    # 4. 修改 YAML 配置文件的提示
    print("\n--- 现在可以修改 nmpc_config.yaml ---")
    print("例如: 将 Q_position 从 10.0 改为 20.0")
    print("修改完成后按 Enter 键重载配置...")
    # input()  # 取消注释以交互式演示
    
    # 5. 热重载配置
    print("\n--- 重载配置 ---")
    controller.reload_config()
    
    # 6. 第二次求解
    print("\n--- 第二次求解 (新参数) ---")
    result = controller.solve(x0)
    print(f"求解时间: {result['solve_time']:.3f} ms")
    print(f"最优控制: {result['u_opt']}")
    
    print("\n✓ 演示完成！")
    print("关键优势: 无需重新生成 solver, 只需修改 YAML 即可调参")


if __name__ == "__main__":
    demo_runtime_tuning()
