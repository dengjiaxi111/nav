"""
使用 acados 导出 NMPC 求解器
设计目标: 50Hz+ 控制频率, 低延迟
"""

from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np
import casadi as ca
from model import WheellegModel
import os


def export_nmpc_solver():
    # 创建模型实例
    model_obj = WheellegModel()
    
    # ========== OCP 配置 ==========
    ocp = AcadosOcp()
    
    # 模型设置
    ocp.model.name = "wheelleg_nmpc"
    ocp.model.x = model_obj.state
    ocp.model.u = model_obj.control
    ocp.model.f_expl_expr = model_obj.dynamics()
    
    # 维度
    nx = model_obj.state.shape[0]  # 5
    nu = model_obj.control.shape[0]  # 2
    
    # 时间参数 (关键性能参数)
    T_horizon = 2.0      # 预测时域 2秒
    N = 20               # 离散化步数 (越少越快,但精度下降)
    ocp.solver_options.tf = T_horizon
    ocp.dims.N = N
    
    # ========== 代价函数 ==========
    # 跟踪代价矩阵 (调整权重以优化性能)
    Q = np.diag([10.0, 10.0, 5.0, 1.0, 1.0])  # 位置>速度
    R = np.diag([0.1, 0.1])                    # 控制平滑性
    
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    
    # 初始阶段代价 (与中间阶段相同)
    ocp.cost.cost_type_0 = 'LINEAR_LS'
    ocp.cost.W_0 = np.block([[Q, np.zeros((nx, nu))],
                             [np.zeros((nu, nx)), R]])
    ocp.cost.Vx_0 = np.vstack([np.eye(nx), np.zeros((nu, nx))])
    ocp.cost.Vu_0 = np.vstack([np.zeros((nx, nu)), np.eye(nu)])
    ocp.cost.yref_0 = np.zeros(nx + nu)
    ocp.model.cost_y_expr_0 = ca.vertcat(model_obj.state, model_obj.control)
    ocp.dims.ny_0 = nx + nu
    
    # 中间阶段代价
    ocp.cost.W = np.block([[Q, np.zeros((nx, nu))],
                           [np.zeros((nu, nx)), R]])
    ocp.cost.Vx = np.vstack([np.eye(nx), np.zeros((nu, nx))])
    ocp.cost.Vu = np.vstack([np.zeros((nx, nu)), np.eye(nu)])
    ocp.cost.yref = np.zeros(nx + nu)
    ocp.model.cost_y_expr = ca.vertcat(model_obj.state, model_obj.control)
    ocp.dims.ny = nx + nu
    
    # 终端代价 (只关注状态)
    ocp.cost.W_e = Q * 2.0  # 终端代价加权
    ocp.cost.Vx_e = np.eye(nx)
    ocp.cost.yref_e = np.zeros(nx)
    ocp.model.cost_y_expr_e = model_obj.state
    ocp.dims.ny_e = nx
    
    # ========== 约束 ==========
    x_min, x_max, u_min, u_max = model_obj.get_constraints()
    
    # 初始状态约束 (等式约束)
    ocp.constraints.x0 = np.zeros(nx)
    
    # 状态约束 (路径约束) - 只约束速度
    ocp.constraints.idxbx = np.array([3, 4])  # 索引: [v, omega]
    ocp.constraints.lbx = np.array([-model_obj.max_v, -model_obj.max_omega])
    ocp.constraints.ubx = np.array([model_obj.max_v, model_obj.max_omega])
    
    # 控制约束
    ocp.constraints.idxbu = np.array([0, 1])  # [a_lin, alpha_ang]
    ocp.constraints.lbu = u_min
    ocp.constraints.ubu = u_max
    
    # ========== 求解器选项 (性能关键) ==========
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'  # 最快
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'        # 避免二阶导数
    ocp.solver_options.integrator_type = 'ERK'                 # 显式 Runge-Kutta
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'            # 实时迭代 (单次迭代)
    ocp.solver_options.nlp_solver_max_iter = 1                # 强制实时性
    
    # QP 求解器设置
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.qp_solver_cond_N = N // 2  # 部分凝聚
    
    # 数值稳定性
    ocp.solver_options.levenberg_marquardt = 1e-4
    
    # ========== 代码生成 ==========
    output_dir = os.path.join(os.path.dirname(__file__), '../nmpc_solver')
    os.makedirs(output_dir, exist_ok=True)
    
    ocp.code_export_directory = output_dir
    
    # 生成求解器
    print(f"正在生成 NMPC solver 到 {output_dir}...")
    solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')
    print("✓ 求解器生成成功!")
    
    # 性能测试
    print("\n性能测试 (100次求解):")
    x0 = np.zeros(nx)
    solver.set(0, 'lbx', x0)
    solver.set(0, 'ubx', x0)
    
    import time
    times = []
    for _ in range(100):
        t_start = time.perf_counter()
        solver.solve()
        times.append((time.perf_counter() - t_start) * 1000)
    
    avg_time = np.mean(times)
    max_time = np.max(times)
    print(f"平均求解时间: {avg_time:.2f} ms")
    print(f"最大求解时间: {max_time:.2f} ms")
    print(f"理论控制频率: {1000/max_time:.1f} Hz")
    
    return solver


if __name__ == "__main__":
    export_nmpc_solver()