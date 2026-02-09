"""
使用 acados 导出 NMPC 求解器
设计目标: 50Hz+ 控制频率, 低延迟
特性: 集成 ESDF 障碍物代价 (通过运行时参数 p 注入)
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
    ocp.model.p = model_obj.params  # 运行时参数: 参考 + ESDF 距离 + 权重缩放
    ocp.model.f_expl_expr = model_obj.dynamics()
    
    # 维度
    nx = model_obj.state.shape[0]   # 5
    nu = model_obj.control.shape[0] # 2
    np_ = model_obj.np              # 9 (参考 + ESDF 距离)
    
    # 时间参数 (关键性能参数)
    T_horizon = 1.5      
    N = 50             
    ocp.solver_options.tf = T_horizon
    ocp.dims.N = N
    
    # ========== ESDF 障碍物代价参数 ==========
    esdf_safe_dist = 0.5   # 安全距离 (m)
    esdf_weight = 20.0     # ESDF 代价权重
    contouring_weight = 50.0  # 路径偏离代价权重（防止切角）
    
    # ========== 代价函数 (EXTERNAL cost) ==========
    # 参数展开
    x_ref = model_obj.x_ref
    y_ref = model_obj.y_ref
    theta_ref = model_obj.theta_ref
    v_ref = model_obj.v_ref
    omega_ref = model_obj.omega_ref
    a_ref = model_obj.a_ref
    alpha_ref = model_obj.alpha_ref
    weight_scale = model_obj.weight_scale
    
    # 角度误差 (wrap 到 [-pi, pi])
    theta_err = model_obj.angle_diff(model_obj.theta, theta_ref)
    
    # 跟踪误差
    state_err = ca.vertcat(
        model_obj.x - x_ref,
        model_obj.y - y_ref,
        theta_err,
        model_obj.v - v_ref,
        model_obj.omega - omega_ref
    )
    control_err = ca.vertcat(
        model_obj.a_lin - a_ref,
        model_obj.alpha_ang - alpha_ref
    )
    
    # Q: 位置 > 航向 > 速度 (根据URDF差速模型调整)
    Q_diag = np.array([10.0, 10.0, 5.0, 1.0, 1.0])
    R_diag = np.array([0.1, 0.1])
    Q = ca.diag(Q_diag)
    R = ca.diag(R_diag)
    
    # ESDF 违反量
    esdf_violation = model_obj.esdf_cost_expr(safe_dist=esdf_safe_dist)
    
    # 路径偏离代价（contouring error）- 防止"切西瓜"
    contouring_cost = contouring_weight * model_obj.contouring_error()
    
    tracking_cost = ca.mtimes([state_err.T, Q, state_err])
    control_cost = ca.mtimes([control_err.T, R, control_err])
    esdf_cost = esdf_weight * esdf_violation**2
    
    # 近端/终端缩放作用于 tracking + esdf + contouring，控制平滑性不缩放
    stage_cost = weight_scale * (tracking_cost + esdf_cost + contouring_cost) + control_cost
    terminal_cost = weight_scale * (tracking_cost + esdf_cost + contouring_cost)
    
    # --- 初始阶段 ---
    ocp.cost.cost_type_0 = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost_0 = stage_cost
    
    # --- 中间阶段 ---
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost = stage_cost
    
    # --- 终端阶段 ---
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost_e = terminal_cost
    
    # ========== 运行时参数默认值 ==========
    # p = [xref(7个), d_esdf=10.0, weight_scale=1.0]
    ocp.parameter_values = np.array([
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # 参考
        10.0,  # d_esdf 远离障碍物
        1.0    # weight_scale
    ])
    
    # ========== 约束 ==========
    x_min, x_max, u_min, u_max = model_obj.get_constraints()
    
    # 初始状态约束 (等式约束)
    ocp.constraints.x0 = np.zeros(nx)
    
    # 状态约束 (路径约束) - 只约束速度
    ocp.constraints.idxbx = np.array([3, 4])  # [v, omega]
    ocp.constraints.lbx = np.array([-model_obj.max_v, -model_obj.max_omega])
    ocp.constraints.ubx = np.array([model_obj.max_v, model_obj.max_omega])
    
    # 控制约束
    ocp.constraints.idxbu = np.array([0, 1])  # [a_lin, alpha_ang]
    ocp.constraints.lbu = u_min
    ocp.constraints.ubu = u_max
    
    # ========== 求解器选项 (性能关键) ==========
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'EXACT'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.nlp_solver_max_iter = 1
    
    # QP 求解器设置
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.qp_solver_cond_N = N // 2
    
    # 数值稳定性
    ocp.solver_options.levenberg_marquardt = 1e-4
    
    # ========== 代码生成 ==========
    output_dir = os.path.join(os.path.dirname(__file__), '../nmpc_solver')
    os.makedirs(output_dir, exist_ok=True)
    
    ocp.code_export_directory = output_dir
    
    # 生成求解器
    print(f"正在生成 NMPC solver 到 {output_dir}...")
    print(f"  状态维度: nx={nx}, 控制维度: nu={nu}")
    print(f"  参数维度: np={np_} (xref[7] + d_esdf + weight_scale)")
    print(f"  成本类型: EXTERNAL (tracking + ESDF + control)")
    print(f"  预测步数: N={N}, 时域: T={T_horizon}s, dt={T_horizon/N}s")
    
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