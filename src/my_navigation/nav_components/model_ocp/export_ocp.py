"""
使用 acados 导出 NMPC 求解器
设计目标: 50Hz+ 控制频率, 低延迟
特性: 集成 ESDF 障碍物代价 + 增广状态速度命令建模 + 底层速度一阶滞后
"""

from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np
import casadi as ca
from model import WheellegModel
import os


def export_nmpc_solver():
    # 模型开关: 通过环境变量控制是否包含惯性/滞后环节
    # NMPC_ENABLE_LAG_MODEL=1/true/on  -> 含惯性模型
    # NMPC_ENABLE_LAG_MODEL=0/false/off -> 不含惯性模型
    lag_flag = os.getenv("NMPC_ENABLE_LAG_MODEL", "1").strip().lower()
    enable_lag_model = lag_flag not in ("0", "false", "off", "no")

    # 创建模型实例
    model_obj = WheellegModel(enable_lag_model=enable_lag_model)
    
    # ========== OCP 配置 ==========
    ocp = AcadosOcp()
    
    # 模型设置
    ocp.model.name = "wheelleg_nmpc"
    ocp.model.x = model_obj.state
    ocp.model.u = model_obj.control
    ocp.model.p = model_obj.params  # 运行时参数: 参考 + ESDF + 权重
    ocp.model.f_expl_expr = model_obj.dynamics()
    
    # 维度
    nx = model_obj.state.shape[0]   # 7
    nu = model_obj.control.shape[0] # 2
    np_ = model_obj.np
    
    # 时间参数 (关键性能参数)
    T_horizon = 1.5      
    N = 50             
    ocp.solver_options.tf = T_horizon
    ocp.dims.N = N
    
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

    q_pos = model_obj.q_pos
    q_theta = model_obj.q_theta
    q_vel = model_obj.q_vel
    r_lin = model_obj.r_lin
    r_ang = model_obj.r_ang
    esdf_weight = model_obj.esdf_weight
    esdf_safe_dist = model_obj.esdf_safe_dist
    contouring_weight = model_obj.contouring_weight
    
    # 角度误差（连续差）
    # 注意：C++ 侧已将 theta_ref 做连续化注入，这里不再 wrap，避免引入 2π 局部极小值
    theta_err = model_obj.theta - theta_ref
    
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
    
    # ESDF 违反量
    esdf_violation = model_obj.esdf_cost_expr(safe_dist=esdf_safe_dist)
    
    # 路径偏离代价（contouring error）- 防止"切西瓜"
    contouring_cost = contouring_weight * model_obj.contouring_error()
    
    tracking_cost = (
        q_pos * (state_err[0] ** 2 + state_err[1] ** 2)
        + q_theta * (state_err[2] ** 2)
        + q_vel * (state_err[3] ** 2)
        # 强化角速度阻尼，抑制 180° 大转向过冲与直线衰减振荡
        + 5.0 * (state_err[4] ** 2)
    )
    control_cost = r_lin * (control_err[0] ** 2) + r_ang * (control_err[1] ** 2)
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
    # p = [xref(7), d_esdf, weight_scale,
    #      q_pos, q_theta, q_vel, r_lin, r_ang,
    #      esdf_weight, esdf_safe_dist, contouring_weight,
    #      vel_lag_tau, omega_lag_tau]
    ocp.parameter_values = np.array([
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        10.0,
        1.0,
        10.0, 5.0, 1.0,
        0.1, 0.1,
        20.0, 0.5, 50.0,
        0.6, 0.6
    ])
    
    # ========== 约束 ==========
    x_min, x_max, u_min, u_max = model_obj.get_constraints()
    
    # 初始状态约束 (等式约束)
    ocp.constraints.x0 = np.zeros(nx)
    
    # 状态约束 (路径约束) - 约束真实速度与命令速度
    ocp.constraints.idxbx = np.array([3, 4, 5, 6])  # [v, omega, v_cmd, omega_cmd]
    ocp.constraints.lbx = np.array([
        -model_obj.max_v, -model_obj.max_omega,
        -model_obj.max_v, -model_obj.max_omega
    ])
    ocp.constraints.ubx = np.array([
        model_obj.max_v, model_obj.max_omega,
        model_obj.max_v, model_obj.max_omega
    ])
    
    # 控制约束 (加速度)
    ocp.constraints.idxbu = np.array([0, 1])  # [a_cmd, alpha_cmd]
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
    print(f"  模型类型: {'Lag-augmented' if enable_lag_model else 'Direct-accel(no-lag)'}")
    print(f"  状态维度: nx={nx}, 控制维度: nu={nu}")
    print(f"  参数维度: np={np_} (xref[7] + ESDF + Q/R + weights + lag_tau)")
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