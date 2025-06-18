"""
sim_runner.py

多无人机一致性-斥力耦合圆形巡逻仿真主流程。
作者：XXX
日期：202X-XX-XX

依赖：
    numpy
    integrator.py
    control/con_law.py
"""

import numpy as np
from integrator import euler_update, rk4_update
from control.con_law import total_control

def default_neighbors(N):
    """
    默认环形通信拓扑，每架与前后两邻居通信。
    返回：邻接表 list of lists
    """
    return [[(i-1)%N, (i+1)%N] for i in range(N)]

def run_simulation(N, init_states, obs_list, neighbors_list, params, dt=0.1, T_final=20, integrator='euler', log_interval=1):
    """
    多无人机仿真主函数
    参数：
        N            : int, 无人机数量
        init_states  : list of (p, v) ndarray，每架无人机初始状态
        obs_list     : list of ndarray, 障碍物中心位置
        neighbors_list: list of lists，每架无人机邻居索引列表
        params       : dict, 控制参数 {'k_c','k_v','k_r','d_r','R','k_o'}
        dt           : float, 时间步长
        T_final      : float, 仿真总时长
        integrator   : str, 'euler' 或 'rk4'
        log_interval : int, 记录间隔
    返回：
        trajectories : list of ndarray, 各无人机轨迹 [N, T, 2]
        velocities   : list of ndarray, 各无人机速度 [N, T, 2]
    """
    steps = int(T_final / dt)
    T_log = steps // log_interval + 1
    trajectories = [np.zeros((T_log, 2)) for _ in range(N)]
    velocities = [np.zeros((T_log, 2)) for _ in range(N)]
    # 状态初始化
    p = [np.copy(init_states[i][0]) for i in range(N)]
    v = [np.copy(init_states[i][1]) for i in range(N)]
    for i in range(N):
        trajectories[i][0] = p[i]
        velocities[i][0] = v[i]
    log_cnt = 1

    for t_idx in range(steps):
        t = t_idx * dt
        # 记录状态，log_interval步记一次
        if (t_idx+1) % log_interval == 0:
            for i in range(N):
                trajectories[i][log_cnt] = p[i]
                velocities[i][log_cnt] = v[i]
            log_cnt += 1
        # 计算每架无人机的控制输入
        u_all = []
        for i in range(N):
            # 邻居状态
            p_neighbors = [p[j] for j in neighbors_list[i]]
            v_neighbors = [v[j] for j in neighbors_list[i]]
            # 其他无人机（斥力用，不含自身）
            p_others = [p[j] for j in range(N) if j != i]
            # 控制输入
            u = total_control(
                p[i], v[i],
                p_neighbors, v_neighbors,
                obs_list, p_others,
                params['k_c'], params['k_v'],
                params['k_r'], params['d_r'],
                params['R'], params['k_o']
            )
            u_all.append(u)
        # 状态积分
        for i in range(N):
            if integrator == 'rk4':
                def u_func(ti, pi, vi):
                    # 控制律需传入邻居、障碍、参数，可传当前邻居状态（假设静态近似）
                    p_neighbors = [p[j] for j in neighbors_list[i]]
                    v_neighbors = [v[j] for j in neighbors_list[i]]
                    p_others = [p[j] for j in range(N) if j != i]
                    return total_control(
                        pi, vi,
                        p_neighbors, v_neighbors,
                        obs_list, p_others,
                        params['k_c'], params['k_v'],
                        params['k_r'], params['d_r'],
                        params['R'], params['k_o']
                    )
                p_next, v_next = rk4_update(p[i], v[i], u_func, dt, t)
            else:
                p_next, v_next = euler_update(p[i], v[i], u_all[i], dt)
            p[i], v[i] = p_next, v_next
    return trajectories, velocities

# -------------------- 示例用法 -----------------------

if __name__ == '__main__':
    # 示例参数
    N = 4
    init_states = []
    R = 20
    for k in range(N):
        # 均匀分布在圆上，初速度为切向
        theta = 2 * np.pi * k / N
        pos = R * np.array([np.cos(theta), np.sin(theta)])
        vel = R * np.array([-np.sin(theta), np.cos(theta)]) * 0.2
        init_states.append( (pos, vel) )
    obs_list = [np.array([0.0, 0.0])]
    neighbors_list = default_neighbors(N)
    params = {'k_c':0.8, 'k_v':1.2, 'k_r':0.02, 'd_r':15, 'R':R, 'k_o':0.5}
    dt = 0.05
    T_final = 30

    trajectories, velocities = run_simulation(
        N, init_states, obs_list, neighbors_list, params, dt, T_final, integrator='euler'
    )

    # 可视化（需要plot_utils.py）
    try:
        from plot_utils import plot_trajectories, plot_error
        plot_trajectories(trajectories)
        plot_error(trajectories, R)
    except ImportError:
        print("可视化模块未找到，可单独运行plot_utils.py查看轨迹。")
