import numpy as np
from integrator import euler_update, rk4_update
from con_law import total_control


def default_neighbors(N):
    """Create a ring topology where each UAV communicates with two neighbors."""
    return [[(i - 1) % N, (i + 1) % N] for i in range(N)]


def run_simulation(
    N,
    init_states,
    obs_list,
    neighbors_list,
    params,
    dt=0.1,
    T_final=20,
    integrator="euler",
    log_interval=1,
):
    """Simulate a fleet of UAVs following the provided control law."""

    steps = int(T_final / dt)
    T_log = steps // log_interval + 1

    trajectories = [np.zeros((T_log, 2), dtype=float) for _ in range(N)]
    velocities = [np.zeros((T_log, 2), dtype=float) for _ in range(N)]

    p = [np.array(init_states[i][0], dtype=float) for i in range(N)]
    v = [np.array(init_states[i][1], dtype=float) for i in range(N)]
    for i in range(N):
        trajectories[i][0] = p[i]
        velocities[i][0] = v[i]

    log_cnt = 1

    for t_idx in range(steps):
        t = t_idx * dt
        if (t_idx + 1) % log_interval == 0:
            for i in range(N):
                trajectories[i][log_cnt] = p[i]
                velocities[i][log_cnt] = v[i]
            log_cnt += 1

        u_all = []
        for i in range(N):
            p_neighbors = [p[j] for j in neighbors_list[i]]
            v_neighbors = [v[j] for j in neighbors_list[i]]
            p_others = [p[j] for j in range(N) if j != i]
            u = total_control(
                p[i],
                v[i],
                p_neighbors,
                v_neighbors,
                obs_list,
                p_others,
                params["k_c"],
                params["k_v"],
                params["k_r"],
                params["d_r"],
                params["R"],
                params["k_o"],
            )
            u_all.append(u)

        for i in range(N):
            if integrator == "rk4":
                def u_func(ti, pi, vi, i=i):
                    p_neighbors = [p[j] for j in neighbors_list[i]]
                    v_neighbors = [v[j] for j in neighbors_list[i]]
                    p_others = [p[j] for j in range(N) if j != i]
                    return total_control(
                        pi,
                        vi,
                        p_neighbors,
                        v_neighbors,
                        obs_list,
                        p_others,
                        params["k_c"],
                        params["k_v"],
                        params["k_r"],
                        params["d_r"],
                        params["R"],
                        params["k_o"],
                    )
                p_next, v_next = rk4_update(p[i], v[i], u_func, dt, t)
            else:
                p_next, v_next = euler_update(p[i], v[i], u_all[i], dt)
            p[i] = p_next
            v[i] = v_next

    return trajectories, velocities


if __name__ == "__main__":
    N = 4
    init_states = []
    R = 20
    for k in range(N):
        theta = 2 * np.pi * k / N
        pos = R * np.array([np.cos(theta), np.sin(theta)])
        vel = R * np.array([-np.sin(theta), np.cos(theta)]) * 0.2
        init_states.append((pos, vel))

    obs_list = [np.array([0.0, 0.0])]
    neighbors_list = default_neighbors(N)
    params = {"k_c": 0.8, "k_v": 1.2, "k_r": 0.02, "d_r": 15, "R": R, "k_o": 0.5}
    dt = 0.05
    T_final = 30

    trajectories, velocities = run_simulation(
        N,
        init_states,
        obs_list,
        neighbors_list,
        params,
        dt,
        T_final,
        integrator="euler",
    )

    try:
        from plot_utils import plot_trajectories, plot_error

        plot_trajectories(trajectories)
        plot_error(trajectories, R)
    except ImportError:
        print("Visualization utilities not found.")
