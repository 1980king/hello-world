import numpy as np
from sim_runner import run_simulation, default_neighbors


def evaluate(trajectories, R):
    N = len(trajectories)
    T = trajectories[0].shape[0]
    error_sum = 0.0
    min_dist = np.inf
    for t in range(T):
        for i in range(N):
            r_err = np.linalg.norm(trajectories[i][t]) - R
            error_sum += abs(r_err)
        for i in range(N):
            for j in range(i + 1, N):
                d = np.linalg.norm(trajectories[i][t] - trajectories[j][t])
                if d < min_dist:
                    min_dist = d
    mean_error = error_sum / (N * T)
    penalty = 10.0 * max(0.0, 2.0 - min_dist)
    return mean_error + penalty, mean_error, min_dist


def main():
    N = 4
    R = 20.0
    init_states = []
    for k in range(N):
        theta = 2 * np.pi * k / N
        pos = R * np.array([np.cos(theta), np.sin(theta)])
        vel = R * np.array([-np.sin(theta), np.cos(theta)]) * 0.2
        init_states.append((pos, vel))

    obs_list = [np.array([0.0, 0.0])]
    neighbors_list = default_neighbors(N)

    k_c_list = [0.6, 0.8, 1.0]
    k_v_list = [1.0, 1.2, 1.4]
    k_o_list = [0.4, 0.5, 0.6]
    k_r_list = [0.01, 0.02, 0.03]

    best_params = None
    best_score = np.inf

    for k_c in k_c_list:
        for k_v in k_v_list:
            for k_o in k_o_list:
                for k_r in k_r_list:
                    params = {
                        "k_c": k_c,
                        "k_v": k_v,
                        "k_r": k_r,
                        "d_r": 15,
                        "R": R,
                        "k_o": k_o,
                    }
                    trajectories, _ = run_simulation(
                        N,
                        init_states,
                        obs_list,
                        neighbors_list,
                        params,
                        dt=0.05,
                        T_final=20,
                        integrator="euler",
                    )
                    score, mean_error, min_dist = evaluate(trajectories, R)
                    print(
                        f"params k_c={k_c}, k_v={k_v}, k_o={k_o}, k_r={k_r} -> "
                        f"error={mean_error:.3f}, min_d={min_dist:.3f}, score={score:.3f}"
                    )
                    if score < best_score:
                        best_score = score
                        best_params = params.copy()

    print("Best parameters:", best_params)
    print("Best score:", best_score)


if __name__ == "__main__":
    main()
