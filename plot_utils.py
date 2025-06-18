import matplotlib.pyplot as plt
import numpy as np

def plot_trajectories(trajectories, obs_list=None, R=None, title="UAV Trajectories"):
    """
    参数:
        trajectories : list of ndarray, 每个shape (T,2)
        obs_list     : list of ndarray, 障碍中心，可选
        R            : float, 期望圆半径，可选
        title        : str, 图标题
    """
    plt.figure(figsize=(7, 7))
    for i, traj in enumerate(trajectories):
        plt.plot(traj[:, 0], traj[:, 1], '-', label=f'UAV {i+1}')
        plt.plot(traj[0, 0], traj[0, 1], 'o', color='k', markersize=5, alpha=0.5)
    if obs_list is not None:
        for obs in obs_list:
            plt.plot(obs[0], obs[1], 'rx', markersize=12, label='Obstacle')
    if R is not None:
        theta = np.linspace(0, 2*np.pi, 300)
        plt.plot(R * np.cos(theta), R * np.sin(theta), 'k--', alpha=0.4, label='Desired Orbit')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title(title)
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_error(trajectories, R, title="Radial Error vs Time"):
    """
    绘制所有无人机的径向误差随时间变化曲线。    绘制所有无人机的径向误差随时间变化曲线。
    参数:    参数:
        trajectories : list of ndarray,         trajectories : list of ndarray, shape (T,T,2)
        R            : float, 期望圆半径        R            : float, 期望圆半径
        title        : str, 图标题        title        : str, 图标题
    """
    plt.figure()
    T = trajectories[0].shape[0]
    time_axis = np.arange(T)
    for i, traj in enumerate(trajectories):
        error = np.linalg.norm(traj, axis=1) - R
        plt.plot(time_axis, error, label=f'UAV {i+1}')
    plt.xlabel('Time Step')
    plt.ylabel('Radial Error (m)')
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_min_distance(trajectories, title="Minimum Inter-UAV Distance vs Time"):
    """
    绘制最小机间距离随时间变化曲线（用于检测碰撞风险）。    绘制最小机间距离随时间变化曲线（用于检测碰撞风险）。
    参数:    参数:
        trajectories : list of ndarray,         trajectories : list of ndarray, shape (T,T,2)
        title        : str, 图标题        title        : str, 图标题
    """
    N = len(trajectories)
    T = trajectories[0].shape[0]
    min_dists = np.zeros(T)
    for t in range(T):
        min_dist = np.inf
        for i in range(N):
            for j in range(i+1, N):
                d = np.linalg.norm(trajectories[i][t] - trajectories[j][t])
                if d < min_dist:
                    min_dist = d
        min_dists[t] = min_dist
    plt.figure()
    plt.plot(np.arange(T), min_dists, 'r-')
    plt.xlabel('Time Step')
    plt.ylabel('Minimum Distance (m)')
    plt.title(title)
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# 示例用法（支持单独运行调试）
if __name__ == "__main__":
    # 生成简单圆轨迹数据测试可视化
    N = 3
    T = 200
    R = 20
    theta = np.linspace(0, 2*np.pi, T)
    trajectories = [np.stack([R * np.cos(theta + 2*np.pi*i/N), R * np.sin(theta + 2*np.pi*i/N)], axis=1) for i in range(N)]
    obs_list = [np.array([0.0, 0.0])]
    plot_trajectories(trajectories, obs_list, R)
    plot_error(trajectories, R)
    plot_min_distance(trajectories)
