import numpy as np

def consensus_input(p_i, v_i, p_neighbors, v_neighbors, k_c, k_v):
    """
    一致性控制项（位置+速度）.
    参数:
        p_i        : ndarray, 本机位置 (2,)
        v_i        : ndarray, 本机速度 (2,)
        p_neighbors: list of ndarray, 邻居位置 [(2,), ...]
        v_neighbors: list of ndarray, 邻居速度 [(2,), ...]
        k_c        : float, 位置一致性增益
        k_v        : float, 速度一致性增益
    返回:
        u_con      : ndarray, 一致性控制输出 (2,)
    """
    if len(p_neighbors) == 0:
        return np.zeros_like(p_i)
    pos_diff = np.sum([p_i - pj for pj in p_neighbors], axis=0)
    vel_diff = np.sum([v_i - vj for vj in v_neighbors], axis=0)
    return -k_c * pos_diff - k_v * vel_diff

def repulsive_input(p_i, p_obs_list, p_others, k_r, d_r):
    """
    斥力控制项：对障碍和其他无人机的安全避障。

    参数:
        p_i       : ndarray, 本机位置 (2,)
        p_obs_list: list of ndarray, 障碍中心 [(2,), ...]
        p_others  : list of ndarray, 其他无人机 [(2,), ...]（不含本机）
        k_r       : float, 斥力势场增益
        d_r       : float, 斥力作用半径

    返回:
        u_rep     : ndarray, 斥力控制输出 (2,)
    """
    u_rep = np.zeros_like(p_i)
    # 对障碍物的斥力
    for p_o in p_obs_list:
        diff = p_i - p_o
        dist = np.linalg.norm(diff)
        if 0 < dist < d_r:
            phi = k_r * (1/dist - 1/d_r) / (dist**2)
            u_rep += phi * (diff / dist)
    # 对其他无人机的斥力
    for p_j in p_others:
        diff = p_i - p_j
        dist = np.linalg.norm(diff)
        if 0 < dist < d_r:
            phi = k_r * (1/dist - 1/d_r) / (dist**2)
            u_rep += phi * (diff / dist)
    return u_rep

def orbit_input(p_i, R, k_o):
    """
    圆周维持控制项。

    参数:
        p_i : ndarray, 本机位置 (2,)
        R   : float, 期望圆半径
        k_o : float, 维持增益

    返回:
        u_orb : ndarray, 圆周维持控制输出 (2,)
    """
    norm = np.linalg.norm(p_i)
    if norm == 0:
        return np.zeros_like(p_i)
    e_i = norm - R
    return k_o * e_i * (p_i / norm)

def total_control(p_i, v_i, p_neighbors, v_neighbors, p_obs_list, p_others, k_c, k_v, k_r, d_r, R, k_o):
    """
    总控制输入：三项加和。
    参数:
        p_i, v_i      : 本机状态
        p_neighbors   : 邻居位置
        v_neighbors   : 邻居速度
        p_obs_list    : 障碍中心
        p_others      : 其他无人机
        k_c, k_v      : 一致性参数
        k_r, d_r      : 斥力参数
        R, k_o        : 圆周维持参数
    返回:
        u_total : ndarray, 控制输入 (2,)
    """
    u_con = consensus_input(p_i, v_i, p_neighbors, v_neighbors, k_c, k_v)
    u_rep = repulsive_input(p_i, p_obs_list, p_others, k_r, d_r)
    u_orb = orbit_input(p_i, R, k_o)
    return u_con + u_rep + u_orb

# 示例用法（调试/单元测试）
if __name__ == "__main__":
    # 构造样例状态
    p_i = np.array([10.0, 0.0])
    v_i = np.array([0.0, 1.0])
    p_neighbors = [np.array([9.5, 0.5]), np.array([10.5, -0.5])]
    v_neighbors = [np.array([0.0, 1.2]), np.array([0.0, 0.8])]
    p_obs_list = [np.array([0.0, 0.0])]
    p_others = [np.array([12.0, 1.0]), np.array([8.0, -1.0])]
    k_c, k_v, k_r, d_r, R, k_o = 0.8, 1.2, 0.02, 15, 10, 0.5

    u = total_control(p_i, v_i, p_neighbors, v_neighbors, p_obs_list, p_others, k_c, k_v, k_r, d_r, R, k_o)
    print("控制输入 u =", u)
