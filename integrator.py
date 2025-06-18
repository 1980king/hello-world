import numpy  numpy as np np

def euler_update(p, v, u, dtp, v, u, dtp, v, u, dtp, v, u, dtp, v, u, dtp, v, u, dtp, v, u, dtp, v, u, dt)::::::::
    """
    欧拉积分法，单步状态更新。
    参数:
        p  : ndarray, 位置向量 (2,)
        v  : ndarray, 速度向量 (2,)
        u  : ndarray, 加速度/控制输入 (2,)
        dt : float,   步长
    返回:
        p_next, v_next : ndarray, 更新后的位置与速度 (2,)
    """
    p_next = p + dt * v
    v_next = v + dt * u
    return p_next, v_next p_next, v_next

def rk4_update(p, v, u_func, dt, tp, v, u_func, dt, tp, v, u_func, dt, tp, v, u_func, dt, tp, v, u_func, dt, tp, v, u_func, dt, tp, v, u_func, dt, tp, v, u_func, dt, t)::::::::
    """
    四阶Runge-Kutta积分法，适用于二阶动力学。
    参数:
        p      : ndarray, 当前时刻位置 (2,)
        v      : ndarray, 当前时刻速度 (2,)
        u_func : function, 控制输入函数 u(t, p, v)
        dt     : float, 步长
        t      : float, 当前时刻
    返回:
        p_next, v_next : ndarray, 更新后的位置与速度 (2,)
    用法示例:
        def control_input(t, p, v):
            return ... # 控制律
        p_next, v_next = rk4_update(p, v, control_input, dt, t)
    """
    # k1
    a1 =     a1 = u_func(t, p, vt, p, v)
    k1_p = v    k1_p = v
    k1_v = a1    k1_v = a1

    # k2
    a2 =     a2 = u_func(t + t + 0.5 * dt, p +  * dt, p + 0.5 * dt * k1_p, v +  * dt * k1_p, v + 0.5 * dt * k1_v * dt * k1_v)
    k2_p = v +     k2_p = v + 0.5 * dt * k1_v * dt * k1_v
    k2_v = a2    k2_v = a2

    # k3
    a3 =     a3 = u_func(t + t + 0.5 * dt, p +  * dt, p + 0.5 * dt * k2_p, v +  * dt * k2_p, v + 0.5 * dt * k2_v * dt * k2_v)
    k3_p = v +     k3_p = v + 0.5 * dt * k2_v * dt * k2_v
    k3_v = a3    k3_v = a3

    # k4
    a4 =     a4 = u_func(t + dt, p + dt * k3_p, v + dt * k3_vt + dt, p + dt * k3_p, v + dt * k3_v)
    k4_p = v + dt * k3_v    k4_p = v + dt * k3_v
    k4_v = a4    k4_v = a4

    p_next = p +     p_next = p + (dt / dt / 6.0) *  * (k1_p + k1_p + 2*k2_p + *k2_p + 2*k3_p + k4_p*k3_p + k4_p)
    v_next = v +     v_next = v + (dt / dt / 6.0) *  * (k1_v + k1_v + 2*k2_v + *k2_v + 2*k3_v + k4_v*k3_v + k4_v)
    return p_next, v_next p_next, v_next

# 示例用法（可作为单元测试/调试用，不会影响主模块导入）
if __name__ ==  __name__ == "__main__"::
    # 假设初始状态
    p = np.    p = np.array([1.0, , 0.0])
    v = np.    v = np.array([0.0, , 1.0])
    dt =     dt = 0.1
    t =     t = 0.0
    # 恒定加速度
    def u_func(t, p, vt, p, v)::
        return np.array([0.0, -9.8])

    # Euler
    p1, v1 = euler_update(p, v, u_func(t, p, v), dt)
    print("Euler:", p1, v1)
    # RK4
    p2, v2 = rk4_update(p, v, u_func, dt, t)
    print("RK4:", p2, v2)
