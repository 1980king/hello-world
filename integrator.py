import numpy as np


def euler_update(p, v, u, dt):
    """Perform one Euler integration step for a second order system."""
    p_next = p + dt * v
    v_next = v + dt * u
    return p_next, v_next


def rk4_update(p, v, u_func, dt, t):
    """Runge–Kutta 4th order step for a second order system.

    Parameters
    ----------
    p : ndarray
        Current position vector.
    v : ndarray
        Current velocity vector.
    u_func : callable
        Function of (t, p, v) returning the acceleration.
    dt : float
        Time step size.
    t : float
        Current time.
    """
    a1 = u_func(t, p, v)
    k1_p = v
    k1_v = a1

    a2 = u_func(t + 0.5 * dt, p + 0.5 * dt * k1_p, v + 0.5 * dt * k1_v)
    k2_p = v + 0.5 * dt * k1_v
    k2_v = a2

    a3 = u_func(t + 0.5 * dt, p + 0.5 * dt * k2_p, v + 0.5 * dt * k2_v)
    k3_p = v + 0.5 * dt * k2_v
    k3_v = a3

    a4 = u_func(t + dt, p + dt * k3_p, v + dt * k3_v)
    k4_p = v + dt * k3_v
    k4_v = a4

    p_next = p + (dt / 6.0) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p)
    v_next = v + (dt / 6.0) * (k1_v + 2 * k2_v + 2 * k3_v + k4_v)
    return p_next, v_next


if __name__ == "__main__":
    p = np.array([1.0, 0.0])
    v = np.array([0.0, 1.0])
    dt = 0.1
    t = 0.0

    def u_func(ti, pi, vi):
        return np.array([0.0, -9.8])

    p1, v1 = euler_update(p, v, u_func(t, p, v), dt)
    print("Euler:", p1, v1)
    p2, v2 = rk4_update(p, v, u_func, dt, t)
    print("RK4:", p2, v2)
