import numpy as np
import pinocchio as pin

# -----------------------------
# User inputs / robot definition
# -----------------------------

URDF_PATH = "ur3e_wrist12.urdf"

# Example: replace with your real values
q_actual  = np.array([ 0.8, -2.2,  2.2, -1.8,  1.8,  0.8])
q_desired = np.array([-0.8, -1.0,  1.4,  1.8, -1.8, -0.8])

# Torque limits (Nm)
tau_limits = np.array([54.0, 54.0, 28.0, 9.0, 9.0, 9.0])

# Joint velocity limits (rad/s) – UR3e style
joint_vel_limits = np.array([3.14, 3.14, 3.14, 6.28, 6.28, 6.28])

# Max joint acceleration (rad/s^2) – scalar for simplicity
max_acc = 5.0  # example


# -----------------------------
# Model loading
# -----------------------------

model = pin.buildModelFromUrdf(URDF_PATH)
data = model.createData()

assert model.nq == 6 and model.nv == 6, "This script assumes a 6-DOF arm."

# Direction of motion in joint space
v_dir = q_desired - q_actual

if np.allclose(v_dir, 0):
    raise ValueError("q_actual and q_desired are identical.")


# -----------------------------
# Core: torque-based max scaling
# -----------------------------

def compute_s_max_at_q(q, v_dir, tau_limits, max_acc):
    """
    Given configuration q and direction v_dir, compute the maximum scalar s
    such that v = s * v_dir respects torque limits.
    """
    pin.forwardKinematics(model, data, q)

    # Gravity torque
    tau_g = pin.computeGeneralizedGravity(model, data, q)

    # RNEA at unit speed along v_dir
    v_unit = v_dir
    a_max = np.full(model.nv,max_acc)
    tau_unit = pin.rnea(model, data, q, v_unit, a_max)

    dynamic_cost = tau_unit - tau_g  # torque contribution from unit-speed motion

    s_factors = []
    for i in range(model.nv):
        available_tau = tau_limits[i] - abs(tau_g[i])
        cost = dynamic_cost[i]

        if available_tau <= 0:
            # Already over limit from gravity alone
            s_factors.append(0.0)
            continue

        if abs(cost) < 1e-9:
            # This joint is not significantly affected by motion in v_dir
            s_factors.append(np.inf)
            continue

        s_i = np.sqrt(available_tau / abs(cost))
        s_factors.append(s_i)

    s_max = min(s_factors)
    return s_max, tau_g, tau_unit, dynamic_cost


# -----------------------------
# Path parameterization: q(s)
# -----------------------------

def q_of_s(s, q0, q1):
    return q0 + s * (q1 - q0)


# -----------------------------
# Sample path and compute s_max(s)
# -----------------------------

def sample_path_s_max(q0, q1, v_dir, tau_limits, max_acc, N=200):
    s_values = np.linspace(0.0, 1.0, N)
    s_max_values = []

    for s in s_values:
        q = q_of_s(s, q0, q1)
        s_max, _, _, _ = compute_s_max_at_q(q, v_dir, tau_limits,max_acc)
        s_max_values.append(s_max)

    return s_values, np.array(s_max_values)

# -----------------------------
# Tsync-based synchronized speed limit
# -----------------------------

def compute_s_dot_sync(q0, q1, v_dir, joint_vel_limits, max_acc):
    dq = np.abs(q1 - q0)
    T_i = np.zeros(6)

    for i in range(6):
        dq_i = dq[i]
        vmax_i = joint_vel_limits[i]
        a = max_acc

        # Check if motion is triangular or trapezoidal
        if dq_i < (vmax_i**2 / a):
            # Triangular profile
            T_i[i] = 2 * np.sqrt(dq_i / a)
        else:
            # Trapezoidal profile
            T_i[i] = dq_i / vmax_i + vmax_i / a

    Tsync = np.max(T_i)

    # Synchronized joint velocities
    q_dot_sync = dq / Tsync

    # Convert to path speed
    s_dot_candidates = []
    for i in range(6):
        if abs(v_dir[i]) > 1e-9:
            s_dot_candidates.append(q_dot_sync[i] / abs(v_dir[i]))

    return min(s_dot_candidates)



# -----------------------------
# Combined safe speed
# -----------------------------

def constant_safe_speed(q0, q1, v_dir, tau_limits, max_acc, joint_vel_limits, N=200):
    # Torque-based limit
    s_values, s_max_values = sample_path_s_max(q0, q1, v_dir, tau_limits, max_acc, N=N)
    s_dot_torque_limit = np.min(s_max_values)

    # Tsync-based limit
    s_dot_sync = compute_s_dot_sync(q0, q1, v_dir, joint_vel_limits, max_acc)

    # Combined
    s_dot_safe = min(s_dot_torque_limit, s_dot_sync)

    v_safe = s_dot_safe * v_dir
    return s_dot_safe, s_values, s_max_values, v_safe, s_dot_sync, s_dot_torque_limit



# -----------------------------
# Run
# -----------------------------

if __name__ == "__main__":
    s_dot_safe, s_vals_const, s_max_vals, v_safe, s_dot_sync, s_dot_torque_limit = constant_safe_speed(
        q_actual, q_desired, v_dir, tau_limits, max_acc, joint_vel_limits, N=200
    )

    v_max_global = np.max(np.abs(v_safe))

    print("=== Constant safe speed along path ===")
    print(f"Safe path speed v_max_global: {v_max_global:.4f}")
    print(f"v_safe vector: {v_safe}")