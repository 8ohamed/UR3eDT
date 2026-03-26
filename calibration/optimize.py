import numpy as np
from scipy.optimize import least_squares
from scipy.interpolate import interp1d
from models.model import MotionModel

def compute_residuals(params, model, scenario, sim_dt=0.002):
    """Return the residual vector (model − observed), flattened.

    params: 1-D array [vel_scale(n), acc_scale(n)]
    model: MotionModel instance
    scenario: dict from load_scenario_csv
    sim_dt: simulation time-step [s]
    """
    n_joints = scenario["q_actual"].shape[1]
    vel_scale = params[:n_joints]
    acc_scale = params[n_joints:2 * n_joints]

    t_obs = scenario["timestamps"]
    q_obs = scenario["q_actual"]

    t_end = t_obs[-1] + sim_dt

    t_model, q_model, _, _ = model.simulate_joint_motion(
        q0=scenario["q0"],
        q_target=scenario["q_target"],
        max_velocity_deg=scenario["max_velocity_deg"],
        acceleration_deg=scenario["acceleration_deg"],
        dt=sim_dt,
        t_end=t_end,
        vel_scale=vel_scale,
        acc_scale=acc_scale,
        smooth_tau=float(params[2 * n_joints]),
    )

    # Interpolate model predictions onto observed time stamps
    q_interp = np.zeros_like(q_obs)
    for j in range(n_joints):
        f = interp1d(t_model, q_model[:, j], kind="linear",
                     fill_value="extrapolate")
        q_interp[:, j] = f(t_obs)

    return (q_interp - q_obs).flatten()


def compute_residuals_multi(params, model, scenarios, sim_dt=0.002):
    """Concatenate residuals from multiple scenarios into one vector."""
    parts = [compute_residuals(params, model, s, sim_dt) for s in scenarios]
    return np.concatenate(parts)


def calibrate(scenarios, initial_vel_scale=None, initial_acc_scale=None,
              initial_smooth_tau=None, sim_dt=0.002, bounds=(0.3, 2.0), max_nfev=500,
              n_starts=3, rng_seed=0, diff_step=1e-3):
    """Run least-squares calibration to find vel_scale and acc_scale.

    Args:
        n_starts: number of random restarts; the best result is kept.
                  The first start always uses the supplied initial guess;
                  subsequent starts perturb the current best solution.
        rng_seed: seed for the random restarts

    Returns a dict with:
        vel_scale, acc_scale: lists of calibrated per-joint factors
        cost: final sum-of-squared residuals
        nfev: total function evaluations across all starts
        success: bool
        message: solver message
    """
    # Accept a single scenario dict or a list of scenarios
    if isinstance(scenarios, dict):
        scenarios = [scenarios]

    n_joints = scenarios[0]["q_actual"].shape[1]

    if initial_vel_scale is None:
        initial_vel_scale = np.full(n_joints, 0.75)
    if initial_acc_scale is None:
        initial_acc_scale = np.full(n_joints, 0.75)
    if initial_smooth_tau is None:
        initial_smooth_tau = 0.05

    x0_base = np.concatenate([
        np.asarray(initial_vel_scale, dtype=float),
        np.asarray(initial_acc_scale, dtype=float),
        [float(initial_smooth_tau)],
    ])

    lo, hi = bounds
    lo_vec = np.concatenate([np.full(2 * n_joints, lo), [0.0]])
    hi_vec = np.concatenate([np.full(2 * n_joints, hi), [0.5]])
    # Clamp initial guess to bounds to handle warm-starts from previous runs
    x0_base = np.clip(x0_base, lo_vec, hi_vec)

    model = MotionModel()
    rng = np.random.default_rng(rng_seed)
    best_opt = None
    total_nfev = 0

    for start_i in range(max(1, n_starts)):
        if start_i == 0:
            x0 = x0_base.copy()
        else:
            # Perturb around the current best by ±10 % of the parameter range
            spread = 0.1 * (np.asarray(hi_vec) - np.asarray(lo_vec))
            x0 = np.clip(
                best_opt.x + rng.normal(0, spread),
                lo_vec, hi_vec
            )
            print(f"  [restart {start_i}] perturbed x0 around previous best")

        opt = least_squares(
            compute_residuals_multi,
            x0,
            args=(model, scenarios, sim_dt),
            bounds=(lo_vec, hi_vec),
            max_nfev=max_nfev,
            diff_step=diff_step,
            verbose=1,
        )
        total_nfev += opt.nfev

        if best_opt is None or opt.cost < best_opt.cost:
            best_opt = opt
            if n_starts > 1:
                print(f"  [restart {start_i}] new best cost={opt.cost:.4e}")

    opt = best_opt
    vel_scale = opt.x[:n_joints].tolist()
    acc_scale = opt.x[n_joints:2 * n_joints].tolist()
    smooth_tau = float(opt.x[2 * n_joints])

    return {
        "vel_scale": vel_scale,
        "acc_scale": acc_scale,
        "smooth_tau": smooth_tau,
        "cost": float(opt.cost),
        "nfev": total_nfev,
        "success": bool(opt.success),
        "message": str(opt.message),
    }


