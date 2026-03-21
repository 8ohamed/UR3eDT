
from calibration.utils import load_scenario_csv, save_parameters, load_parameters
from calibration.optimize import calibrate
import os

if __name__ == "__main__":

    scenario_paths = [
        "data/10_deg.csv",
        "data/30_deg.csv",
        "data/45_deg.csv",
        "data/60_deg.csv",
        "data/75_deg.csv",
        "data/90_deg.csv",

    ]
    output_path = "models/parameters.json"

    scenarios = []
    for path in scenario_paths:
        s = load_scenario_csv(path)
        print(f"  {path}: {len(s['timestamps'])} samples")
        scenarios.append(s)

    print("\nRunning calibration...")

    # Warm-start from existing parameters if available
    initial_vel_scale = None
    initial_acc_scale = None
    initial_smooth_tau = None
    if os.path.exists(output_path):
        prev = load_parameters(output_path)
        initial_vel_scale = prev.get("vel_scale")
        initial_acc_scale = prev.get("acc_scale")
        initial_smooth_tau = prev.get("smooth_tau")
        print(f"  warm-starting from existing: {output_path}")

    result = calibrate(scenarios,
        initial_vel_scale=initial_vel_scale,
        initial_acc_scale=initial_acc_scale,
        initial_smooth_tau=initial_smooth_tau,
    )





    status = "SUCCEEDED" if result["success"] else "FAILED"
    print(f"\nCalibration {status}:")
    print(f"  cost     : {result['cost']:.6e}")
    print(f"  nfev     : {result['nfev']}")
    print(f"\nSaving parameters to: {output_path}")
    save_parameters(output_path, result)


