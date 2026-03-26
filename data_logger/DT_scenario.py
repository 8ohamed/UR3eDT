"""THis module runs a scenario on the model,
collects the data and saves it to a file for later use in analysis."""


import sys, os
root = os.path.abspath(os.path.join(os.path.dirname(__file__) if '__file__' in dir() else os.getcwd(), '..'))
sys.path.insert(0, root)
import numpy as np
import matplotlib.pyplot as plt
from models.model import MotionModel
from calibration.utils import load_parameters
model = MotionModel()


def create_scenario(q0, q_target, max_vel, acceleration, dt, t_end, scenario_name):
    """Simulate a scenario with the given parameters and save the logged data."""
    parameters = load_parameters(os.path.join(root, "models/parameters.json"))
    alpha_v = parameters["vel_scale"]
    alpha_a = parameters["acc_scale"]

    t, qlog, qdlog, qddlog = model.simulate_joint_motion(q0, q_target, max_vel, acceleration, dt, t_end, alpha_v, alpha_a)
    try:
        os.makedirs("./data", exist_ok=True)
        file_path = os.path.join("./data", scenario_name + ".csv")

        with open(file_path, 'w') as f:
            f.write("timestamp,actual_joint_position,\
                    target_joint_position,\
                    max_joint_speed,joint_acceleration,\
                    velocity, acceleration\n")

            for i in range(len(t)):
                row = [
                    str(t[i]),
                    str(qlog[i]),
                    str(q_target),
                    str(max_vel),
                    str(acceleration),
                    str(qdlog[i]),
                    str(qddlog[i])
                ]
                f.write(",".join(row) + "\n")

        print(f"✓ Data saved successfully to {file_path}")

    except Exception as e:
        print(f"✗ Failed to save data to file: {e}")


def main():
    create_scenario(
    q0=[0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
    q_target=[np.pi/2, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2],
    max_vel=60, acceleration=30, dt=0.01, t_end=5, scenario_name="test_scenario")


if __name__ == "__main__":    main()
