from roboticstoolbox.robot.IK import IKSolution


from numpy.typing import ArrayLike
from typing import List
from roboticstoolbox.robot.DHLink import RevoluteDH
from roboticstoolbox.robot.DHRobot import DHRobot
import roboticstoolbox as rtb
import numpy as np
from roboticstoolbox.robot import IKSolution

class JointConfig:
    
    def __init__(self):
        # Joint parameters
        self.links: list[RevoluteDH] = [
            rtb.RevoluteDH(d = 0.15185, a = 0,        alpha = np.pi/2),
            rtb.RevoluteDH(d = 0,       a = -0.2435, alpha = 0),
            rtb.RevoluteDH(d = 0,       a = -0.2132,  alpha = 0),
            rtb.RevoluteDH(d = 0.13105, a = 0,        alpha = np.pi/2),
            rtb.RevoluteDH(d = 0.08535, a = 0,        alpha = -np.pi/2),
            rtb.RevoluteDH(d = 0.0921,  a = 0,        alpha = 0)
        ]
        self.q_actual = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.acc = 0
        self.vel = 0
        self.mask = [1, 1, 1, 1, 1, 1]
        
        # Create the robot object
        self.robot: DHRobot = rtb.DHRobot(self.links, name="UR3e")
                                
    # Generate a joint configuration with xyz
    def generate_configuration(self,preset = None, x = None, y = None, z = None, v_max=1, alpha_pt=1):
        if preset is not None:
            match preset:
                case 1:
                    x=-0.4
                    y=-0.35
                    z=0.1
                    self.acc = 100
                    self.vel = [100,100,20,20,40,40]
                case 2:
                    x = 0.4
                    y=-0.2
                    z=0.1
                    self.acc = 80
                    self.vel = [100,100,20,20,40,40]
                case 3:
                    x=0.15
                    y=-0.2
                    z=0.40
                    self.acc = 40
                    self.vel = [80,90,60,10,20,40]
                case _:
                    print("Preset not available")
                    return None
        
        # Verify inputs manuel inputs
        if x is None or y is None or z is None:
            print("Input coordinates must be provided")
            return None
        
        if not (-0.5 < x and x < 0.5):
            print("x is outside range acceptable range [-0.5, 0.5]")
            return None
        if not (-0.5 < y and y < 0.5):
            print("y is outside range acceptable range [-0.5, 0.5]")
            return None
        if not (-0.5 < z and z < 0.5):
            print("z is outside range acceptable range [-0.5, 0.5]")
            return None
        
        # Rotation matrix
        R = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
                
        # Setup matrix
        T = np.eye(4)
        T[:3,:3] = R
        T[:3,3] = [x,y,z]

        # Calculate joint configuration
        # configuration: IKSolution = self.robot.ikine_LM(T,q0=self.q_actual,joint_limits=True,tol=1e-3)
        configuration: IKSolution = self.robot.ikine_LM(T,q0=self.q_actual,joint_limits=True)
        return configuration
        
    def set_link_constraint(self,joint: int):
        range = 0.1
        self.links[joint].qlim = [self.q_actual[joint]-range, self.q_actual[joint]+range]
        # print(f"Set link[{joint}] constraint {self.q_actual[joint]*180/np.pi}")
        self.robot = rtb.DHRobot(self.links) # Update settings
        # print(self.robot)

    def reset_link_constraint(self,joint: int):
        self.links[joint].qlim = [-np.pi, np.pi]
        self.robot = rtb.DHRobot(self.links) # Update settings
    
    def set_q_actual(self,q_actual: ArrayLike):
        self.q_actual = q_actual
        # print(f"Updated robot q_acutal: {q_actual}")
        
    def get_links(self):
        return self.links
        
