from roboticstoolbox.robot.DHLink import RevoluteDH
from roboticstoolbox.robot.DHRobot import DHRobot
from typing import Any
import roboticstoolbox as rtb
import numpy as np
from roboticstoolbox.robot import IKSolution

import time

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
        self.configuration: IKSolution
        self.q_actual= [0,0,0,0,0,0]
        self.acc = 0
        self.vel = 0
        
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
                    self.vel = 100
                case 2:
                    x = 0.4
                    y=-0.2
                    z=0.1
                    self.acc = 80
                    self.vel = 80
                case 3:
                    x=0.15
                    y=-0.2
                    z=0.40
                    self.acc = 70
                    self.vel = 40
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
        configuration = self.robot.ikine_LM(T,q0=self.q_actual)
                   
        if configuration.success == True:
            # Update configuration
            # print(f"\nConfiguration: \n\tFrom:\t{self.q_actual}\n\tTo:\t{configuration.q}")
            # self.configuration = configuration
            return configuration
        
        # return False???        
    
    def set_q_actual(self,q_actual):
        self.q_actual = q_actual
        
    def cont_movement(self):
        # Contineous movement (0 -> 3)
        preset = 1
        while True:
            res = self.generate_configuration(preset)
            print(res)
            # Update preset
            preset += 1
            if preset > 3:
                preset = 1
            time.sleep(5)
            
        
    def operation(self):     
        while True:
            print("\n\n*** PRESETS ***\n1: [-0.4, -0.35, 0.1]\n2: [0.4, -0.2, 0.1]\n3: [0.15, -0.2, 0.40]\n")
            user_input = input("Enter preset: ").strip().lower()
            
            if (1 <= int(user_input) and int(user_input) <= 3): 
                self.generate_configuration(user_input)
                user_input = ""

if __name__ == "__main__":
    jointConfig: JointConfig = JointConfig()
    jointConfig.cont_movement()
