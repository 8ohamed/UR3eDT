import time
from roboticstoolbox.robot.IK import IKSolution


from pathlib import Path
import sys

from numpy.typing import ArrayLike
from roboticstoolbox import IKSolution
project_root = str(Path(__file__).parent.parent)
if project_root not in sys.path:
    sys.path.append(project_root)

import threading
from threading import Condition

import numpy as np

from communication import protocol
from communication.rabbitmq import Rabbitmq
from models.JointConfig import JointConfig

class JointConstraint():
    
    def __init__(self, jointConfigOBV):
        self.robot_config: ArrayLike
        self.is_idle = False
        self.joint_Config: JointConfig = jointConfigOBV
        self.current_config = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.rmq = Rabbitmq(ip="localhost",port=5672,username="ur3e",password="ur3e",vhost="/",exchange="UR3E_AMQP",type="topic")
        self.idle_condition: Condition = Condition()

    def test_configuration(self,preset):
        """
        Steps:
        1. Read current configuration and only save if mode is idle
        2. If configuration is not None -> Try presets
        3. Evaluate
        4. Set configuration to None
        """
        # READ CURRENT CONFIGURATION
        with self.idle_condition:
                self.idle_condition.wait(timeout=5.0)
        
        self.joint_Config.set_q_actual(q_actual=self.current_config)
        print(f"Current config: {np.round(self.current_config,5)}")    
        for j in range(0,6):
        # for j in range(0,1):
            self.joint_Config.set_link_constraint(joint=j)
            solution: IKSolution = self.joint_Config.generate_configuration(preset) 
            
            if solution.success == True:
                print(f"Joint[{j+1}] LOCKED \tFound possible configuration\t✅")
            else:
                print(f"Joint[{j+1}] LOCKED \tNo possible configuration\t❌")
            # print(solution)
            self.joint_Config.reset_link_constraint(joint=j)     
                        
    def update_robot_state(self, ch, method, properties, body):
        if body[protocol.RobotArmStateKeys.ROBOT_MODE] == "Idle":
            if protocol.RobotArmStateKeys.Q_ACTUAL in body:
                with self.idle_condition:
                    self.current_config = body[protocol.RobotArmStateKeys.Q_ACTUAL]
                    self.is_idle = True
                    self.idle_condition.notify_all()
                    
    def setup(self):
        try:
            self.rmq.connect_to_server()
            print("Connected to RabbitMQ")
        except Exception as e:
            print(f"Failed to connect [{e}]")
            
        self.rmq.subscribe(routing_key=protocol.ROUTING_KEY_STATE, on_message_callback=self.update_robot_state)
    
    def start(self):
        def run_consumer():
            try:
                self.rmq.start_consuming()
            except Exception as e:
                print(f"Could not consume [{e}]")
                self.setup()
                self.start()
        
        consumer_thread = threading.Thread(target=run_consumer, daemon=True)
        consumer_thread.start()
            
# Connect via rabbitMQ to seperate and run as an independant service
if __name__ == "__main__":
    jointConfig = JointConfig()
    jointConstraint = JointConstraint(jointConfig)
    jointConstraint.setup()
    jointConstraint.start()
    joint = ""
    preset = ""
    print("Enter 0 to read state\n\n")
    while True:
        print("\n____________________________________\n")
        preset = input("Enter which preset to test [1,2,3]: ")
        print(f"\n###\tTESTING PRESET {preset}\t###\n")
        if preset == "0":
            while True:
                print(np.round(jointConstraint.current_config,5))
                time.sleep(1)
        else:
            jointConstraint.test_configuration(preset=int(preset))
                

   