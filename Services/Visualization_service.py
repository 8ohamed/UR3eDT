from pathlib import Path
import sys
project_root = str(Path(__file__).parent.parent)
if project_root not in sys.path:
    sys.path.append(project_root)
    
from roboticstoolbox import DHRobot, IKSolution
from roboticstoolbox.backends import PyPlot as PyPLOT
import roboticstoolbox as rtb
from communication import protocol
from communication.rabbitmq import Rabbitmq
from models.JointConfig import JointConfig
import threading
import time

class Visualization():
    def __init__(self, jointConfigOBJ) -> None:
        self.links = jointConfigOBJ.get_links()
        self.robot: DHRobot = rtb.DHRobot(self.links, name="UR3e_Visualize")
        self.robot_q_actual = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.rmq = Rabbitmq(ip="localhost", port=5672,username="ur3e",password="ur3e",vhost="/",exchange="UR3E_AMQP",type="topic")

    def visualize(self):
        backend = PyPLOT.PyPlot()
        backend.launch(name="UR3e",limits=[-0.5,0.5,-0.5,0.5,0,0.5])
        backend.add(self.robot)
        
        while True:
            self.robot.q = self.robot_q_actual
            backend.step()
            # self.robot.plot(self.robot_q_actual,backend="pyplot")            
            # time.sleep(0.1)

    def update_robot_state(self,ch, method, properties, body):
        if protocol.RobotArmStateKeys.Q_ACTUAL in body:
            self.robot_q_actual = body[protocol.RobotArmStateKeys.Q_ACTUAL]
            
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
            
if __name__ == "__main__":
    jointConfig = JointConfig()
    visualization = Visualization(jointConfigOBJ=jointConfig)
    visualization.setup()
    visualization.start()
    
    test= input("WAITING FOR ENTER")
    while True:
        visualization.visualize()