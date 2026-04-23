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

    def update_robot_state(self,ch, method, properties, body):
        if protocol.RobotArmStateKeys.Q_ACTUAL in body:
            self.robot_q_actual = body[protocol.RobotArmStateKeys.Q_ACTUAL]
      
    def consumer_thread(self):
        rmq = Rabbitmq(ip="localhost",port=5672,username="ur3e",password="ur3e",vhost="/",exchange="UR3E_AMQP",type="topic")
        try:
            rmq.connect_to_server()
            print("Consumer\tConnected to RabbitMQ")
        except Exception as e:
            print(f"Failed to connect [{e}]")
            
        rmq.subscribe(routing_key=protocol.ROUTING_KEY_STATE, on_message_callback=self.update_robot_state)
        
        try:
            rmq.start_consuming()
            print("Start consuming")

        except Exception as e:
            print(f"Could not consume [{e}]")
            

if __name__ == "__main__":
    jointConfig = JointConfig()
    visualization = Visualization(jointConfigOBJ=jointConfig)
    
    consumer_thread = threading.Thread(target=visualization.consumer_thread, daemon=True)
    consumer_thread.start()

    while True:
        visualization.visualize()