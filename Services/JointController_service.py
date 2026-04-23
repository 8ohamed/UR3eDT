# Pose Controller Service
#Service will read mockup configuration and use it as a initial configution to enhance the target configuration.

#1. Read $q_{actual}$ from rabbitMQ
#2. Calculate/Determine $q_{target}$
#3. Send $q_{target}$ via rabbitMQ to mockup

# PRE
# 1. start rabbitmq
# 2. start mockup

# Add the project root to Python's path
import sys
from pathlib import Path
project_root = str(Path(__file__).parent.parent)
if project_root not in sys.path:
    sys.path.append(project_root)

from typing import Any
import threading
from threading import Condition
import numpy
from communication.rabbitmq import Rabbitmq
from communication import protocol
from models.JointConfig import JointConfig
import time

class JointController():
    """
    loop:
        Read q_actual from rabbitMQ when robot is idle
        Determine q_target with presets
        Send q_target
    
    """
    
    def __init__(self):
        # Robot values
        self.robot_state = None 
        self.robot_status = None
        self.q_target = None
        self.jointConfig= JointConfig()
        # self.rmq = Rabbitmq(ip="localhost",port=5672,username="ur3e",password="ur3e",vhost="/",exchange="UR3E_AMQP",type="topic")
        self.condition_idle: Condition = threading.Condition()
        self.rmq_producer = Rabbitmq(ip="localhost",port=5672,username="ur3e",password="ur3e",vhost="/",exchange="UR3E_AMQP",type="topic")
        self.setup_complete = False
    
                   
    def update_robot_state(self, ch, method, properties, body):
        with self.condition_idle:
            self.robot_state = body
            # print("\n\n", body)

            if protocol.RobotArmStateKeys.ROBOT_MODE in body:
                self.robot_status: Any = body[protocol.RobotArmStateKeys.ROBOT_MODE]
                self.condition_idle.notify_all()  # Notify all waiting threads

            if protocol.RobotArmStateKeys.Q_ACTUAL in body:
                self.jointConfig.set_q_actual(q_actual=body[protocol.RobotArmStateKeys.Q_ACTUAL])   
     
    def consumer_thread(self):
        rmq = Rabbitmq(ip="localhost",port=5672,username="ur3e",password="ur3e",vhost="/",exchange="UR3E_AMQP",type="topic")
        # self.rmq_producer = rmq
        try:
            rmq.connect_to_server()
            print("Consumer\tConnected to RabbitMQ")
        except Exception as e:
            print(f"Failed to connect [{e}]")
            
        rmq.subscribe(routing_key=protocol.ROUTING_KEY_STATE, on_message_callback=self.update_robot_state)
        
        self.setup_complete = True
        try:
            rmq.start_consuming()
            print("Start consuming")

        except Exception as e:
            print(f"Could not consume [{e}]")
            # self.program()
            
                     
    def setup(self):
        try:
            self.rmq_producer.connect_to_server()
            print("Producer\tConnected to RabbitMQ")
        except Exception as e:
            print(f"Failed to connect [{e}]")
            
        # self.rmq_.subscribe(routing_key=protocol.ROUTING_KEY_STATE, on_message_callback=self.update_robot_state)
                        
    # def start(self):
    #     def run_consumer():
    #         try:
    #             self.rmq.start_consuming()
    #         except Exception as e:
    #             print(f"Could not consume [{e}]")
    #             self.setup()
    #             self.start()
    #             self.program()
        
    #     consumer_thread = threading.Thread(target=run_consumer, daemon=True)
    #     consumer_thread.start()
        
        
    def determine_config(self, preset):
        if self.robot_state is not None:
            self.q_target = self.jointConfig.generate_configuration(preset)
            
            return self.q_target
        
    def send_target_config(self):
        if self.q_target is None:
            print("error?")
        else:    
            msg= {
                protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.LOAD_PROGRAM,
                protocol.CtrlMsgKeys.JOINT_POSITIONS: self.q_target.q.tolist(),
                # protocol.CtrlMsgKeys.MAX_VELOCITY: 60,
                # protocol.CtrlMsgKeys.ACCELERATION: 60,
            }
            
            self.send_control_message(msg)
            
            msg_start = {
                protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.PLAY,
            }

            self.send_control_message(msg=msg_start)
        

    def send_control_message(self, msg):
        """Send a control message to the UR3e Mockup via RabbitMQ."""
        try:
            self.rmq_producer.send_message(
                routing_key=protocol.ROUTING_KEY_CTRL,
                message=msg
            )
            # print(f"✓ Control message: {msg} sent successfully")
        except Exception as e:
            print(f"✗ Failed to send control message: {e}")
            self.setup()
            self.send_control_message(msg)
            # self.program()

    def cont_movement(self):
        preset = 1
        while True:
            self.single_movement(preset)
            time.sleep(5)
            preset += 1
            if preset > 3:
                preset = 1

    def single_movement(self, preset):
        print("\n____________________________________\n")

        with self.condition_idle:
            while self.robot_status != protocol.RobotMode.ROBOT_MODE_IDLE:  # Wait for Idle
                self.condition_idle.wait()
            try:
                # print("Q_ACTUAL:\t",numpy.round(self.jointConfig.q_actual,5))                       
                res= self.determine_config(preset)
                if res is not None:
                    print("🟨 Q_TARGET:\t",numpy.round(res.q,5))
                    self.send_target_config()
            except Exception as e:
                print(f"Error: {e}")
                
            while self.robot_status == protocol.RobotMode.ROBOT_MODE_IDLE:
                self.condition_idle.wait()
                
            while self.robot_status != protocol.RobotMode.ROBOT_MODE_IDLE:
                self.condition_idle.wait()
            
            print("✅ Q_ACTUAL:\t", numpy.round(self.jointConfig.q_actual, 5))

    def program(self):
        print("\n____________________________________\n")

        print("Enter [r] to read state\n")
        while True:
            res = input("Run auto [a], Single [s]: ")
            match res:
                case "a":
                    jointController.cont_movement() # Automatic moving from [1 -> 2 -> 3 -> 1...]
                case "s":
                    while True:
                        print("\n\n*** PRESETS ***\n1: [-0.4, -0.35, 0.1]\n2: [0.4, -0.2, 0.1]\n3: [0.15, -0.2, 0.40]\n")
                        user_input = input("Enter target [1,2,3]: ").strip().lower()

                        jointController.single_movement(int(user_input)) # Manual moving based on input
                case "r":
                    while True:
                        print(numpy.round(jointController.jointConfig.q_actual,5))
                        time.sleep(1)
        
            
if __name__ == "__main__":
    jointController = JointController()
    jointController.setup()
    # jointController.start()
        # Start consuming data
    consumer_thread = threading.Thread(target=jointController.consumer_thread, daemon=True)
    consumer_thread.start()
    
    while jointController.setup_complete == False:
        time.sleep(1)
    
    jointController.program()
    
                
    
   