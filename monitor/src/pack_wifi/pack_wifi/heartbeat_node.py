import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

import signal
from datetime import datetime
import sys
import time
import threading
from multiprocessing import Process
import subprocess

def speak(text):
    speed = 250
    subprocess.Popen(["espeak", "-s", str(speed), text])
    
        
class HeartBeatNode(Node):

    def __init__(self):
        super().__init__('heartbeat_node',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)
        
        # capture ctrl+c
        signal.signal(signal.SIGINT, self.terminate)
        
        # ros2 param handling
        self.set_parameters_callback(self.parameter_callback)
        self.frequency = self.get_parameter('frequency').value        
        self.packet_num = self.get_parameter('packet_num').value        
        self.between_collection_delay = self.get_parameter('between_collection_delay').value        
        self.collectors = self.get_parameter('collectors').value
        
        self.heartbeat_publisher = self.create_publisher(String, '/heartbeat', 10)
        self.heartbeat_msg = String()
        self.heartbeat_msg.data = "" # empty for heartbeat and "filename" for ending
        
        # each collector should announce when its done working after each collection period
        self.collectors_done = {collector: False for collector in self.collectors}
        self.done_topic = self.create_subscription(String, '/finished', self.handle_done_service, 10)
        
        
        self.packet_counter = 0
        self.prev_second = -1
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)
    


    def timer_callback(self):
        self.packet_counter += 1
        if self.packet_counter < self.packet_num:
            self.heartbeat_msg.data = f"{self.packet_counter}-resume"
            self.heartbeat_publisher.publish(self.heartbeat_msg)
        else:
            filename = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
            self.heartbeat_msg.data = f"{self.packet_counter}-{filename}"
            self.heartbeat_publisher.publish(self.heartbeat_msg)
            
            # waiting for all collectors to be ready again for the next round of collection - handle_done_service will restart the timer
            self.timer.cancel()                
            
        current_second = int(self.packet_counter*(1.0/self.frequency))
        if current_second > self.prev_second:
            print(str(current_second))
            speak(str(current_second))
        self.prev_second = current_second
    
    
    def handle_done_service(self, msg):
        node_name = msg.data
        if node_name not in self.collectors_done.keys():
            print(f"INCORRECT NODE NAME!! {node_name} is wrong")
        self.collectors_done[node_name] = True
        # restart the timer if all collectors are done
        if all(self.collectors_done.values()):
            self.collectors_done = {collector: False for collector in self.collectors}
            self.packet_counter = 0
            self.prev_second = -1
            self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)
    
    
    def terminate(self, sig, frame):
        self.timer.cancel()
        filename = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
        self.heartbeat_msg.data = f"{self.packet_counter}-{filename}"
        # self.heartbeat_publisher.publish(self.heartbeat_msg)
        print("exiting... filename:", filename)
        sys.exit()
        
    
    # calback for the parameters
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'frequency':
                self.frequency = param.value
                self.timer.cancel()
                self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)
                print(f"heartbeat restart with {self.frequency} frequency")
            elif param.name == 'packet_num':
                self.packet_num = param.value
                print(f"packet number changed to {self.packet_num}")
            elif param.name == 'between_collection_delay':
                self.between_collection_delay = param.value
                print(f"between collection delay changed to {self.between_collection_delay}")
            elif param.name == 'collectors':
                self.collectors = param.value
                print(f"collectors changed to {self.collectors}")
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = HeartBeatNode()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()