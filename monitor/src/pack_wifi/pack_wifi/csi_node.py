import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

import socket
import signal
import sys
from threading import Thread, Lock
import time
from copy import deepcopy
from multiprocessing import Process
import pickle
import os
import subprocess


def speak(text):
    speed = 250
    subprocess.Popen(["espeak", "-s", str(speed), text])

mutex = Lock()
last_packets = {} # key: sniffer ip, value: (time, data)
stop_csi_thread = False # terminate condition for the thread
class ThreadCSI(Thread):
    def __init__(self):
        super().__init__()
        
        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.client.bind(("", 5500))
    
    
    def run(self):
        while True:
            if stop_csi_thread:
                break
            data, addr = self.client.recvfrom(1042)
            with mutex:
                if not addr[0] in last_packets.keys():
                    last_packets[addr[0]] = []
                last_packets[addr[0]].append((time.time_ns(), data))


class CSINode(Node):

    def __init__(self):
        super().__init__('csi_node',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)
        
        # capture ctrl+c
        signal.signal(signal.SIGINT, self.terminate)
        
        # finding workspace path to save the data files in it
        self.ws_path = os.path.dirname(os.path.abspath(__file__))
        idx = self.ws_path.find("WiFi") + len("WiFi")
        self.ws_path = self.ws_path[0:idx]
        
        # ros2 param handling
        self.set_parameters_callback(self.parameter_callback)
        self.frequency = self.get_parameter('frequency').value   
        
        # done publisher
        self.finished_publisher = self.create_publisher(String, '/finished', 10)     
        
        self.heartbeat_subscription = self.create_subscription(String, '/heartbeat', self.heartbeat_callback, 10)
        
        self.collector = {}
        self.thread_csi = ThreadCSI()
        self.thread_csi.start()
        

    def heartbeat_callback(self, msg):
        with mutex:
            last_packets_tmp = deepcopy(last_packets)

        packet_num, stat = msg.data.split('-')
        for ip, lasts in last_packets_tmp.items():
            if not ip in self.collector.keys():
                self.collector[ip] = {}
            diff = int(packet_num) - len(self.collector[ip])
            to_add = self.get_last_n_elements(lasts, diff)
            for (t, data) in to_add:
                self.collector[ip][t] = data
        with mutex: # clear last packets
            for ip in last_packets.keys():
                last_packets[ip] = []
        
        if stat != "resume":
            print(f"number of collectors {len(self.collector)}")
            for key in self.collector.keys():
                print(f" - number of samples from {key} is {len(self.collector[key])}")
            print("writing data to file...")
            filename = stat + '_csi.dat'
            filepath = os.path.join(self.ws_path, filename)
            pickle_out = open(filepath, 'wb')
            pickle.dump(self.collector, pickle_out)
            pickle_out.close()
            print(f"DONE. Saved to {filename}")
            self.collector.clear()
            
            # publishing done message
            msg = String()
            msg.data = self.get_name()
            self.finished_publisher.publish(msg)
    
    @staticmethod  
    def get_last_n_elements(lst, n):
        if len(lst) >= n:
            return lst[-n:]
        return lst[-len(lst):]
        
        
    def terminate(self, sig, frame):
        global stop_csi_thread
        stop_csi_thread = True
        print("exiting...")
        sys.exit()
           
    # calback for the parameters
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'frequency':
                self.frequency = param.value
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CSINode()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()