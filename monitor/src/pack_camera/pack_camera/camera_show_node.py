import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

import signal
import sys
import os
import subprocess
import cv2


def speak(text):
    speed = 250
    subprocess.Popen(["espeak", "-s", str(speed), text])


class CameraShowNode(Node):
    def __init__(self):
        super().__init__('camera_show_node',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)
        
        # capture ctrl+c
        signal.signal(signal.SIGINT, self.terminate)
        
        # ros2 param handling
        self.set_parameters_callback(self.parameter_callback)
        self.frequency = self.get_parameter('frequency').value 
        self.camera_num = self.get_parameter('camera_num').value        
        self.resolution0 = self.get_parameter('resolution0').value        
        self.resolution1 = self.get_parameter('resolution1').value 
        
        # finding workspace path to save the data files in it
        self.ws_path = os.path.dirname(os.path.abspath(__file__))
        idx = self.ws_path.find("WiFi") + len("WiFi")
        self.ws_path = self.ws_path[0:idx]
        
                
        # connecting to cameras
        self.cams = []
        for i in range(self.camera_num * 3): # camera indices is not always in sequence (trying 3*number of cameras indices to find all of them)
            cap = cv2.VideoCapture(i)
            if not cap.isOpened():
                continue
            self.cams.append(cap)
            self.cams[-1].set(cv2.CAP_PROP_FPS, self.frequency)
            self.cams[-1].set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution0)
            self.cams[-1].set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution1)
            print("successfull")
            if len(self.cams) == self.camera_num:
                print("camera setup finished")
                break
        if len(self.cams) != self.camera_num:
            print("COULD NOT FIND ALL THE REQUESTED CAMERAS.")
            self.terminate()
        
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writers = []
        for i in range(self.camera_num):
            filepath = os.path.join(self.ws_path, f"{i}.mp4")
            self.writers.append(cv2.VideoWriter(filepath, self.fourcc, self.frequency, [self.resolution0, self.resolution1]))

        
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)
        

    def timer_callback(self):        
        for i in range(self.camera_num):
            ret, frame = self.cams[i].read()
            if not ret:
                speak("SKIPPED")
                return
            cv2.imshow(f'frame {i}', frame)
            self.writers[i].write(frame)
            cv2.waitKey(1)
            
    
    def terminate(self, sig, frame):
        print("exiting...")
        for i in range(self.camera_num):
            self.writers[i].release()
        sys.exit()
           
    # calback for the parameters
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'camera_num':
                self.camera_num = param.value
                print(f"camera number changed to {self.camera_num}, PLEASE CHANGE IT FROM THE START OF THE COLLECTION")
            elif param.name == 'resolution0':
                self.resolution0 = param.value
                print(f"resolution0 changed to {self.resolution0}, PLEASE CHANGE IT FROM THE START OF THE COLLECTION")
            elif param.name == 'resolution1':
                self.resolution1 = param.value
                print(f"resolution1 changed to {self.resolution1}, PLEASE CHANGE IT FROM THE START OF THE COLLECTION")
            elif param.name == 'frequency':
                self.frequency = param.value
                self.timer.cancel()
                self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)
                print(f"frequency changed to {self.frequency}")
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CameraShowNode()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()