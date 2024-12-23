# RoboMNIST
**RoboMNIST: A Multimodal Dataset for Multi-Robot Activity Recognition Using WiFi Sensing, Video, and Audio**

We introduce a novel dataset for multi-robot activity recognition (MRAR) using two robotic arms integrating WiFi channel state information (CSI), video, and audio data. This multimodal dataset utilizes signals of opportunity, leveraging existing WiFi infrastructure to provide detailed indoor environmental sensing without additional sensor deployment. Data were collected using two Franka Emika robotic arms, complemented by three cameras, three WiFi sniffers to collect CSI, and three microphones capturing distinct yet complementary audio data streams. The combination of CSI, visual, and auditory data can enhance robustness and accuracy in MRAR. This comprehensive dataset enables a holistic understanding of robotic environments, facilitating advanced autonomous operations that mimic human-like perception and interaction. By repurposing ubiquitous WiFi signals for environmental sensing, this dataset offers significant potential aiming to advance robotic perception and autonomous systems. It provides a valuable resource for developing sophisticated decision-making and adaptive capabilities in dynamic environments.

https://github.com/user-attachments/assets/b5266bee-0ef5-4132-9270-546de9581254

## Dataset
Please refer to [our paper TODO](http://TODO.com) for a detailed description of our dataset and where you can find the dataset files.

Our dataset is composed of $60$ different primary activity classes performed by the robotic arms, capturing activities through our sensor-rich modules. Our dataset encompasses four variations:

- **Action:** The Franka Emika robotic arms were programmed to draw the numbers $0$ through $9$ on a vertical imaginary plane, resulting in ten distinct classes of activities. We denote the actions performed by the robots as a ∈ {0, 1, ..., 9}.

- **Robot number:** Indicated by r ∈ {1, 2}, this specifies which of the two available robotic arms is performing the activity.

- **Robot velocity:** Denoted by v ∈ {*High*, *Medium*, *Low*}, this describes the velocity level at which the robot performs the action.

- **Motion uncertainty:** Denoted by $u \in \mathbb{R}^+$, where $\mathbb{R}^+$ represents the positive real numbers, measures the $L_2$ norm error of the end effector's position relative to its intended trajectory over time.

The combination of ten actions, two robots performing these actions, and three velocity levels results in a total of $60$ unique primary classes. For primary each class, we have collected $32$ repetitions. Each repetition spans $15$ seconds, during which the robot performs the activity with consistent variations in action, robot arm, and velocity, while incorporating motion uncertainty. This introduces deviations in each repetition as the robot writes on an imaginary plane, adding a realistic layer of complexity to the dataset.


<p align="center">
<img src="resources/mnist_like_plots.png" alt="mnist_like_plots"
title="mnist_like_plots" width="820" align="middle" />
</p>


## Examples
You can find the Python example notebooks in the [examples folder](examples) of this GitHub repository to help you get started to use the dataset.


## Collection Setup & Code
This section is helpful if you want to understand the details of the data collection process or reproduce the dataset. The diagram below shows the collection setup.

<p align="center">
<img src="resources/diagram.jpg" alt="diagram"
title="diagram" width="600" align="middle" />
</p>

The framework consists of three parts:

1. **Sensor Module**  
   This part of the framework reads data from the three sensors (CSI, Video, Audio) and sends it to the central monitor.

2. **Motion Controller**  
   This component handles robot control. It starts the movement when it receives a trigger from the central monitor.

3. **Synchronization & Central Monitor**  
   As the name suggests, this is the central monitoring system. It oversees the data collection process, ensures data synchronization, and is responsible for saving and storing the data.


### Sensor Module
Reading the sensor outputs from the camera and microphone is straightforward using OpenCV and TODO, respectively. For reading the CSI data, we used a Raspberry Pi 4 Model B integrated with the [Nexmon project](https://github.com/seemoo-lab/nexmon). Follow the instructions provided in the Nexmon project to set up your Raspberry Pi. To send the CSI information over the network, you need to configure the iptables (TODO: verify this) to forward the packets generated by Nexmon to the central monitor. Alternatively, we have provided a pre-configured Raspberry Pi image at [this link](TODO_Onedrive.com). You can clone this image onto your Raspberry Pi, where everything is ready to use. 
You can clone the image on to your Pi using the instructions in [this link](https://pbxbook.com/other/dd_clone.html) in a MacOS or other similar approaches for other operating systems.

Then, connect your Raspberry Pi to a router using an **ethernet cable** (the Raspberry will lose the WiFi capability when running the Nexmon project). then run the following commands to start the collection.

```bash
sudo bash setup.sh --laptop-ip <ip> --raspberry-ip <ip> --mac-adr <MAC> --channel <channel> --bandwidth <bandwidth> --core <core> --spatial-stream <spatial stream>
```

 - The `--laptop-ip` is the IP of the central monitor, this central monitor must be connected to the same router as the raspberry.
 - The `--raspberry-ip` is the IP of the Raspberry Pi.
 - The `--mac-adr` is the MAC address of the transmitter you want to filter.
 - `--channel`, `--bandwidth`, `--core`, and `--spatial-stream` are the CSI collection specifications (read more from Nexmon CSI project)

 ```bash
TODO command for audio
```


### Motion Controller
The code used to control the robot(s) is provided in the [motion_control folder](motion_control). The laptop running the controller must be connected to both the robot and the router that the sniffer and central monitor are connected to. 

You can find the specific configuration in the [config file](motion_control/config.json), which includes the IP address of the robot, the IP address of the central monitor, the speed level, and other settings. More details about these configurations are provided in the paper.

You can build and run the code using the following steps.

 ```bash
mkdir build
cd build
cmake .. # you need the Frankalib installed for C++
make
./main # run the compiled code
```


### Synchronization & Central Monitor (TODO check everythong)
We have used ROS2 for this part of the framework, the code can be found in the [monitor folder](monitor). Our ROS2 workspace is consist of the following nodes.

- **csi_node.** which is responsible to receive CSI information from the module(s), timestamp them, and store them at the end.

- **video_node.** which is responsible for receiving video frames from the module(s), timestamp them, and store them at the end.

- **audio_node.** which is responsible for receiving audio signals from the module(s), timestamp them, and store them at the end.

- **heartbeat_node.** This node serves as a syncronizer, where it sendes heartbeats to each node specifying the time that the nodes need to get the sensor data.

You can build the project using `colcon build` and then run each of the nodes using the following commands.

```bash
ros2 run pack_wifi csi_node (TODO)
```




## Our Paper
If you use this dataset for your academic research, please cite our paper.

```
@article{todo,
  title={},
  author={},
  journal={}, 
  year={},
  volume={}, 
  number={}, 
  pages={}
}
```


<!-- ## Acknowledgements
We use [nexmon project](https://github.com/seemoo-lab/nexmon_csi) for extracting WiFi CSI information in our modules. -->
