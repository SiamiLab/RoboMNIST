# RoboMNIST
**Leveraging Signals of Opportunity: Enhancing Activity Recognition through Multimodal Sensing with WiFi CSI, Video, and Sound**

We introduce a novel dataset for robot activity recognition (RAR) integrating WiFi Channel State Information (CSI), vision, and audio data. This multimodal dataset utilizes signals of opportunity, leveraging existing WiFi infrastructure to provide detailed environmental sensing without additional sensor deployment. Data was collected using two Franka Emika robotic arms, with three cameras, three WiFi sniffers to collect CSI, and three microphones capturing distinct yet complementary data streams. The combination of CSI, visual, and auditory data enhances robustness and accuracy in activity recognition. This comprehensive dataset enables a holistic understanding of robotic environments, facilitating advanced autonomous operations that mimic human-like perception and interaction. By repurposing ubiquitous WiFi signals for environmental sensing, this dataset offers significant reuse potential for researchers aiming to advance robotic perception and autonomous systems, providing a valuable resource for developing sophisticated decision-making and adaptive capabilities in dynamic environments.


## Dataset

Please refer to [our paper TODO](http://TODO.com) for a detailed description of our dataset and where you can find the dataset files.

## Examples
You can find the Python example notebooks in the [examples folder](examples) of this GitHub repository to help you get started to use the dataset.


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


## Acknowledgements
We use [nexmon project](https://github.com/seemoo-lab/nexmon_csi) for extracting WiFi CSI information in our modules.