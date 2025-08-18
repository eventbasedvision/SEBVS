# Updates
<!-- syntax for updating -->
<!-- - [6 August 2025] Tool for Manual Annotation Release -->

# *SEBVS*: Synthetic Event-based Visual Servoing for Robot Navigation and Manipulation 

<!-- for adding link to paper and image -->
<div>
<a href="#">Paper</a> | 
<a href="https://drive.google.com/file/d/1tlwI928wwzoIaphzWcdPFxZYTYJ-hMQC/view?usp=sharing">Supplementary</a>
</div> 

<hr>

<div style="text-align: center;">
<img src="https://github.com/user-attachments/assets/d0923a73-4495-4e9b-b28a-f61efedc6c66"/>
</div>

<p align="justify">
Event cameras have emerged as a powerful sensing modality for robotics, offering microsecond latency, high dynamic range, and low power consumption. These characteristics make them well-suited for real-time robotic perception in scenarios affected by motion blur, occlusion, and extreme changes in illumination. Despite this potential, event-based vision—particularly through video-to-event (v2e) simulation—remains underutilized in mainstream robotics simulators, limiting the advancement of event-driven solutions for navigation and manipulation.

This work presents an open-source, user-friendly v2e robotics operating system (ROS 2) package for Gazebo simulation that enables seamless event stream generation from RGB camera feeds. The package is used to investigate event-based robotic policies (ERP) for real-time navigation and manipulation. Two representative scenarios are evaluated: (1) object following with a mobile robot and (2) object detection and grasping with a robotic manipulator. Transformer-based ERPs are trained by behavior cloning and compared to RGB-based counterparts under various operating conditions. Experimental results show that event-based policies consistently deliver competitive and often superior robustness in high-speed or visually challenging environments. These results highlight the potential of event-driven perception to improve real-time robotic navigation and manipulation, providing a foundation for broader integration of event cameras into robotic policy learning.
</p>

<!-- <div style="text-align: center;">
<img src="https://github.com/user-attachments/assets/82c93cc6-4f7d-4e35-b38f-5079b1b12ef3"/>
</div> -->

<!-- # Dataset Download
Dataset can be downloaded <a href="https://drive.google.com/drive/folders/1dwbeWHASKkLbLOImyHKE8of8hWCq7bdO?usp=drive_link">here</a> -->

# Package Overview
<p align="justify">
To enable event-camera simulation in Gazebo, a lightweight ROS 2 package was developed that integrates v2ecore’s EventEmulator with standard RGB camera topics. The emulator subscribes to the RGB image stream `/camera/image_raw`, performs resizing and grayscale conversion, and forwards the processed images to the EventEmulator. The generated event stream is published on the topic `/dvs/events`, which can subsequently be accumulated into event frames for downstream processing.
</p>

# Dataset 
ERPArm dataset is available <a href="https://www.dropbox.com/scl/fo/kzj9bw8gq81dc7hf0ff0m/AGCW_q92Vi1fRGIX-ODrlFA?rlkey=2edq60rlehlbjukocmov8kl6q&st=j5nj4sgd&dl=0">here</a>

ERPNav dataset is available <a href="https://www.dropbox.com/scl/fo/vtwkgit49jqnsafroyzmz/AEcEx-MVqFCMS-1Bi0EHQrI?rlkey=g0t6g5mnymc2wcpx98ypq7812&st=pn805sxx&dl=0">here</a>


### Tested Setup
- **OS:** Ubuntu 22.04  
- **ROS 2:** Humble (desktop-full)  
- **Python:** 3.10  
- **Gazebo:** Any ROS camera publishing `sensor_msgs/Image` works (Classic)


### Package Folder Structure
```bash
src/
└── ros2_v2e_emulator
    ├── setup.py
    └── ros2_v2e_emulator
        └── emulator_node.py
```

# Installation Guide

Make sure you have **ROS 2 Humble** and **Gazebo** installed.  
Install **PyTorch** directly in your ROS 2 Python environment (not inside a Conda environment).  

Next, install the [v2e](https://github.com/SensorsINI/v2e) packages by following the official instructions.  

Then clone and build this package:


```bash 
git clone https://github.com/eventbasedvision/SEBVS.git
cd SEBVS
colcon build
```

# Usage

To run the default emulator node: 

```bash 
source install/setup.bash
ros2 run ros2_v2e_emulator emulator_node
```



# License

<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.


# Citation
If you use our package, we appreciate a citation to the paper and to the original author of v2e. 
```bash
@inproceedings{hu2021v2e,
  title={v2e: From video frames to realistic DVS events},
  author={Hu, Yuhuang and Liu, Shih-Chii and Delbruck, Tobi},
  booktitle={Proceedings of the IEEE/CVF conference on computer vision and pattern recognition},
  pages={1312--1321},
  year={2021}
}
```
