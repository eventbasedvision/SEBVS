# Updates
<!-- syntax for updating  -->
<!-- - [6 August 2025] Tool for Manual Annotation Release -->

# *SEBVS*: Synthetic Event-based Visual Servoing for Robot Navigation and Manipulation 


<!-- for adding link to paper and image -->

<!-- <div>
<a href="#"> Paper</a> |
<a href="https://interact-videoqa.github.io/InterActVideoQA/">Website</a> |
<a href="https://huggingface.co/datasets/joeWabbit/InterAct_Video_Reasoning_Rich_Video_QA_for_Urban_Traffic/blob/main/README.md">Data</a> |
<a href="https://interact-videoqa.github.io/InterActVideoQA/docs/InterAct_VideoQADatasetDescription.pdf" target="_blank">Doc </a> |
<a href="https://github.com/joe-rabbit/interact_videoqa/tree/main/Video_Annotation" target="_blank"> Anotation Tool </a>
</div> -->
<hr>
<div style="text-align: center;">
<img src="https://github.com/user-attachments/assets/d0923a73-4495-4e9b-b28a-f61efedc6c66"/>
</div>

<p align="justify">Event cameras have emerged as a powerful sensing modality for robotics, offering microsecond latency, high dynamic range, and low power consumption. These characteristics make them well-suited for real-time robotic perception in scenarios affected by motion blur, occlusion, and extreme changes in illumination. Despite this potential event-based vision, particularly through video-to-event (v2e) simulation,remains underutilized in mainstream robotics simulators, limiting the advancement of event-driven solutions for navigation and manipulation. This work presents an open-source, user-friendly v2e robotics operating system (ROS2) package for Gazebo simulation that enables seamless event stream generation from RGB camera feeds. The package is used to investigate event-based robotic policies (ERP) for real-time navigation and manipulation. Two representative scenarios are evaluated: (1) object following with a mobile robot and (2) object detection and grasping with a robotic manipulator. Transformer-based ERPs are trained by behavior cloning and compared to RGB-based counterparts under various operating conditions. Experimental results show that event-based policies
consistently deliver competitive and often superior robustness in high-speed or visually challenging environments. These results highlight the potential of event-driven perception to improve real-time robotic navigation and manipulation, providing a foundation for broader integration of event cameras into robotic policy learning..
</p>

# Related Works



<!-- <div style="text-align: center;">
<img src="https://github.com/user-attachments/assets/82c93cc6-4f7d-4e35-b38f-5079b1b12ef3"/>
</div> -->


<!-- # Dataset Download
Dataset can be downloaded <a href="https://drive.google.com/drive/folders/1dwbeWHASKkLbLOImyHKE8of8hWCq7bdO?usp=drive_link">here</a> -->


# Package Overview
<p align="justify">
To enable event-camera simulation in Gazebo, a lightweight ROS2 package was developed that integrates v2ecore’s EventEmulator with standard RGB camera topics. The emulator subscribes to the RGB image stream /camera/image_raw, performs resizing and grayscale conversion, and forwards the processed images to the EventEmulator. The generated event stream is published on the topic /dvs/events, which can subsequently be accumulated into event frames for downstream processing.

</p>

# Installation Guide

# Folder Structure
```
data
├── videoannotations.csv
└── Videos
    ├── clip_videos_0.mp4
    ├── clip_videos_1.mp4
    ├── clip_videos_2.mp4
    └── ...

```
<!-- # Model Setup 
Please look at the official github page for the models to set up.
- [VideoLLama2](https://github.com/DAMO-NLP-SG/VideoLLaMA2)
- [LlavaNext-Video](https://github.com/LLaVA-VL/LLaVA-NeXT)  
- [Qwen2-VL-7B-hf](https://github.com/QwenLM/Qwen2.5-VL)
# Baseline

<div style="text-align: center;">
    <img src="https://github.com/user-attachments/assets/264443ff-05c6-49d2-9d5c-60a0789b6b2d" alt="Image">
</div> -->

<!-- The main implementations for these models for InterAct VideoQA can be found here. -->

<!-- [VideoLLama2](https://github.com/joe-rabbit/interact_videoqa/tree/main/interAct%20VideoQA/VideoLlama2) |
[LlavaNext-Video](https://github.com/joe-rabbit/interact_videoqa/tree/main/interAct%20VideoQA/Llava-Next-Video) | 
[Qwen2-VL-7B-hf](https://github.com/joe-rabbit/interact_videoqa/tree/main/interAct%20VideoQA/Qwen-VL2-7B-hf) -->

# License

<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.


# Citation
If you use our package, we appreciate a citation to the paper and to the original author of v2e. 
```
@inproceedings{hu2021v2e,
  title={v2e: From video frames to realistic DVS events},
  author={Hu, Yuhuang and Liu, Shih-Chii and Delbruck, Tobi},
  booktitle={Proceedings of the IEEE/CVF conference on computer vision and pattern recognition},
  pages={1312--1321},
  year={2021}
}

  ```


