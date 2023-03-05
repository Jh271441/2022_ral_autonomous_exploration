# Autonomous Exploration
- 论文：[Autonomous Exploration in a Cluttered Environment for 
a Mobile Robot With 2D-Map Segmentation and Object Detection](https://ieeexplore.ieee.org/document/9765335/)
- 源代码：https://github.com/KimHyung/autonomous_exploration
- 运行：
```	
    //1. Launch Gazebo Simulation
    roslaunch autonomous_exploration maze.launch
    //2. Launch SLAM & Rviz
    roslaunch autonomous_exploration gmapping.launch

    //3-1. Luanch autonomus exploration without yolo
    roslaunch autonomous_exploration autonomous_exploration.launch
    //3-2. Luanch autonomus exploration with yolo
    roslaunch autonomous_exploration yolo.launch
    
    *To launch autonomous_exploration with yolo, you have to set up your environment(nvidia driver, CUDA, cudnn) for YOLO(ref. https://github.com/leggedrobotics/darknet_ros)
``````
- 复现日期：2023.2.28-2023.3.5