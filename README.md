# Maze Escape Robot – BPC-PRP

This project implements control of an autonomous mobile robot for the final *Maze Escape* task (and also previous tasks - *Line Following* and *Corridor Following*) in the BPC-PRP (Practical Robotics and Computer Vision) course at Brno University of Technology.

The robot is built on a ROS2-based architecture and combines multiple sensing modalities including:

- LiDAR corridor navigation
- IMU-based orientation estimation
- Line following using PID control
- Wheel encoder odometry
- Camera-based ArUco marker detection

The software is divided into modular ROS2 nodes responsible for sensing, filtering, control, and decision making. The robot uses a finite state machine to navigate through the maze, detect intersections, execute turns, and react to visual navigation hints.

## Features

- Stable line following with PID regulation
- Corridor centering using LiDAR distance balancing
- IMU-assisted turning and orientation tracking
- ArUco marker recognition with OpenCV
- Modular ROS2 node architecture
- Real-time autonomous maze navigation

## Technologies

- C++
- ROS2
- OpenCV
- LiDAR
- IMU
- PID control

## Course Resources

- Course documentation: https://robotics-but.github.io/BPC-PRP
- Project template: https://github.com/Robotics-BUT/BPC-PRP
