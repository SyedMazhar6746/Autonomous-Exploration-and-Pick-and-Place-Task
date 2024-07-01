# Autonomous exploration and pick and place task


This repository contains the implementation of the autonomous task for region exploration and pick and place task.

It uses behavior trees to sequentially plan and execute the tasks. It consists of four modules: Planning, Localization, Perception and Intervention.

All the modules are implemented from scratch.

# Simulators

    Gazebo
    Stonefish

# Sensors

    Camera
    Wheel Encoder
    IMU


# Planning

    Exploration: Breadth-First search algorithm
    Global Path: RRT star
    Local Path : Dynamics Window Approach

# Localization

    Aruco Based EKF SLAM with Extrinsic Camera Calibration


# Intervention

    Task-Priority Based Redundancy Control of Robot Manipulators


# Videos: Please check the Videos folder for the real robot video demonstration.
