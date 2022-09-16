# Autonomous Driving Pipeline in ROS

Open Source Autonomous Driving Pipeline implemented in ROS. It is part of EPFL [VITA](https://www.epfl.ch/labs/vita/) Lab project

<p float="left">
  <img src="./img/ADP.gif" width="400" />
  <img src="./src/control/Images/MR_Loomo_closed_loop.gif" width="400" /> 
</p>

## Description

We propose a modular pipeline for autonomous systems, which combines Computer Vision algorithms leveraging Deep Learning with classic robust methods for Path Planning, State Estimation and Control tasks. Our structure is designed to be general to adapt to different types of mobile robots and also flexible to easily change algorithms in its various independent modules. 

We use [Robot Operating System (ROS)](https://www.ros.org/) and network socket to establish wireless connection between the Server (Computer running the pipeline) and the Client (Mobile Robot).

For our implementation we use a [Loomo Segway Robot](https://store.segway.com/segway-loomo-mini-transporter-robot-sidekick).

Finally in our case we specifically adapt this general Autonomous Driving framework to fit the needs of neuroscience research area by precisely estimating the 3D Pose of the target, helping paraplegic patients to walk using our Loomo mobile robot as an assistant.

## User Guide

Before using the pipeline be sure to have completed the [🛠️ Install](./Install.md) process.


### Quick Start

In the root folder, ROS request that we type the following command everytime before launching any package.

```shell
source devel/setup.bash
```

If we need to change some parameters of the ROS structure, we can go to [Loomo.launch](/src/loomo) inside the launch folder.

The Autonomous System used in our work is a [Loomo](https://store.segway.com/segway-loomo-mini-transporter-robot-sidekick) Segway Robot. We need beforehand to install a specific Android app to enable the connection between the client (Loomo) and the server (ROS pipeline). The app source code and package can be found [here](https://github.com/theoh-io/Loomo_app_ADP).

Once everything is set up and adjusted to our specific context, we are ready to launch:

```shell
roslaunch loomo Loomo.launch
```


### Structure

We present all the different pillars of the pipeline and a brief explanation of each one.

```
Autonomous Pipeline
│
│─── Loomo ───> Package which contains the launch file with all configurable parameters. Also contains the tools (functions common to every modules)
│
│─── Message Types ───> Package which includes all types of messages for topics.
│
│─── Perception ───> Package and node to robustly detect and track from cameras and depth sensors. Perception package is modular enabling for change in used configurations.
|       |
|       |─── Detectors ───> Person Detection algorithms (Yolov5).
|       |
|       |─── Trackers ───> Single Object Tracker running on top of detection to robustly track the person of Interest (Stark).
|       |
|       |─── Keypoints ───> 2D/3D keypoints Estimation to get the position of the link of the target in real time.
|       |
|       |─── Perceptors ───> High Level Class where we combine all the perception module providing simple call to perception.
│
│─── Estimation ───> Package to estimate current state from sensors information.
│       │
│       │─── Robot State ───> Node to estimate the state from IMU and GPS.
│       │
│       └─── Map State ───> Node to generate a map and estimate the state from all sensors.
│
│─── Prediction ───> Package and node to generate trajectory prediction of the detected person.
│
│─── Path Planning ───> Package and node to calculate robot's desired path to avoid collision.
│
│─── Control ───> Package and node to compute control commands using Model Predictive Control and ensure that the robot is following the desired path.
│
└─── Visualization ───> Plotting tools for all nodes current data.
```

## Acknowledgements

This Repository is based on the previous project [Autonomous Driving Pipeline](https://github.com/cconejob/Autonomous_driving_pipeline)

Before adopting the pipeline for a singular purpose, we recommend to read the article that explains the previous version of this work in detail:
[Design and Implementation for Fully Autonomous Driving Systems. Paraplegic Patients Assistance](/paper.pdf) [pdf]

---

## Future Improvements
- [x] Add Perception functions from MMTrack
- [x] Add more Documentation
- [x] Redo the Install Procedure
- [ ] Create a Perception object only running yolo for simple quickstart
- [ ] Create Docker Image for easy install
- [ ] New Node for 2D/3D Keypoints to reduce latency
- [ ] Support for Multiple Person Tracking and Avoidance 
- [ ] Create a simulation environment in gazebo to be able to test the algorithms without the robot
- [ ] add a Robot description file (.urdf) of Loomo robot 








