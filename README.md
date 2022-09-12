# Autonomous Driving Pipeline in ROS

## Paper
Before adopting the pipeline for a singular purpose, we strongly recommend to read the article that explains our work in detail:
[Design and Implementation for Fully Autonomous Driving Systems. Paraplegic Patients Assistance](/paper.pdf) [pdf]

<p align="center">
<img src="./src/control/Images/MR_Loomo_closed_loop.gif" alt="drawing" width="700"/>
</p>

## User Guide

### Quick Start



### Structure

We present all the different pillars of the pipeline and a brief explanation of each one.

```
Autonomous Pipeline
│
│─── Loomo ───> Package which contains the launch file with all configurable parameters.
│
│─── Message Types ───> Package which includes all types of messages for topics.
│
│─── Perception ───> Package and node to robustly detect and track from cameras and depth sensors.
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
│─── Control ───> Package and node to ensure that the robot is following the desired path.
│
└─── Visualization ───> Plotting tools for all nodes current data.
```
---
## Install

Grab a cup of coffee :coffee: :cookie: and get ready for the full installation procedure. This is a long and quite tedious process, take your time and follow each step carrefully. Everything is going to be fine :relieved:.

First step is to clone the repository inside the desired ROS folder using:

    git clone https://github.com/theoh-io/ROS_Autonomous_Driving

### Dependencies

**Python 3.7**

Check if python 3.7 is already installed running python3.7 in terminal.
If you do not have this Python version, we recommend you to download it by typing the following command in the Ubuntu terminal:

```shell
sudo chmod +x python_Autonomous_driving_pipeline.sh
./python_Autonomous_driving_pipeline.sh
```

**ROS Kinetic/Melodic/Noetic**

Depending on the Ubuntu version, there are three compatible ROS distributions.

* Ubuntu 16.04: ROS Kinetic. http://wiki.ros.org/kinetic/Installation/Ubuntu

* Ubuntu 18.04: ROS Melodic. http://wiki.ros.org/melodic/Installation/Ubuntu

* Ubuntu 20.04: ROS Noetic. http://wiki.ros.org/noetic/Installation/Ubuntu

Check installed Version by using `rosversion -d ` in terminal

**Python Libraries**

First create a virtual environment using python 3.7 by typing `python3.7 -m venv <name of your virtual environment>` and then activate it using `source <name venv>/bin/activate`. 

Finally we advise to update pip and basic package using `python -m pip install -U pip wheel setuptools --no-cache-dir `

We have built a bash script and a requirements.txt file with all the python modules required for our ROS pipeline. You can run them in your command line by typing:

```shell
sudo chmod +x dependencies_Autonomous_driving_pipeline.sh
./dependencies_Autonomous_driving_pipeline.sh
pip install -r requirements.txt
```

:warning: this step might cause you some trouble, if the scripts keeps throwing error messages try to run it line by line and identifying the command causing problem. If it persists feel free to create an issue in the repo.

**TrajNet++**

Package used in Prediction.

We have made a bash script to automatize the installation, you can run it by calling:

    sudo ./trajnet.sh
    ./trajnet.sh

In case of problem or for detailed install, follow the installation procedure described in the [trajnetbaseline](https://github.com/vita-epfl/trajnetplusplusbaselines) repo. and this step by step [guide](https://thedebugger811.github.io/posts/2020/03/intro_trajnetpp/).

**MMTracking**

Package used in Perception. Implementation of state-of-the-art method in SOT/MOT

We have made a bash script to automatize the installation, you can run it by calling:

    sudo ./mmtracking.sh
    ./mmtracking.sh

In case of problem or for further details, please check the official repo: [MMTracking](https://github.com/open-mmlab/mmtracking)

**MMPose**

Package used in Perception. Implementation of state-of-the-art method for 2D/3D pose and keypoints estimation.

We have made a bash script to automatize the installation, you can run it by calling:

    sudo ./mmpose.sh
    ./mmpose.sh

In case of problem or for further details, please check the official repo: [MMPose](https://github.com/open-mmlab/mmpose)


Indicative list of dependencies:
* Openpifpaf 0.11.9

* TrajNet++

* math

* numpy

* scipy

* matplotlib

* PIL

* torch 1.7.1

* collections

* cython 0.29.21 

* OpenCV 3.3

### Build

Once we have installed all **dependencies** (see section below), we need to build the ```devel``` and ```build``` folders by typing inside the ```Autonomous_driving_Pipeline``` folder:

```shell
catkin_make
```

If the command ```catkin_make``` fails, try executing it again and check if the percentage is growing.

In the same folder, type the following command before launching all packages.

```shell
source devel/setup.bash
```

If we need to change some parameters of the ROS structure, we can go to [Loomo.launch](/src/loomo) inside the launch folder.

The Autonomous System used in our work is a Loomo Segway Robot. It needs some specific algorithms to enable the connection with the server (ROS pipeline), presented in https://github.com/cconejob/loomo-vita-testing-app.

Once everything is set up and adjusted to our specific context, we are ready to launch:

```shell
roslaunch loomo Loomo.launch
```

## Future Improvements
- [x] Add Perception functions from MMTrack
- [x] Add more Documentation
- [x] Redo the Install Procedure
- [ ] Create Docker Image for easy install
- [ ] Support for Multiple Person Tracking and Avoidance 
- [ ] Create a simulation environment in gazebo to be able to test the algorithms without the robot
- [ ] add a Robot description file (.urdf) of Loomo robot 








