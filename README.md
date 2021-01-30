# Autonomous Driving Pipeline in ROS

<center>

Mobile Robot                |  ROS Pipeline
:-------------------------: |:-------------------------:
<img src="./src/control/Images/3_Obstacles_2.gif" alt="drawing" width="250"/> | <img src="./src/control/Images/3_Obstacles_1.gif" alt="drawing" width="500"/>

</center>

## Paper
Before adopting the pipeline for a singular purpose, we strongly recommend to read the article that explains our work in detail:
[Design and Implementation for Fully Autonomous Driving Systems](/Design_and_Implementation_of_a_Pipeline_for_Fully_Autonomous_Driving_Systems.pdf) [pdf]


## User Guide

### Quick Start

First of all, we need to clone all the ```./src``` folder inside a new ROS folder:

```shell
mkdir ROS_AD_pipeline
cd ROS_AD_pipeline
git clone https://github.com/cconejob/Autonomous_driving_pipeline.git
```

Once we have installed all **dependencies** (see subsection below), we need to build the ```devel``` and ```build``` folders by typing inside the ```ROS_AD_pipeline``` folder:

```shell
catkin_make
```

If the command ```catkin_make``` fails, try executing it again and check if the percentage is growing.

In the same folder, type the following command before launching all packages.

```shell
source devel/setup.bash
```

Now, we are ready to launch:

```shell
roslaunch loomo Loomo.launch
```

### Structure

We present all the different pillars of the pipeline and a brief explanation of each one.

```
Autonomous Pipeline
│
│─── Loomo ───> Package which contains the launch file with all configurable parameters.
│
│─── Message Types ───> Package which includes all types of messages for topics.
│
│─── Perception ───> Package and node to perceive and detect from cameras and depth sensors.
│
│─── Estimation ───> Package to estimate current state from sensors information.
│       │
│       │─── Robot State ───> Node to estimate the state from IMU and GPS.
│       │
│       └─── Map State ───> Node to generate a map and estimate the state from all sensors.
│
│─── Prediction ───> Package and node to generate future positions (motion) for detections.
│
│─── Path Planning ───> Package and node to calculate robot's desired path to avoid collision.
│
│─── Control ───> Package and node to ensure that the robot is following the desired path.
│
└─── Visualization ───> Plotting tools for all nodes current data.
```

### Dependencies

We have built a bash script and a requirements.txt file with all the modules required for our ROS pipeline. You can run them in your command line by typing:

```shell
sudo chmod +x dependencies_ROS_AD_Pipeline.sh
./dependencies_ROS_AD_Pipeline.sh
pip install -r requirements.txt
```

**Ubuntu 16.04**

https://releases.ubuntu.com/16.04/

**Python 3.7**

**ROS Kinetic**


**Python Libraries**

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


