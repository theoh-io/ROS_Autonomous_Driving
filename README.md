# Autonomous Driving Pipeline in ROS

## Paper
Before adopting the pipeline for a singular purpose, we strongly recommend to read the article that explains our work in detail:
[Design and Implementation for Fully Autonomous Driving Systems. Paraplegic Patients Assistance](/paper.pdf) [pdf]

<center>

<img src="./src/control/Images/MR_Loomo_closed_loop.gif" alt="drawing" width="700"/>

</center>


## User Guide

### Quick Start

First of all, we need to clone all the ```./src``` folder inside a new ROS folder:

```shell
git clone https://github.com/cconejob/Autonomous_driving_pipeline.git
```

Once we have installed all **dependencies** (see section below), we need to build the ```devel``` and ```build``` folders by typing inside the ```Autonomous_driving_Pipeline``` folder:

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

**Python 3.7**

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

We have built a bash script and a requirements.txt file with all the python modules required for our ROS pipeline. You can run them in your command line by typing:

```shell
sudo chmod +x dependencies_Autonomous_driving_pipeline.sh
./dependencies_Autonomous_driving_pipeline.sh
pip install -r requirements.txt
```






