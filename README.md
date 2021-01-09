# Autonomous Driving Pipeline in ROS

## User Guide

### Quick Start



### Structure

```shell
Autonomous Pipeline
│─── Loomo ───> Launch file with all configurable parameters.
│─── Perception ───> Perceive and detect objects/persons from cameras and depth sensors.
│─── Estimation ───> Estimate current state from sensors information.
│─── Prediction ───> Generate future predicted positions (motion) for detections.
│─── Path Planning ───> Calculate the desired path for the robot in order to avoid collision.
│─── Control ───> Ensure that the robot is following the desired path.
│─── Visualization ───> Plotting tools for all nodes.
└─── Message Types ───> Contains all types of messages for topics.
```

### Examples



## Dependencies

### Ubuntu

* **Version 16.04:** https://releases.ubuntu.com/16.04/

### Python

We strongly recommend to install both versions: python 2 and 3.

* **Version 2.7:** https://tecadmin.net/install-python-2-7-on-ubuntu-and-linuxmint/

* **Version 3.7:** https://websiteforstudents.com/installing-the-latest-python-3-7-on-ubuntu-16-04-18-04/

### ROS Kinetic

* **Kinetic:** http://wiki.ros.org/kinetic/Installation/Ubuntu

### OpenCV

* **Version 3.3:** https://gist.github.com/danigosa/367b8a8cbc8d883df80c5c071423e4b2

### Openpifpaf

* **Version 0.11.9:** https://pypi.org/project/openpifpaf/

### TrajNet++
Follow instructions on https://thedebugger811.github.io/posts/2020/03/intro_trajnetpp/

* **trajnetplusplusbaselines**
* **trajnetplusplusdataset**
* **trajnetv**

### Other modules

We present other modules, which are required for our ROS pipeline.

* **math**

```shell
pip install python-math
```

* **numpy**

```shell
pip install numpy
```

* **scipy**

```shell
pip install scipy
```

* **matplotlib**

```shell
pip install matplotlib
```

* **PIL**

```shell
python3 -m pip install Pillow
```

* **torch 1.7.1**

```shell
pip install torch
```

* **collections**

```shell
pip install collections-extended
```

* **cython 0.29.21** 

```shell
pip install Cython
```