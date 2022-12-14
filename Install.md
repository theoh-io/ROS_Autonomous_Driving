# 🛠️ Install

Grab a cup of coffee :coffee: :cookie: and get ready for the full installation procedure. This is a long and quite tedious process, take your time and follow each step carrefully. 

First step is to clone the repository inside the desired ROS folder using:

    git clone https://github.com/theoh-io/ROS_Autonomous_Driving

## Dependencies

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

#### Dependency

:warning: before install openmmlab kits be sure to have definitive version of torch and torchvision otherwise you will have to uninstall and re-install everything. In our case we use a RTX3090 GPU specifying special version of torch and torchvision. We provide a script for that: [torch script](./torch.sh)

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

* Openpifpaf 

* TrajNet++

* math

* numpy

* scipy

* matplotlib

* PIL

* torch 

* torchvision

* collections

* cython 

* OpenCV

* MMCV

## Build

Once we have installed all **dependencies** (see section below), we need to build the ```devel``` and ```build``` folders by typing inside the ```Autonomous_driving_Pipeline``` folder:

```shell
catkin_make
```

If the command ```catkin_make``` fails, try executing it again and check if the percentage is growing.

## Perception functions weight

Please Check the [installation procedure](./src/perception/README.md) to download the weights of the different models, depending on the custom Perception algorithm you plan to use.

You won't be able to use the pipeline without downloading the associated weights beforehand.

### End of Install

:tada: Congrats if you managed to reach this point most of the installation should be done, you can finally test the program.

Go back to the [UserGuide](./README.md)
