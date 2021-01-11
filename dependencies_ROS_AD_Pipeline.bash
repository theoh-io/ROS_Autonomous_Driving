#!/bin/bash

# PYTHON 3.7

# build a docker
docker build --tag python_builder .
docker run --rm --interactive --tty --name python_builder python_builder

# run inside the docker container:
set -o errexit -o nounset -o pipefail -o xtrace

PYTHON_VERSION=3.7.6

# this must not be plain python as otherwise the system Python gets overridden
PACKAGE_NAME=python3.7

# sources need to be available for build-deb
sed --in-place=~orig 's/# deb-src/deb-src/' /etc/apt/sources.list

apt-get update
apt-get upgrade

# install things we need for this script
apt-get install --assume-yes \
    checkinstall \
    wget \

# install things to compile Python
apt-get build-dep --assume-yes python3.5

# download the latest version (from https://www.python.org/downloads/source/)
wget https://www.python.org/ftp/python/${PYTHON_VERSION}/Python-${PYTHON_VERSION}.tgz

tar xvf Python-${PYTHON_VERSION}.tgz
cd Python-${PYTHON_VERSION}

# enabling optimizations requires running tests which takes a long time but results in a faster Python
./configure \
    --enable-optimizations \
    --with-ensurepip=install \

# compile as much as possible in parallel
make --jobs 4

# use altinstall to not replace the system Python
checkinstall --pkgname ${PACKAGE_NAME} --pkgversion ${PYTHON_VERSION} --default make altinstall

# run this from a another terminal while the container is still running:
docker cp python_builder:/Python-3.7.2/python3.7_3.7.2-1_amd64.deb .

# install the package
sudo dpkg -i python3.7_3.7.2-1_amd64.deb


# ROS KINETIC

echo ""
echo "[Note] Target OS version  >>> Ubuntu 16.04.x (xenial) or Linux Mint 18.x"
echo "[Note] Target ROS version >>> ROS Kinetic Kame"
echo "[Note] Catkin workspace   >>> $HOME/catkin_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

name_os_version=${name_os_version:="xenial"}
name_ros_version=${name_ros_version:="kinetic"}
name_catkin_workspace=${name_catkin_workspace:="catkin_ws"}

sudo apt-get update -y
sudo apt-get upgrade -y

sudo apt-get install -y chrony ntpdate build-essential
sudo ntpdate ntp.ubuntu.com

if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${name_os_version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

roskey=`apt-key list | grep "Open Robotics"`
if [ -z "$roskey" ]; then
  sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
fi

roskey=`apt-key list | grep "Open Robotics"`
if [ -n "$roskey" ]; then
  echo "[ROS key exists in the list]"
else
  echo "[Failed to receive the ROS key, aborts the installation]"
  exit 0
fi

sudo apt-get update -y
sudo apt-get upgrade -y

sudo apt-get install -y ros-$name_ros_version-desktop-full ros-$name_ros_version-rqt-*

sudo sh -c "rosdep init"
rosdep update

source /opt/ros/$name_ros_version/setup.sh
sudo apt-get install -y python-rosinstall git

mkdir -p $HOME/$name_catkin_workspace/src
cd $HOME/$name_catkin_workspace/src
catkin_init_workspace
cd $HOME/$name_catkin_workspace
catkin_make

sh -c "echo \"alias eb='nano ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias gs='git status'\" >> ~/.bashrc"
sh -c "echo \"alias gp='git pull'\" >> ~/.bashrc"
sh -c "echo \"alias cw='cd ~/$name_catkin_workspace'\" >> ~/.bashrc"
sh -c "echo \"alias cs='cd ~/$name_catkin_workspace/src'\" >> ~/.bashrc"
sh -c "echo \"alias cm='cd ~/$name_catkin_workspace && catkin_make'\" >> ~/.bashrc"

sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_catkin_workspace/devel/setup.bash\" >> ~/.bashrc"

sh -c "echo \"export ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc"
sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.bashrc"

source $HOME/.bashrc


# OPENCV 3.3

sudo apt-get -y update
sudo apt-get -y upgrade
sudo apt-get -y dist-upgrade
sudo apt-get -y autoremove

# Build tools:
sudo apt-get install -y build-essential cmake

# GUI (if you want to use GTK instead of Qt, replace 'qt5-default' with 'libgtkglext1-dev' and remove '-DWITH_QT=ON' option in CMake):
sudo apt-get install -y qt5-default libvtk6-dev

# Media I/O:
sudo apt-get install -y zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libjasper-dev libopenexr-dev libgdal-dev

# Video I/O:
sudo apt-get install -y libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev

# Parallelism and linear algebra libraries:
sudo apt-get install -y libtbb-dev libeigen3-dev

# Python:
sudo apt-get install -y python-dev python-tk python-numpy python3-dev python3-tk python3-numpy

# Java:
sudo apt-get install -y ant default-jdk

# Documentation:
sudo apt-get install -y doxygen

sudo apt-get install -y unzip wget
wget https://github.com/opencv/opencv/archive/3.3.1.zip
unzip 3.3.1.zip
rm 3.3.1.zip
mv opencv-3.3.1 OpenCV
cd OpenCV
mkdir build
cd build
cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DBUILD_EXAMPLES=ON -DENABLE_PRECOMPILED_HEADERS=OFF ..
make -j4
sudo make install
sudo ldconfig


# TRAJNET++

## Create directory to setup Trajnet++
mkdir trajnet++
cd trajnet++ 

## Clone Repositories
git clone https://github.com/vita-epfl/trajnetplusplusdataset.git
git clone https://github.com/vita-epfl/trajnetplusplusbaselines.git

## Make virtual environment
virtualenv -p /usr/bin/python3.6 trajnetv
source trajnetv/bin/activate

## Download Requirements
cd trajnetplusplusbaselines/ 
pip install -e .

cd ../trajnetplusplusdataset/ 
pip install -e .
pip install -e '.[test, plot]'


# Other common modules
sudo -H python3 -m pip install Pillow
