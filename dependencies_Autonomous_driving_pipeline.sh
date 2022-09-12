#!/bin/bash

# Build tools:
sudo apt-get install -y build-essential cmake

# GUI (if you want to use GTK instead of Qt, replace 'qt5-default' with 'libgtkglext1-dev' and remove '-DWITH_QT=ON' option in CMake):
sudo apt-get install -y qt5-default libvtk6-dev

# Media I/O:
sudo apt-get install -y zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libopenexr-dev libgdal-dev
#resolving problem of installation for libjasper-dev
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt install libjasper1 libjasper-dev


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

# sudo apt-get install -y unzip wget
# wget https://github.com/opencv/opencv/archive/3.3.1.zip
# unzip 3.3.1.zip
# rm 3.3.1.zip
# mv opencv-3.3.1 OpenCV
# cd OpenCV
# mkdir build
# cd build
# cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DBUILD_EXAMPLES=ON -DENABLE_PRECOMPILED_HEADERS=OFF ..
# make -j4
# sudo make install
# sudo ldconfig


# # TRAJNET++

# ## Create directory to setup Trajnet++
# mkdir trajnet++
# cd trajnet++ 

# ## Clone Repositories
# git clone https://github.com/vita-epfl/trajnetplusplusdataset.git
# git clone https://github.com/vita-epfl/trajnetplusplusbaselines.git

# ## Download Requirements
# cd trajnetplusplusbaselines/ 
# pip install -e .

# cd ../trajnetplusplusdataset/ 
# pip install -e .
# pip install -e '.[test, plot]'


# Other common modules
python3 -m pip install Pillow
python3 -m pip install pyyaml
pip install trajnetplusplustools
pip install rospkg
pip install opencv-python
pip install netifaces
pip install tqdm

# without sudo:

curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python3 get-pip.py --force-reinstall
pip install vcstools
pip install -U rosdep
pip install -U wstool
pip install rosinstall rosdep update
pip install pyyaml

#download and install all deb packages
mkdir deb
cd deb
apt-get download libboost-all-dev libgtest-dev libpcl-dev libboost-thread-dev python3-rospkg libboost-filesystem-dev hddtemp python3-paramiko python-mock 
apt-get download libyaml-cpp-dev uuid-dev python3-empy libconsole-bridge-dev liburdfdom-dev liburdfdom-headers-dev libtinyxml-dev libtinyxml2-dev
apt-get download python3-mock libgpgme-dev python-opengl libcppunit-dev python-wxtools pyqt5-dev python3-pyqt5 python3-pyqt5.qtsvg python3-sip-dev libboost-date-time-dev 
apt-get download python3-pycryptodome python3-gnupg python3-rospkg-modules python3-catkin-pkg-modules python3-rosdep-modules python3-coverage python3-pydot python3-pygraphviz
apt-get download gazebo9 libgazebo9-dev python3-defusedxml sbcl libpoco-dev libgtk2.0-dev libfltk1.3-dev libtool-bin python3-netifaces python-sip-dev libapr1-dev
apt-get download libaprutil1-dev libboost-regex-dev libboost-system-dev liblog4cxx-dev libassimp-dev libogre-1.9-dev python3-catkin-pkg google-mock python3-nose

for entry in ./*
do
    dpkg -x "$entry" ../dir
done
# dpkg -x gazebo9_9.0.0+dfsg5-3ubuntu1+ppa2_amd64.deb dir
# dpkg -x python-mock_2.0.0-3_all.deb dir
# dpkg -x google-mock_1.8.0-6_amd64.deb dir
# dpkg -x python-opengl_3.1.0+dfsg-1_all.deb dir
# dpkg -x hddtemp_0.3-beta15-53_amd64.deb dir                    
# dpkg -x python-pyqt5.qtopengl_5.10.1+dfsg-1ubuntu2_amd64.deb dir
# dpkg -x libapr1-dev_1.6.3-2_amd64.deb dir                      
# dpkg -x python-pyqt5.qtwebkit_5.10.1+dfsg-1ubuntu2_amd64.deb dir
# dpkg -x libaprutil1-dev_1.6.1-2_amd64.deb dir                  
# dpkg -x python-rospkg_1.2.9-100_all.deb dir
# dpkg -x libassimp-dev_4.1.0~dfsg-3_amd64.deb dir               
# dpkg -x python-sip-dev_4.19.7+dfsg-1ubuntu0.1_amd64.deb dir
# dpkg -x libboost-all-dev_1.65.1.0ubuntu1_amd64.deb dir         
# dpkg -x python-wxtools_3.0.2.0+dfsg-7_all.deb dir
# dpkg -x libboost-date-time-dev_1.65.1.0ubuntu1_amd64.deb dir   
# dpkg -x python3-catkin-pkg-modules_0.4.23-1_all.deb dir
# dpkg -x libboost-filesystem-dev_1.65.1.0ubuntu1_amd64.deb dir  
# dpkg -x python3-catkin-pkg_0.4.23-100_all.deb dir
# dpkg -x libboost-regex-dev_1.65.1.0ubuntu1_amd64.deb dir       
# dpkg -x python3-coverage_4.5+dfsg.1-3_amd64.deb dir
# dpkg -x libboost-system-dev_1.65.1.0ubuntu1_amd64.deb dir      
# dpkg -x python3-defusedxml_0.5.0-1ubuntu1_all.deb dir
# dpkg -x libboost-thread-dev_1.65.1.0ubuntu1_amd64.deb dir      
# dpkg -x python3-empy_3.3.2-1build1_all.deb dir
# dpkg -x libconsole-bridge-dev_0.4.0+dfsg-2_amd64.deb dir       
# dpkg -x python3-gnupg_0.4.1-1ubuntu1.18.04.1_all.deb dir
# dpkg -x libcppunit-dev_1.14.0-3_amd64.deb dir                  
# dpkg -x python3-mock_2.0.0-3_all.deb dir
# dpkg -x libfltk1.3-dev_1.3.4-6_amd64.deb dir                   
# dpkg -x python3-netifaces_0.10.4-0.1build4_amd64.deb dir
# dpkg -x libgazebo9-dev_9.0.0+dfsg5-3ubuntu1+ppa2_amd64.deb dir 
# dpkg -x python3-nose_1.3.7-3_all.deb dir
# dpkg -x libgpgme-dev_1.10.0-1ubuntu2_amd64.deb dir             
# dpkg -x python3-paramiko_2.0.0-1ubuntu1.2_all.deb dir
# dpkg -x libgtest-dev_1.8.0-6_amd64.deb dir                     
# dpkg -x python3-pycryptodome_3.4.7-1ubuntu1_amd64.deb dir
# dpkg -x libgtk2.0-dev_2.24.32-1ubuntu1_amd64.deb dir           
# dpkg -x python3-pydot_1.2.3-1_all.deb dir
# dpkg -x liblog4cxx-dev_0.10.0-13ubuntu2_amd64.deb dir          
# dpkg -x python3-pygraphviz_1.4~rc1-1build2.1_amd64.deb dir
# dpkg -x libogre-1.9-dev_1.9.0+dfsg1-10_amd64.deb dir           
# dpkg -x python3-pyqt5.qtsvg_5.10.1+dfsg-1ubuntu2_amd64.deb dir
# dpkg -x libpcl-dev_1.8.1+dfsg1-2ubuntu2.18.04.1_amd64.deb dir  
# dpkg -x python3-pyqt5_5.10.1+dfsg-1ubuntu2_amd64.deb dir
# dpkg -x libpoco-dev_1.8.0.1-1ubuntu4_amd64.deb dir             
# dpkg -x python3-rosdep-modules_0.20.0-1_all.deb dir
# dpkg -x libtinyxml-dev_2.6.2-4_amd64.deb dir                   
# dpkg -x python3-rospkg-modules_1.2.9-1_all.deb dir
# dpkg -x libtinyxml2-dev_6.0.0+dfsg-1_amd64.deb dir             
# dpkg -x python3-rospkg_1.2.9-100_all.deb dir
# dpkg -x libtool-bin_2.4.6-2_amd64.deb dir                      
# dpkg -x python3-sip-dev_4.19.7+dfsg-1ubuntu0.1_amd64.deb dir
# dpkg -x liburdfdom-dev_1.0.0-2ubuntu0.1_amd64.deb dir          
# dpkg -x sbcl_2%3a1.4.5-1_amd64.deb dir
# dpkg -x liburdfdom-headers-dev_1.0.0-1ubuntu0.1_amd64.deb dir
# dpkg -x libyaml-cpp-dev_0.5.2-4ubuntu1_amd64.deb dir           
# dpkg -x uuid-dev_2.31.1-0.4ubuntu3.7_amd64.deb dir
# dpkg -x python-catkin-pkg_0.4.23-100_all.deb dir
