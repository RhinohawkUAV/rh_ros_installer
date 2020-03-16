#!/bin/sh
#Author: JESSE THALOOR mazhai@gmail.com
#Date: 03/16/2020


if [ $USER = 'root' ]
then
  echo "Error: Please run this as non-root user"
  exit 1
fi

INSTALL_DIR=${PWD}

# Ubuntu Version. This is hardcoded as lsb_release does not work in all *nix platforms.
UBUNTU_VERSION="bionic"

# ROS Version
ROS_VERSION="melodic"

# Directory locations
CATKIN_WS=${HOME}/catkin_ws
ARDUPILOT_SRC=${HOME}/src/

# GIT Locations
ARDUPILOT_GIT=https://github.com/ArduPilot/ardupilot
RHINOHAWK_GIT=https://github.com/RhinohawkUAV/rh_ros.git
ALIENCONTROL_GIT=https://github.com/acschaefer/aliencontrol.git
GSCAM_GIT=https://github.com/ros-drivers/gscam.git

# setup ROS repositories
echo "deb http://packages.ros.org/ros/ubuntu ${UBUNTU_VERSION} main" > ${HOME}/ros-latest.list
sudo cp ${HOME}/ros-latest.list /etc/apt/sources.list.d/

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"

# Install ROS Base (not the full or desktop version)
sudo apt-get update 
sudo apt-get install -y \
			libopencv-dev libprotobuf-dev \
            libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
			libgstreamer-plugins-good1.0-dev \
			ros-${ROS_VERSION}-desktop \
			ros-${ROS_VERSION}-gazebo-ros \
			ros-${ROS_VERSION}-geographic-msgs \
            ros-${ROS_VERSION}-vision-opencv \
			ros-${ROS_VERSION}-mavlink \
			ros-${ROS_VERSION}-mavros \
			ros-${ROS_VERSION}-polled-camera \
            ros-${ROS_VERSION}-rosbridge-server \
			ros-${ROS_VERSION}-cv-bridge\
			ros-${ROS_VERSION}-video-stream-opencv \
			git python3-pip python-pip
sudo apt-get clean autoclean

# Install catkin_python3
sudo pip3 install catkin_pkg
sudo pip3 -q install scikit-learn pykml pymavlink
sudo pip -q install scikit-learn pykml pymavlink

# update bashrc
echo "source /opt/ros/${ROS_VERSION}/setup.bash" >> ${HOME}/.bashrc

# Create directories
mkdir -p ${CATKIN_WS}
mkdir -p ${ARDUPILOT_SRC}

# update ROS
source  /opt/ros/${ROS_VERSION}/setup.bash
sudo rosdep init
rosdep update

# install Ardupilot
cd ${ARDUPILOT_SRC}
git clone --recursive ${ARDUPILOT_GIT}
cd ardupilot/Tools/environment_install
./install-prereqs-ubuntu.sh -y

# compile arduplane for faster first time sim_vehicle.py startup
cd ${ARDUPILOT_SRC}/ardupilot
./waf configure
./waf plane

# Add airport location
cat ${INSTALL_DIR}/locations.txt >> Tools/autotest/locations.txt

#install RH
cd ${CATKIN_WS}
mkdir src
cd src
git clone --recursive ${RHINOHAWK_GIT}

# install aliencontrol
git clone --recursive  ${ALIENCONTROL_GIT}

#install gscam
git clone --recursive  ${GSCAM_GIT}

# install geolib
sudo sh ${INSTALL_DIR}/install_geolib.sh

# build instructions
cd ${CATKIN_WS}

echo "Please follow these steps to setup rh_ros for SITL testing\
1) Source /opt/ros/${ROS_VERSION}/setup.bash\
2) Run catkin_make from the catkin_ws directory\
3) Follow rh_simulation instructions"

