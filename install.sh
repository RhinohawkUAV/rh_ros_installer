#!/bin/bash 
#
# Rhinohawk Installation Script
#

# Exit on error
set -e

# The location of this script
INSTALLER_DIR=$(dirname "$0")

# ROS distribution to install 
ROS_DIST=kinetic

# Installation command for system packages
INSTALL_CMD="sudo apt-get install -y"

# Installation command for Python packages
PYINSTALL_CMD="sudo pip install"

# Git clone command
GIT_CLONE_CMD="git clone"

# Source directory for non-ROS software
SRC_DIR=$HOME/src

# Catkin workspace for ROS nodes
CATKIN_WS_DIR=$HOME/catkin_ws

# Git repositories
ARDUPILOT_GIT=https://github.com/ArduPilot/ardupilot
RHINOHAWK_GIT=https://github.com/RhinohawkUAV/rh_ros.git
GSCAM_GIT=https://github.com/ros-drivers/gscam.git


# Installer Prompt
echo
echo ".----------------------------------------."
echo "| Welcome to the Rhinohawk ROS Installer |"
echo "'----------------------------------------'"
echo
echo "This script will install:"
echo "  - ROS $ROS_DIST using '$INSTALL_CMD ...'"
echo "  - Required Python libraries using '$PYINSTALL_CMD ...'"
echo
echo "Then it will clone these Git repositories:"
echo "  - $ARDUPILOT_GIT into $SRC_DIR"
echo "  - $RHINOHAWK_GIT into $CATKIN_WS_DIR/src"
echo "  - $GSCAM_GIT into $CATKIN_WS_DIR/src"
echo 
echo "You can modify install locations by editing the script."
echo 
read -p "Are you sure you want to proceed? (Y/N) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]
then

echo "Starting installation..."
sleep 1


####################################################################
# Install ROS 
# From http://wiki.ros.org/kinetic/Installation/Ubuntu
####################################################################

DIST=$ROS_DIST
echo "Installing ROS $DIST"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
$INSTALL_CMD ros-$DIST-desktop
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

####################################################################
# Install ArduPilot
####################################################################
mkdir -p $SRC_DIR
cd $SRC_DIR
git clone $ARDUPILOT_GIT
cd ardupilot
git submodule update --init --recursive
./Tools/scripts/install-prereqs-ubuntu.sh -y
echo "export PATH=$PATH:$HOME/src/ardupilot/Tools/autotest" >> ~/.bashrc
echo "export PATH=/usr/lib/ccache:$PATH" >> ~/.bashrc
source ~/.bashrc

# Add our custom starting locations for the simulator
cat $INSTALLER_DIR/locations.txt >> $SRC_DIR/ardupilot/Tools/autotest/locations.txt


####################################################################
# Install Rhinohawk Dependencies
####################################################################
echo "Installing additional ROS packages"
$INSTALL_CMD libav-tools gphoto2 libgphoto2-dev \
    build-essential ros-$DIST-vision-opencv ros-$DIST-polled-camera \
    ros-$DIST-camera-info-manager ros-$DIST-camera-info-manager-py \
    ros-$DIST-tf ros-$DIST-image-proc ros-$DIST-rosbridge-server \
    ros-$DIST-mavros

sudo /opt/ros/kinetic/lib/mavros/install_geographiclib_datasets.sh

$INSTALL_CMD ros-kinetic-rqt ros-kinetic-jsk-rqt-plugins

$INSTALL_CMD python-pip
$PYINSTALL_CMD --upgrade pip==9.0.3
$PYINSTALL_CMD shapely typing scikit-learn pykml


####################################################################
# Install Rhinohawk 
####################################################################

echo "Installing Rhinohawk at $CATKIN_WS"
mkdir -p $CATKIN_WS_DIR/src
cd $CATKIN_WS_DIR/src
catkin_init_workspace
$GIT_CLONE_CMD $RHINOHAWK_GIT


####################################################################
# Install Gscam for 3dr Solo Interoperability 
####################################################################
$INSTALL_CMD gstreamer1.0-tools libgstreamer1.0-dev \
	libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
cd $CATKIN_WS_DIR/src
$GIT_CLONE_CMD $GSCAM_GIT
cd $CATKIN_WS_DIR


echo "Running Catkin Make"
cd $CATKIN_WS_DIR
catkin_make install

echo
echo "Rhinohawk has been successfully installed."
echo

else
echo "Installation cancelled."
fi

