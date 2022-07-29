#!/bin/bash
set -e

echo "###########################################"
echo "INSTALLATION OF SYSTEM WIDE PACKAGES"
echo "###########################################"
echo ""
echo "Upgrading Ubuntu packages to the latest versions..."
sudo apt-add-repository universe
sudo apt --yes update
sudo apt --yes upgrade
echo "Install Curl"
sudo apt --yes install curl
echo "Install Python"
sudo apt --yes install python ipython
echo "Install Pip"
sudo apt --yes install python-pip
echo "Install Git"
sudo apt --yes install git
echo "Install rpl via apt"
sudo apt --yes install rpl
echo "Install ipython3 via apt"
sudo apt --yes install ipython3
sudo apt --yes install python3-pip
echo "Install Emacs packages"
sudo apt --yes install emacs yaml-mode
echo "Install nettools"
sudo apt --yes install net-tools
echo "Install git and wget"
sudo apt --yes install wget
echo "Install vim"
sudo apt --yes install vim
echo "Install Python packages"
sudo apt --yes install python-serial
echo "Install GSL for csm"
sudo apt --yes install libgsl0-dev
echo "Install Port Audio"
sudo apt --yes install portaudio19-dev
echo "###########################################"
echo "DONE WITH INSTALLATION OF SYSTEM WIDE PACKAGES"
echo "###########################################"
echo ""


echo "###########################################"
echo "INSTALLATION OF HARDWARE PACKAGES"
echo "###########################################"
# packages to support stretch_body
echo "Installing lm-sensors"
sudo apt-get install lm-sensors
sudo apt-get install nvme-cli
echo "###########################################"
echo "DONE WITH INSTALLATION OF HARDWARE PACKAGES"
echo "###########################################"
echo ""


# Install ROS Melodic
# see http://wiki.ros.org/melodic/Installation/Ubuntu#Installation for details
echo "###########################################"
echo "INSTALLATION OF ROS MELODIC"
echo "###########################################"

echo "Setting up sources.list"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
echo "Setting up keys"
# New key as of Jun 22, 2021
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "apt update"
sudo apt --yes update
echo "Installating Desktop-Full Version"
sudo apt --yes install ros-melodic-desktop-full
echo "Initialize rosdep"
sudo apt --yes install python-rosdep
echo "Install additional ROS packages"
sudo apt --yes install python-rosinstall python-rosinstall-generator python-wstool build-essential
echo "###########################################"
echo "DONE WITH MAIN INSTALLATION OF ROS MELODIC"
echo "###########################################"
echo ""

################ Additional packages#####################################
echo "###########################################"
echo "ADDITIONAL INSTALLATION OF ROS MELODIC"
echo "###########################################"
echo "Install packages to work with URDFs"
sudo apt --yes install liburdfdom-tools meshlab
echo "Install cheese for camera testing"
sudo apt --yes install cheese
echo "Install joint state GUI package"
sudo apt --yes install ros-melodic-joint-state-publisher-gui
echo "Install TF2 related packages"
sudo apt --yes install ros-melodic-tf2-tools
echo "Install IMU visualization plugin for RViz and IMU filter"
sudo apt --yes install ros-melodic-rviz-imu-plugin ros-melodic-imu-filter-madgwick
echo "Install robot pose filter for use with IMU and wheel odometry"
sudo apt --yes install ros-melodic-robot-pose-ekf
echo "Install robot localization package for use with IMU and wheel odometry"
sudo apt --yes install ros-melodic-robot-localization
echo "Install ros_numpy package for msgs conversions"
sudo apt --yes install ros-melodic-ros-numpy
echo "Install ROS packages for Robotis Dynamixel actuators"
sudo apt --yes install ros-melodic-dynamixel-sdk ros-melodic-dynamixel-workbench
echo "Install ROS control packages (primarily for simulations with Gazebo)"
sudo apt --yes install ros-melodic-ros-control ros-melodic-ros-controllers
echo "Install dynamic reconfiguration package for use with Intel D435i"
sudo apt --yes install ros-melodic-ddynamic-reconfigure-python
echo "Install ROS teleop packages"
sudo apt --yes install ros-melodic-teleop-twist-keyboard
#echo "Install ROS navigation and mapping packages"
#sudo apt --yes install ros-melodic-move-base ros-melodic-map-server ros-melodic-amcl ros-melodic-cartographer ros-melodic-cartographer-ros ros-melodic-cartographer-rviz
sudo apt --yes install ros-melodic-move-base ros-melodic-move-base-msgs
sudo apt --yes install ros-melodic-gmapping ros-melodic-navigation
#echo "Install ROS MoveIt! installation"
#sudo apt --yes install ros-melodic-moveit
# SMACH VIEWER HAS A BUG, SO NOT INSTALLING FOR NOW
#echo "Install additional SMACH packages"
#sudo apt install ros-melodic-smach-viewer
echo "Install RPLidar A1M8 packages"
sudo apt --yes install ros-melodic-rplidar-ros ros-melodic-rplidar-ros-dbgsym
echo "Install Respeaker and speech recognition packages"
sudo apt --yes install ros-melodic-respeaker-ros ros-melodic-ros-speech-recognition
echo "###########################################"
echo "DONE WITH ADDITIONAL INSTALLATION OF ROS MELODIC"
echo "###########################################"
echo ""


echo "###########################################"
echo "INSTALLATION OF INTEL D435i"
echo "###########################################"
sudo apt --yes install ros-melodic-realsense2-camera ros-melodic-realsense2-description
# "The following NEW packages will be installed:
#  ros-melodic-ddynamic-reconfigure ros-melodic-librealsense2 ros-melodic-realsense2-camera"

echo "Register the librealsense APT server's public key"
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
echo "Add the librealsense APT server to the list of APT repositories"
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u
echo "Remove old records in case of upgrading"
sudo rm -f /etc/apt/sources.list.d/realsense-public.list
echo "Apt update"
sudo apt --yes update
echo "Install D435i packages"
sudo apt --yes install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
echo "###########################################"
echo "DONE WITH INSTALLATION OF INTEL D435i"
echo "NOTE: Update firmware using realsense-viewer"
echo "###########################################"
echo ""


