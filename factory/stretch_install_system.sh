#!/bin/bash
set -e

function install {
    sudo apt-get install -y "$@" > /dev/null
}

echo "###########################################"
echo "INSTALLATION OF SYSTEM WIDE PACKAGES"
echo "###########################################"
echo "Apt update & upgrade"
sudo apt-add-repository universe > /dev/null
sudo apt-get --yes update > /dev/null
sudo apt-get --yes upgrade > /dev/null
echo "Install zip & unzip"
install zip unzip
echo "Install Curl"
install curl
echo "Install Python"
install python ipython
echo "Install Pip"
install python-pip
echo "Install Git"
install git
echo "Install rpl via apt"
install rpl
echo "Install ipython3 via apt"
install ipython3
install python3-pip
echo "Install Emacs packages"
install emacs yaml-mode
echo "Install nettools"
install net-tools
echo "Install wget"
install wget
echo "Install vim"
install vim
echo "Install Python packages"
install python-serial
echo "Install GSL for csm"
install libgsl0-dev
echo "Install Port Audio"
install portaudio19-dev
echo "Install lm-sensors & nvme-cli"
install lm-sensors
install nvme-cli
echo "###########################################"
echo "DONE WITH INSTALLATION OF SYSTEM WIDE PACKAGES"
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
echo "Apt update"
sudo apt-get --yes update > /dev/null
echo "Install ROS Melodic desktop"
install ros-melodic-desktop-full
echo "Install rosdep"
install python-rosdep
echo "Install other ROS workspace tools"
install python-rosinstall python-rosinstall-generator python-wstool build-essential
echo "###########################################"
echo "DONE WITH INSTALLATION OF ROS MELODIC"
echo "###########################################"
echo ""

echo "###########################################"
echo "INSTALLATION OF ADDITIONAL ROS MELODIC PKGS"
echo "###########################################"
echo "Install packages to work with URDFs"
install liburdfdom-tools meshlab
echo "Install cheese for camera testing"
install cheese
echo "Install joint state GUI package"
install ros-melodic-joint-state-publisher-gui
echo "Install TF2 related packages"
install ros-melodic-tf2-tools
echo "Install IMU visualization plugin for RViz and IMU filter"
install ros-melodic-rviz-imu-plugin ros-melodic-imu-filter-madgwick
echo "Install robot pose filter for use with IMU and wheel odometry"
install ros-melodic-robot-pose-ekf
echo "Install robot localization package for use with IMU and wheel odometry"
install ros-melodic-robot-localization
echo "Install ros_numpy package for msgs conversions"
install ros-melodic-ros-numpy
echo "Install ROS packages for Robotis Dynamixel actuators"
install ros-melodic-dynamixel-sdk ros-melodic-dynamixel-workbench
echo "Install ROS control packages (primarily for simulations with Gazebo)"
install ros-melodic-ros-control ros-melodic-ros-controllers
echo "Install dynamic reconfiguration package for use with Intel D435i"
install ros-melodic-ddynamic-reconfigure-python
echo "Install ROS teleop packages"
install ros-melodic-teleop-twist-keyboard
#echo "Install ROS navigation and mapping packages"
#install ros-melodic-move-base ros-melodic-map-server ros-melodic-amcl ros-melodic-cartographer ros-melodic-cartographer-ros ros-melodic-cartographer-rviz
install ros-melodic-move-base ros-melodic-move-base-msgs
install ros-melodic-gmapping ros-melodic-navigation
echo "Install RPLidar A1M8 packages"
install ros-melodic-rplidar-ros ros-melodic-rplidar-ros-dbgsym
echo "Install Respeaker and speech recognition packages"
install ros-melodic-respeaker-ros ros-melodic-ros-speech-recognition
echo "###########################################"
echo "DONE WITH INSTALLATION OF ADDITIONAL ROS MELODIC PKGS"
echo "###########################################"
echo ""


echo "###########################################"
echo "INSTALLATION OF INTEL D435i"
echo "###########################################"
echo "Install realsense-ros"
install ros-melodic-realsense2-camera ros-melodic-realsense2-description
echo "Register the librealsense APT server's public key"
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
echo "Add the librealsense APT server to the list of APT repositories"
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u
echo "Remove old records in case of upgrading"
sudo rm -f /etc/apt/sources.list.d/realsense-public.list
echo "Apt update"
sudo apt-get --yes update > /dev/null
echo "Install librealsense2 packages"
install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
echo "###########################################"
echo "DONE WITH INSTALLATION OF INTEL D435i"
echo "NOTE: Update firmware using realsense-viewer"
echo "###########################################"
echo ""

