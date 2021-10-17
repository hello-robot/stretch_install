#!/bin/bash

echo "This script will install and update system wide packages for Stretch"

echo "###########################################"
echo "INSTALLATION OF OS PACKAGES"
# upgrade to the latest versions of Ubuntu packages
echo "Upgrading Ubuntu packages to the latest versions..."
sudo apt --yes update
sudo apt --yes upgrade
echo "Done."
echo ""
echo "Install Curl"
sudo apt --yes install curl
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
sudo apt --yes install git wget
echo "Install vim"
sudo apt --yes install vim
echo "Install Python packages"
sudo apt --yes install python3-serial
echo "Install GSL for csm"
sudo apt --yes install libgsl0-dev
echo "Install Cython for FUNMAP"
sudo apt --yes install cython3
echo "Install Port Audio"
sudo apt --yes install portaudio19-dev
echo "DONE WITH MAIN INSTALLATION OF OS PACKAGES"
echo "###########################################"
echo ""


echo "###########################################"
echo "INSTALLATION OF HARDWARE PACKAGES"
# packages to support stretch_body
echo "Installing lm-sensors"
sudo apt-get install lm-sensors
if ! nvme_loc="$(type -p "nvme")" || [[ -z $nvme_loc ]]; then
    echo "Making and installing nvme"
    cd ~/
    git clone https://github.com/linux-nvme/nvme-cli.git
    cd nvme-cli/
    make
    sudo make install
    cd ..
    rm -rf nvme-cli
fi
echo "DONE WITH MAIN INSTALLATION OF HARDWARE PACKAGES"
echo "###########################################"
echo ""


# Install ROS Noetic
# see http://wiki.ros.org/noetic/Installation/Ubuntu for details
echo "###########################################"
echo "MAIN INSTALLATION OF ROS NOETIC"
echo "Setting up sources.list"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
echo "Setting up keys"
# New key as of Jun 22, 2021
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "apt update"
sudo apt --yes update
echo "Installating Desktop-Full Version"
sudo apt --yes install ros-noetic-desktop-full
echo "Initialize rosdep"
sudo apt --yes install python3-rosdep
sudo rosdep init
rosdep update
echo "Install vcstool ROS packages"
sudo apt --yes install python3-vcstool
echo "Source .bash file"
source /opt/ros/noetic/setup.bash
echo "DONE WITH MAIN INSTALLATION OF ROS NOETIC"
echo "###########################################"
echo ""

# Install ROS2 Galactic
# see https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html for details
echo "###########################################"
echo "MAIN INSTALLATION OF ROS2 GALACTIC"
echo "Setting up keys"
# New key as of Jun 22, 2021
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "Setting up sources.list"
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
echo "apt update"
sudo apt --yes update
echo "Installating Desktop Version"
sudo apt --yes install ros-galactic-desktop
echo "Initialize rosdep"
sudo apt --yes install python3-rosdep
sudo rosdep init
rosdep update
echo "Source .bash file"
source /opt/ros/galactic/setup.bash
echo "DONE WITH MAIN INSTALLATION OF ROS2 GALACTIC"
echo "###########################################"
echo ""

################ Additional packages#####################################
echo "###########################################"
echo "ADDITIONAL INSTALLATION OF ROS NOETIC"
echo "Install packages to work with URDFs"
sudo apt --yes install liburdfdom-tools meshlab ros-noetic-urdfdom-py
echo "Install cheese for camera testing"
sudo apt --yes install cheese
echo "Install joint state GUI package"
sudo apt --yes install ros-noetic-joint-state-publisher-gui
echo "Install TF2 related packages"
sudo apt --yes install ros-noetic-tf2-tools
echo "Install IMU visualization plugin for RViz and IMU filter"
sudo apt --yes install ros-noetic-rviz-imu-plugin ros-noetic-imu-filter-madgwick
#echo "Install robot pose filter for use with IMU and wheel odometry"
#sudo apt --yes install ros-noetic-robot-pose-ekf
echo "Install robot localization package for use with IMU and wheel odometry"
sudo apt --yes install ros-noetic-robot-localization
echo "Install ros_numpy package for msgs conversions"
sudo apt --yes install ros-noetic-ros-numpy
echo "Install ROS control packages (primarily for simulations with Gazebo)"
sudo apt --yes install ros-noetic-ros-control ros-noetic-ros-controllers
echo "Install ROS RGB-D package and dynamic reconfiguration package for use with Intel D435i"
sudo apt --yes install ros-noetic-rgbd-launch
echo "Install ROS teleop packages"
sudo apt --yes install ros-noetic-teleop-twist-keyboard
echo "Install ROS navigation and mapping packages"
#sudo apt --yes install ros-noetic-move-base ros-noetic-map-server ros-noetic-amcl ros-noetic-cartographer ros-noetic-cartographer-ros ros-noetic-cartographer-rviz
sudo apt --yes install ros-noetic-move-base ros-noetic-move-base-msgs
sudo apt --yes install ros-noetic-gmapping ros-noetic-navigation
#echo "Install ROS MoveIt! installation"
#sudo apt --yes install ros-noetic-moveit
# SMACH VIEWER HAS A BUG, SO NOT INSTALLING FOR NOW
#echo "Install additional SMACH packages"
#sudo apt install ros-noetic-smach-viewer
echo "Will build RPLidar A1M8 packages in workspace"
# sudo apt --yes install ros-noetic-rplidar-ros ros-noetic-rplidar-ros-dbgsym
#echo "Install Respeaker and speech recognition packages"
#sudo apt --yes install ros-noetic-respeaker-ros ros-noetic-ros-speech-recognition python3-pyaudio
echo "DONE WITH ADDITIONAL INSTALLATION OF ROS NOETIC"
echo "###########################################"
echo ""

echo "###########################################"
echo "ADDITIONAL INSTALLATION OF ROS2 GALACTIC"
echo "Install colcon to build ROS2 packages"
sudo apt --yes install python3-colcon-common-extensions
echo "Install packages to work with URDFs"
sudo apt --yes install liburdfdom-tools meshlab ros-galactic-urdfdom-py
echo "Install joint state GUI package"
sudo apt --yes install ros-galactic-joint-state-publisher-gui
echo "Install TF2 related packages"
sudo apt --yes install ros-galactic-tf2-tools
echo "Install IMU visualization plugin for RViz and IMU filter"
sudo apt --yes install ros-galactic-rviz-imu-plugin ros-galactic-imu-filter-madgwick
#echo "Install robot pose filter for use with IMU and wheel odometry"
#sudo apt --yes install ros-galactic-robot-pose-ekf
echo "Install robot localization package for use with IMU and wheel odometry"
sudo apt --yes install ros-galactic-robot-localization
echo "Install ros_numpy package for msgs conversions"
sudo apt --yes install ros-galactic-ros-numpy
echo "Install ROS control packages (primarily for simulations with Gazebo)"
sudo apt --yes install ros-galactic-ros2-control ros-galactic-ros2-controllers
echo "Install ROS RGB-D package and dynamic reconfiguration package for use with Intel D435i"
sudo apt --yes install ros-galactic-rgbd-launch
echo "Install ROS teleop packages"
sudo apt --yes install ros-galactic-teleop-twist-keyboard
echo "Install ROS navigation and mapping packages"
#sudo apt --yes install ros-galactic-move-base ros-galactic-map-server ros-galactic-amcl ros-galactic-cartographer ros-galactic-cartographer-ros ros-galactic-cartographer-rviz
sudo apt --yes install ros-galactic-move-base ros-galactic-move-base-msgs
sudo apt --yes install ros-galactic-gmapping ros-galactic-navigation
#echo "Install ROS MoveIt! installation"
#sudo apt --yes install ros-galactic-moveit
# SMACH VIEWER HAS A BUG, SO NOT INSTALLING FOR NOW
#echo "Install additional SMACH packages"
#sudo apt install ros-galactic-smach-viewer
echo "Install RPLidar A1M8 packages"
sudo apt --yes install ros-galactic-rplidar-ros ros-galactic-rplidar-ros-dbgsym
#echo "Install Respeaker and speech recognition packages"
#sudo apt --yes install ros-galactic-respeaker-ros ros-galactic-ros-speech-recognition python3-pyaudio
echo "DONE WITH ADDITIONAL INSTALLATION OF ROS2 GALACTIC"
echo "###########################################"
echo ""

echo "###########################################"
echo "EXTRA PACKAGES OF ROS/ROS2"
echo ""
echo "Install ROS packages for Robotis Dynamixel actuators"
sudo apt --yes install ros-noetic-dynamixel-sdk ros-noetic-dynamixel-workbench
sudo apt --yes install ros-galactic-dynamixel-sdk
echo "DONE WITH EXTRA PACKAGES OF ROS/ROS2"
echo "###########################################"
echo ""

echo "###########################################"
echo "INSTALLATION OF INTEL D435i"
sudo apt --yes install ros-noetic-realsense2-camera ros-noetic-realsense2-description
sudo apt --yes install ros-galactic-realsense2-camera ros-galactic-realsense2-description

# # see https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
echo "INSTALL INTEL D435i"
echo "Register the server's public key"
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
echo "Add the server to the list of repositories"
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u
echo "Remove old records in case of upgrading"
sudo rm -f /etc/apt/sources.list.d/realsense-public.list
echo "Update"
sudo apt --yes update
echo "Install D435i packages"
sudo apt --yes install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
#######
# NOTE: Use realsense-viewer to install the recommended firmware.
#######
echo "DONE WITH INSTALLATION OF INTEL D435i"
echo "###########################################"
echo ""


