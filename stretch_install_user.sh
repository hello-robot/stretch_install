#!/bin/bash

#Get fleet ID

if [ "$HELLO_FLEET_ID" ]; then
    echo "###########################################"
    echo "UPDATING USER SOFTWARE"
    UPDATING=true
else
    UPDATING=false
    . /etc/hello-robot/hello-robot.conf
    echo "###########################################"
    echo "NEW INSTALLATION OF USER SOFTWARE"
    echo "UPDATE .bashrc FOR STRETCH_BODY"
    echo "export HELLO_FLEET_PATH=${HOME}/stretch_user" >> ~/.bashrc
    echo "export HELLO_FLEET_ID=${HELLO_FLEET_ID}">> ~/.bashrc
    echo "export PATH=\${PATH}:~/.local/bin" >> ~/.bashrc
    echo "source .bashrc"
    source ~/.bashrc
    echo "Done."
    echo ""
    export HELLO_FLEET_PATH=${HOME}/stretch_user
    export HELLO_FLEET_ID=${HELLO_FLEET_ID}
fi

echo "###########################################"
echo "SETUP OF STRETCH_BODY"

echo "Setting up new user for robot $HELLO_FLEET_ID"
mkdir ~/repos
mkdir ~/stretch_user
mkdir ~/stretch_user/log
mkdir ~/stretch_user/debug
mkdir ~/stretch_user/maps
mkdir ~/stretch_user/models

echo "Cloning stretch_install repository into standard location."
cd ~/repos/
git clone https://github.com/hello-robot/stretch_install.git
cd stretch_install
git pull

echo "Cloning stretch_install/respeaker repositories into standard location."
cd ~/repos/
git clone https://github.com/respeaker/usb_4_mic_array.git
cd usb_4_mic_array
git pull


echo "Cloning stretch_deep_perception_models into standard location."
cd ~/stretch_user
git clone https://github.com/hello-robot/stretch_deep_perception_models
cd stretch_deep_perception_models
git pull


echo "Setting up local copy of robot factory data if not already there"
#Take care to not copy over existing user data if doing update

if [ "$UPDATING" = true ]; then
     echo "stretch_user data present: not updating"
else
    echo "Setting up stretch user $HELLO_FLEET_ID"
    cp -rf /etc/hello-robot/$HELLO_FLEET_ID ~/stretch_user
    chmod a-w ~/stretch_user/$HELLO_FLEET_ID/udev/*.rules
    chmod a-w ~/stretch_user/$HELLO_FLEET_ID/calibration_steppers/*.yaml
fi

# set up the robot's code to run automatically on boot
echo "Setting up this machine to start the robot's code automatically on boot..."
mkdir -p ~/.config/autostart
cp ~/repos/stretch_install/factory/hello_robot_audio.desktop ~/.config/autostart/
cp ~/repos/stretch_install/factory/hello_robot_xbox_teleop.desktop ~/.config/autostart/
cp ~/repos/stretch_install/factory/hello_robot_lrf_off.desktop ~/.config/autostart/
echo "Done."
echo ""

echo "Updating media assets"
sudo cp ~/repos/stretch_install/images/stretch_about.png /etc/hello-robot

echo "Adding user hello to the dialout group to access Arduino..."
sudo adduser $USER dialout
echo "This is unlikely to take effect until you log out and log back in."
echo "Done."
echo ""

echo "Adding user hello to the plugdev group to access serial..."
sudo adduser $USER plugdev
echo "This is unlikely to take effect until you log out and log back in."
echo "Done."
echo ""

echo "Adding user hello to the input group to access input devices (e.g. gamepad)..."
sudo adduser $USER input
echo "This is unlikely to take effect until you log out and log back in."
echo "Done."
echo ""

echo "DONE WITH ADDITIONAL INSTALLATION STRETCH_BODY"
echo "###########################################"
echo ""


echo "###########################################"
echo "INSTALLATION OF PIP PACKAGES"
echo ""
echo "Upgrade pip3"
python3 -m pip install --user --upgrade pip

echo "Install stretch_body and stretch_factory via pip"
python3 -m pip install hello-robot-stretch-body
python3 -m pip install hello-robot-stretch-body-tools
python3 -m pip install hello-robot-stretch-factory
python3 -m pip uninstall -y opencv-contrib-python opencv-python-inference-engine
python3 -m pip install opencv-python-inference-engine

# TODO RE-ENABLED
# cd ~/repos/usb_4_mic_array/
# echo " - Flashing Respeaker with 6 channel firmware"
# sudo python2 dfu.py --download 6_channels_firmware.bin

echo ""
echo "DONE WITH PIP PACKAGES"
echo "###########################################"
echo ""


echo "###########################################"
echo "INSTALLATION OF ROS WORKSAPCE"
 # update .bashrc before using catkin tools
if [ "$UPDATING" = true ]; then
     echo "Updating: Not updating ROS in .bashrc"
else
    echo "UPDATE .bashrc for ROS"
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    echo "add catkin development workspace overlay to .bashrc"
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    echo "set log level for realsense camera"
    echo "export LRS_LOG_LEVEL=None #Debug" >> ~/.bashrc
    echo "source .bashrc"
    source ~/.bashrc
    source /opt/ros/noetic/setup.bash
    echo "DONE UPDATING .bashrc"
    echo ""
fi

# create the ROS workspace
# see http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
echo "Creating the ROS workspace..."
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "Source .bash file"
source ~/catkin_ws/devel/setup.bash
echo "Make sure new ROS package is indexed"
rospack profile
echo "Done."
echo ""

echo "Make ROS package"
cd ~/catkin_ws/
catkin_make
echo "Install ROS package"
catkin_make install
echo "Make sure new ROS package is indexed"
rospack profile
echo "DONE INSTALLING ROS_NUMPY FROM GITHUB"
echo ""

# clone the Hello Robot ROS repository
echo "Install the Hello Robot ROS repository"
cd ~/catkin_ws/src/

echo "Cloning stretch_ros repository"
git clone https://github.com/hello-robot/stretch_ros.git -b dev/noetic
cd stretch_ros
git pull

echo "Updating meshes in stretch_ros to this robot batch"
~/catkin_ws/src/stretch_ros/stretch_description/meshes/update_meshes.py

cd ~/catkin_ws/
echo "Make the ROS repository"
catkin_make
echo "Make sure new ROS package is indexed"
rospack profile
echo "Install ROS packages. This is important for using Python modules."
catkin_make install
echo ""

if [ "$UPDATING" = true ]; then
    echo "Not updating URDF"
else
    echo "Setup calibrated robot URDF"
    rosrun stretch_calibration update_uncalibrated_urdf.sh
    #This will grab the latest URDF and calibration files from ~/stretch_user
    #rosrun stretch_calibration update_with_most_recent_calibration.sh
    #Force to run interactive so $HELLO_FLEET_ID is found
    echo "This may fail if doing initial robot bringup. That is OK."
    bash -i ~/catkin_ws/src/stretch_ros/stretch_calibration/nodes/update_with_most_recent_calibration.sh
    echo "--Done--"
fi

# compile Cython code
echo "Compiling Cython code"
source ~/.bashrc
cd ~/catkin_ws/src/stretch_ros/stretch_funmap/src/stretch_funmap
./compile_cython_code.sh
echo "Done"

# install scan_tools for laser range finder odometry
echo "INSTALL SCAN_TOOLS FROM GITHUB"
cd ~/catkin_ws/
echo "Cloning the csm github repository."
git clone https://github.com/AndreaCensi/csm
cd csm
git pull

echo "Handle csm dependencies."
cd ~/catkin_ws/
rosdep update
rosdep install --from-paths src --ignore-src -r -y
echo "Make csm."
sudo apt --yes install libgsl0-dev
cd ~/catkin_ws/csm/
cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/local .
make
echo "Install csm."
sudo make install
echo "Cloning the scan_tools github repository."
cd ~/catkin_ws/src/
git clone https://github.com/ccny-ros-pkg/scan_tools.git
cd scan_tools
git pull

echo "Make scan_tools."
cd ~/catkin_ws/
catkin_make
echo "Make sure new ROS packages are indexed"
rospack profile
echo ""

echo "INSTALL SLAMTEC RPLIDAR ROS PACKAGE FROM GITHUB"
echo "Cloning the Slamtec rplidar github repository."
cd ~/catkin_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
echo "Make the Slamtec rplidar package."
cd ~/catkin_ws
catkin_make
echo "Make sure new ROS packages are indexed."
rospack profile
echo ""

echo "Initialize URDF and controller calibration parameters to generic uncalibrated defaults."
echo "Create uncalibrated URDF."
rosrun stretch_calibration update_uncalibrated_urdf.sh
rosrun stretch_description xacro_to_urdf.sh 
echo "Copy factory defaults for controller calibration parameters."
cp  `rospack find stretch_core`/config/controller_calibration_head_factory_default.yaml `rospack find stretch_core`/config/controller_calibration_head.yaml
echo "Make sure new ROS packages are indexed"
rospack profile
echo ""

echo "DONE WITH ROS WORKSPACE"
echo "###########################################"
echo ""

echo "DONE WITH STRETCH_USER_INSTALL"
echo "###########################################"
echo ""
