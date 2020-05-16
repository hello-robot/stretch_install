#!/bin/bash

#Current issues
# python -m pip install --user numba
# catkin  - csm


echo "Setting up Stretch new user account"


echo "###########################################"
echo "INSTALLATION OF STRETCH_BODY"

#Get fleet ID
. /etc/hello-robot/hello-robot.conf
echo "Setting up new user for robot $HELLO_FLEET_ID"
mkdir ~/repos
mkdir ~/stretch_user
mkdir ~/stretch_user/log
mkdir ~/stretch_user/debug
mkdir ~/stretch_user/deep_learning_models
mkdir ~/stretch_user/head_scans
mkdir ~/stretch_user/maps

echo "Cloning stretch_install repository into standard location."
cd ~/repos/
git clone -b ros_install https://github.com/hello-robot/stretch_install.git

echo "Setting up local copy of robot factory data"
cp -rf /etc/hello-robot/$HELLO_FLEET_ID ~/stretch_user
chmod a-w ~/stretch_user/$HELLO_FLEET_ID/stretch_re1_factory_params.yaml
chmod a-w ~/stretch_user/$HELLO_FLEET_ID/udev/*.rules
chmod a-w ~/stretch_user/$HELLO_FLEET_ID/calibration_steppers/*.yaml

# set up the robot's code to run automatically on boot
echo "Setting up this machine to start the robot's code automatically on boot..."
mkdir ~/.config/autostart
cp ~/repos/stretch_install/factory/hello_robot_audio.desktop ~/.config/autostart/
cp ~/repos/stretch_install/factory/hello_robot_xbox_teleop.desktop ~/.config/autostart/
cp ~/repos/stretch_install/factory/hello_robot_lrf_off.desktop ~/.config/autostart/
echo "Done."
echo ""

#echo "Removing stretch_install"
#rm -rf stretch_install


echo "Install stretch_body via pip"
pip install hello-robot-stretch-body
pip install hello-robot-stretch-body-tools
pip3 install hello-robot-stretch-body-tools-py3

#Other packages required by stretch_body
echo "Install PyYaml via pip"
python -m pip install PyYaml
echo "Install inputs via pip"
python -m pip install inputs
echo "Install drawnow via pip"
python -m pip install drawnow
echo "Install rplidar via pip"
python -m pip install rplidar

echo "Setting up .bashrc"
echo "export EDITOR='emacs -nw'" >> ~/.bashrc
echo "export HELLO_FLEET_PATH=${HOME}/stretch_user" >> ~/.bashrc
echo "export HELLO_FLEET_ID=${HELLO_FLEET_ID}">> ~/.bashrc
echo "export PATH=\${PATH}:~/.local/bin" >> ~/.bashrc
echo "source .bashrc"
source ~/.bashrc
echo "Done."
echo ""


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

echo "DONE WITH ADDITIONAL INSTALLATION STRETCH_BODY"
echo "###########################################"
echo ""


echo "###########################################"
echo "INSTALLATION OF ADDITIONAL PIP PACKAGES"
echo "Install pip Python profiler output viewer (SnakeViz)"
python -m pip install --user snakeviz
echo "Install pip Python packages for Respeaker and speech recognition"
python -m pip install --user pyusb SpeechRecognition pixel-ring click
echo "Install pip Python CMA-ES optimization package"
python -m pip install --user cma
echo "Install latest version of Pythin OpenCV via pip"
python -m pip install --user opencv-contrib-python
echo "Install colorama via pip"
python -m pip install --user colorama
echo "Install numba via pip using pip_constraints.txt file to handle llvmlite version issue"
python -m pip install --user -c pip_constraints.txt numba
echo "Install scikit-image via pip"
python -m pip install --user scikit-image
echo "Install Open3D."
echo "WARNING: THIS MAY BE INCOMPATIBLE WITH ROSBRIDGE AND WEB TELEOPERATION DUE TO TORNADO PACKAGE INSTALLATION"
python -m pip install --user open3d
echo "Install SciPy and related software with pip for recent versions" 
python -m pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose
# install D435i packages
echo "INSTALL INTEL D435i"
echo "Install D435i Python wrapper"
python -m pip install --user pyrealsense2
echo "DONE INSTALLING INTEL D435i"
echo ""
echo "DONE WITH ADDITIONAL ADDITIONAL PIP PACKAGES"
echo "###########################################"
echo ""

echo "###########################################"
echo "INSTALLATION OF PYTHON 3 PIP PACKAGES"
echo ""
echo "Upgrade pip3"
python3 -m pip install --user --upgrade pip
echo "Install urdfpy for Python 3 via pip3"
python3 -m pip install --user urdfpy
echo "Install Numba for Python 3 via pip3"
python3 -m pip install --user numba
echo "Install Python3 OpenCV with deep neural network (DNN) support via pip3"
python3 -m pip install --user opencv-python-inference-engine
echo "Install rospkg for Python 3 via pip3"
python3 -m pip install --user rospkg
echo "Install scipy for Python 3 via pip3"
python3 -m pip install --user scipy
echo ""
echo "DONE WITH PYTHON 3 PIP PACKAGES"
echo "###########################################"
echo ""


echo "###########################################"
echo "INSTALLATION OF ROS WORKSAPCE"
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
# update .bashrc
echo "UPDATE .bashrc for ROS"
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "add catkin development workspace overlay to .bashrc"
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source .bashrc"
source ~/.bashrc
echo "DONE UPDATING .bashrc"
echo ""

# install ros_numpy from github
echo "INSTALL ROS_NUMPY FROM GITHUB"
cd ~/catkin_ws/src/
echo "Cloning the ros_numpy github repository."
git clone https://github.com/eric-wieser/ros_numpy.git
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
echo "Clone the github repository"
git clone https://github.com/hello-robot/stretch_ros.git
cd ~/catkin_ws/
echo "Make the ROS repository"
catkin_make
echo "Make sure new ROS package is indexed"
rospack profile
echo "Install ROS packages. This is important for using Python modules."
catkin_make install
echo ""

# compile Cython code
echo "Compiling Cython code"
cd ~/catkin_ws/src/stretch_ros/stretch_body/nodes
./compile_cython_code.sh
echo "Done"

# create directories for development
cd ~/catkin_ws/src/stretch_ros/stretch_body/nodes
mkdir head_scans
mkdir max_height_images
mkdir test_scans
mkdir objects_to_grasp

# install scan_tools for laser range finder odometry
echo "INSTALL SCAN_TOOLS FROM GITHUB"
cd ~/catkin_ws/
echo "Cloning the csm github repository."
git clone https://github.com/AndreaCensi/csm
echo "Handle csm dependencies."
cd ~/catkin_ws/
rosdep install --from-paths src --ignore-src -r -y
echo "Make csm."
cd ~/catkin_ws/csm/
cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/local .
make
echo "Install csm."
sudo make install
echo "Cloning the scan_tools github repository."
cd ~/catkin_ws/src/
git clone https://github.com/ccny-ros-pkg/scan_tools.git
echo "Make scan_tools."
cd ~/catkin_ws/
catkin_make
echo "Make sure new ROS packages are indexed"
rospack profile
echo ""

echo "DONE WITH ADDITIONAL ADDITIONAL PIP PACKAGES"
echo "###########################################"
echo ""

echo "The IP address for this machine follows:"
curl ifconfig.me
echo ""
echo "Make it a static IP and then use it for SSH and VNC."
echo "Done!"

echo "DONE WITH STRETCH_USER_INSTALL"
echo "###########################################"
echo ""
