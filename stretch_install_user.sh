#!/bin/bash

echo "Install Python"
sudo apt --yes install python 
echo "Install Pip"
sudo apt --yes install python-pip
echo "Install Git"
sudo apt --yes install git

#Get fleet ID
. /etc/hello-robot/hello-robot.conf
echo "Setting up new user for robot $HELLO_FLEET_ID"
mkdir ~/repos
mkdir ~/stretch_user
mkdir ~/stretch_user/log

echo "Cloning stretch_install repository into standard location."
cd ~/repos/
git clone https://github.com/hello-robot/stretch_install.git

echo "Setting up local copy of robot factory data"
cp -rf /etc/hello-robot/$HELLO_FLEET_ID ~/stretch_user
chmod a-w ~/stretch_user/$HELLO_FLEET_ID/stretch_re1_factory_params.yaml
chmod a-w ~/stretch_user/$HELLO_FLEET_ID/udev/*.rules
chmod a-w ~/stretch_user/$HELLO_FLEET_ID/calibration_steppers/*.yaml

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
echo "Install rpl via apt"
sudo apt --yes install rpl
echo "Install ipython3 via apt"
sudo apt --yes install ipython3
sudo apt --yes install python3-pip

echo "Install urdfpy via pip3"
pip3 install urdfpy

# update .bashrc to add body code directory to the Python path
echo "export HELLO_FLEET_PATH=${HOME}/stretch_user" >> ~/.bashrc
echo "export HELLO_FLEET_ID=${HELLO_FLEET_ID}">> ~/.bashrc
echo "export PATH=\${PATH}:~/.local/bin" >> ~/.bashrc
echo "export PYTHONPATH=\${PATH}:/opt/ros/melodic/lib/python2.7/dist-packages" >> ~/.bashrc
echo "source .bashrc"
source ~/.bashrc
echo "Done."
echo ""

# set up the robot's code to run automatically on boot
echo "Setting up this machine to start the robot's code automatically on boot..."
mkdir ~/.config/autostart
cp ~/repos/stretch_install/factory/hello_robot_audio.desktop ~/.config/autostart/
cp ~/repos/stretch_install/factory/hello_robot_xbox_teleop.desktop ~/.config/autostart/
cp ~/repos/stretch_install/factory/hello_robot_lrf_off.desktop ~/.config/autostart/
echo "Done."
echo ""


#Install tools for managing hw
sudo apt-get install lm-sensors

echo "Making and installing nvme"
cd ~/
git clone https://github.com/linux-nvme/nvme-cli.git
cd nvme-cli/
make
sudo make install
cd ..
rm -rf nvme-cli

# add user to the dialout group to access devices
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


# upgrade to the latest versions of Ubuntu packages
echo "Upgrading Ubuntu packages to the latest versions..."
sudo apt --yes update
sudo apt --yes upgrade
echo "Done."
echo ""

# install additional packages
echo "INSTALL ADDITIONAL PACKAGES WITH APT"
echo "Install Emacs packages"
sudo apt --yes install emacs yaml-mode
echo "Install nettools"
sudo apt --yes install net-tools
echo "Install git and wget"
sudo apt --yes install git wget
echo "Install Python packages"
sudo apt --yes install ipython python-pip python-serial
echo "Install packages to work with URDFs"
sudo apt --yes install liburdfdom-tools meshlab

echo "INSTALL ADDITIONAL PACKAGES WITH PIP"
echo "Install pip Python profiler output viewer (SnakeViz)"
python -m pip install --user snakeviz
echo "Install pip Python packages for Respeaker and speech recognition"
python -m pip install --user pyusb SpeechRecognition pixel-ring click
echo "Install pip Python CMA-ES optimization package"
python -m pip install --user cma
echo "Install latest version of Pythin OpenCV via pip"
python -m pip install --user opencv-contrib-python
echo "Install numba and colorama numba dependency via pip"
python -m pip install --user numba colorama
echo "Install numba via pip"
python -m pip install --user numba
echo "Install scikit-image via pip"
python -m pip install --user scikit-image
echo "Install cheese for camera testing"
sudo apt --yes install cheese
echo "Install Open3D."
echo "WARNING: THIS MAY BE INCOMPATIBLE WITH ROSBRIDGE AND WEB TELEOPERATION DUE TO TORNADO PACKAGE INSTALLATION"
python -m pip install --user open3d
echo "Install SciPy and related software with pip for recent versions" 
python -m pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose
echo "DONE INSTALLING ADDITIONAL PACKAGES WITH PIP"
echo ""

# install D435i packages
echo "INSTALL INTEL D435i"
echo "Install D435i Python wrapper"
python -m pip install --user pyrealsense2
echo "DONE INSTALLING INTEL D435i"
echo ""
