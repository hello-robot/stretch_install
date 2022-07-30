#!/bin/bash
set -e

if [ "$HELLO_FLEET_ID" ]; then
    UPDATING=true
    echo "###########################################"
    echo "UPDATING USER SOFTWARE"
    echo "###########################################"
else
    UPDATING=false
    . /etc/hello-robot/hello-robot.conf
    echo "###########################################"
    echo "NEW INSTALLATION OF USER SOFTWARE"
    echo "###########################################"
    echo "Update .bashrc"
    echo "export HELLO_FLEET_PATH=${HOME}/stretch_user" >> ~/.bashrc
    echo "export HELLO_FLEET_ID=${HELLO_FLEET_ID}">> ~/.bashrc
    echo "export PATH=\${PATH}:~/.local/bin" >> ~/.bashrc
    echo "Source .bashrc"
    source ~/.bashrc
    export HELLO_FLEET_PATH=${HOME}/stretch_user
    export HELLO_FLEET_ID=${HELLO_FLEET_ID}
fi

echo "Creating repos and stretch_user directories..."
mkdir -p ~/repos
mkdir -p ~/stretch_user
mkdir -p ~/stretch_user/log
mkdir -p ~/stretch_user/debug
mkdir -p ~/stretch_user/maps
mkdir -p ~/stretch_user/models

echo "Cloning Stretch deep perception models..."
cd ~/stretch_user
git clone https://github.com/hello-robot/stretch_deep_perception_models > /dev/null
cd stretch_deep_perception_models
git pull > /dev/null

echo "Setting up user copy of robot factory data (if not already there)..."
if [ "$UPDATING" = true ]; then
    echo "~/stretch_user/$HELLO_FLEET_ID data present: not updating"
else
    cp -rf /etc/hello-robot/$HELLO_FLEET_ID $HOME/stretch_user
    chmod a-w $HOME/stretch_user/$HELLO_FLEET_ID/udev/*.rules
    chmod a-w $HOME/stretch_user/$HELLO_FLEET_ID/calibration_steppers/*.yaml
fi

echo "Setting up this user to start the robot's code automatically on boot..."
mkdir -p ~/.config/autostart
cp ~/stretch_install/factory/hello_robot_audio.desktop ~/.config/autostart/
cp ~/stretch_install/factory/hello_robot_xbox_teleop.desktop ~/.config/autostart/
cp ~/stretch_install/factory/hello_robot_lrf_off.desktop ~/.config/autostart/
cp ~/stretch_install/factory/hello_robot_pimu_ping.desktop ~/.config/autostart/

echo "Updating media assets..."
sudo cp $HOME/stretch_install/images/stretch_about.png /etc/hello-robot

echo "Adding user hello to the dialout group to access Arduino..."
sudo adduser $USER dialout
echo "Adding user hello to the plugdev group to access serial..."
sudo adduser $USER plugdev
echo "Adding user hello to the input group to access input devices (e.g. gamepad)..."
sudo adduser $USER input
echo ""

echo "###########################################"
echo "INSTALLATION OF USER LEVEL PIP PACKAGES"
echo "###########################################"
echo "Upgrade pip3"
python3 -m pip -q install --user --upgrade pip
echo "Install setuptools"
python2 -m pip -q install setuptools-scm==5.0.2
echo "Install Stretch Body"
python2 -m pip -q install hello-robot-stretch-body
echo "Install Stretch Body Tools"
python2 -m pip -q install hello-robot-stretch-body-tools
echo "Install Stretch Factory"
python2 -m pip -q install hello-robot-stretch-factory
echo "Install Stretch Tool Share"
python2 -m pip -q install hello-robot-stretch-tool-share
echo "###########################################"
echo "DONE WITH INSTALLATION OF USER LEVEL PIP PACKAGES"
echo "###########################################"
echo ""

~/stretch_install/factory/stretch_create_catkin_workspace.sh
