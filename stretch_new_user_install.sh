#!/bin/bash
set -e

REDIRECT_LOGFILE="$HOME/stretch_user/log/stretch_new_user_install.`date '+%Y%m%d%H%M'`_redirected.txt"

source /etc/os-release
factory_osdir="$VERSION_ID"
if [[ ! $factory_osdir =~ ^(18.04|20.04)$ ]]; then
    echo "Could not identify OS. Please contact Hello Robot Support."
    exit 1
fi

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
    echo "Update ~/.bashrc dotfile..."
    echo "" >> ~/.bashrc
    echo "######################" >> ~/.bashrc
    echo "# STRETCH BASHRC SETUP" >> ~/.bashrc
    echo "######################" >> ~/.bashrc
    echo "export HELLO_FLEET_PATH=${HOME}/stretch_user" >> ~/.bashrc
    echo "export HELLO_FLEET_ID=${HELLO_FLEET_ID}">> ~/.bashrc
    echo "export PATH=\${PATH}:~/.local/bin" >> ~/.bashrc
    echo "export LRS_LOG_LEVEL=None #Debug" >> ~/.bashrc
    if [[ $factory_osdir = "18.04" ]]; then
        echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    elif [[ $factory_osdir = "20.04" ]]; then
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
        echo "#source /opt/ros/galactic/setup.bash" >> ~/.bashrc
    fi
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
if [ ! -d "$HOME/stretch_user/stretch_deep_perception_models" ]; then
    git clone https://github.com/hello-robot/stretch_deep_perception_models >> $REDIRECT_LOGFILE
fi
cd stretch_deep_perception_models
git pull >> $REDIRECT_LOGFILE

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
cp ~/stretch_install/factory/$factory_osdir/hello_robot_audio.desktop ~/.config/autostart/
cp ~/stretch_install/factory/$factory_osdir/hello_robot_xbox_teleop.desktop ~/.config/autostart/
cp ~/stretch_install/factory/$factory_osdir/hello_robot_lrf_off.desktop ~/.config/autostart/
cp ~/stretch_install/factory/$factory_osdir/hello_robot_pimu_ping.desktop ~/.config/autostart/

echo "Updating media assets..."
sudo cp $HOME/stretch_install/factory/$factory_osdir/stretch_about.png /etc/hello-robot

echo "Adding user to the dialout group to access Arduino..."
sudo adduser $USER dialout >> $REDIRECT_LOGFILE
echo "Adding user to the plugdev group to access serial..."
sudo adduser $USER plugdev >> $REDIRECT_LOGFILE
echo "Adding user to the input group to access input devices (e.g. gamepad)..."
sudo adduser $USER input >> $REDIRECT_LOGFILE
echo ""

if [[ $factory_osdir = "18.04" ]]; then
    echo "###########################################"
    echo "INSTALLATION OF USER LEVEL PIP2 PACKAGES"
    echo "###########################################"
    echo "Upgrade pip3"
    python3 -m pip -q install --no-warn-script-location --user --upgrade pip
    echo "Install setuptools"
    python2 -m pip -q install setuptools-scm==5.0.2
    echo "Install Stretch Body (this might take a while)"
    python2 -m pip -q install hello-robot-stretch-body
    echo "Install Stretch Body Tools"
    python2 -m pip -q install hello-robot-stretch-body-tools
    echo "Install Stretch Factory"
    python2 -m pip -q install hello-robot-stretch-factory
    echo "Install Stretch Tool Share"
    python2 -m pip -q install hello-robot-stretch-tool-share
    echo "Install opencv-python-inference-engine"
    python3 -m pip -q install --no-warn-script-location opencv-python-inference-engine
    echo ""
elif [[ $factory_osdir = "20.04" ]]; then
    echo "###########################################"
    echo "INSTALLATION OF USER LEVEL PIP3 PACKAGES"
    echo "###########################################"
    echo "Upgrade pip3"
    python3 -m pip -q install --no-warn-script-location --user --upgrade pip
    echo "Install Stretch Body"
    python3 -m pip -q install --no-warn-script-location hello-robot-stretch-body
    echo "Install Stretch Body Tools"
    python3 -m pip -q install --no-warn-script-location hello-robot-stretch-body-tools
    echo "Install Stretch Factory"
    python3 -m pip -q install --no-warn-script-location hello-robot-stretch-factory
    echo "Install Stretch Tool Share"
    python3 -m pip -q install --no-warn-script-location hello-robot-stretch-tool-share
    echo "Upgrade prompt_toolkit"
    python3 -m pip -q install --no-warn-script-location -U prompt_toolkit
    echo ""
fi

if [[ $factory_osdir = "18.04" ]]; then
    ~/stretch_install/factory/$factory_osdir/stretch_create_catkin_workspace.sh "$HOME/catkin_ws"
elif [[ $factory_osdir = "20.04" ]]; then
    ~/stretch_install/factory/$factory_osdir/stretch_create_catkin_workspace.sh "$HOME/catkin_ws"
    echo ""
    ~/stretch_install/factory/$factory_osdir/stretch_create_ament_workspace.sh "$HOME/ament_ws"
fi
