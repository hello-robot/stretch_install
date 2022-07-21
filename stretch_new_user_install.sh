#!/bin/bash
set -e

#This script setups a new user account with Stretch software
#If the software has already been installed then it will update it to the latest versions

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
mkdir -p ~/repos
mkdir -p ~/stretch_user
mkdir -p ~/stretch_user/log
mkdir -p ~/stretch_user/debug
mkdir -p ~/stretch_user/maps
mkdir -p ~/stretch_user/models

dd="$HOME/repos/stretch_install"
if [ -d "$dd" ]; then
    echo "Directory $dd exists. Performing git pull."
    cd $dd
    git pull
    if [ $? -ne 0 ]; then
        echo "Installation failed. Exiting"
        exit 1
    fi
else
    echo "Cloning stretch_install repository into standard location."
    cd $HOME/repos/
    git clone https://github.com/hello-robot/stretch_install.git
    if [ $? -ne 0 ]; then
        echo "Installation failed. Exiting"
        exit 1
    fi
fi
echo "###########################################"

dd="$HOME/repos/usb_4_mic_array"
if [ -d "$dd" ]; then
    echo "Directory $dd exists."
    cd $dd
    git pull
    if [ $? -ne 0 ]; then
        echo "Installation failed. Exiting"
        exit 1
    fi
else
    echo "Cloning stretch_install/respeaker repositories into standard location."
    cd $HOME/repos/
    git clone https://github.com/respeaker/usb_4_mic_array.git
    if [ $? -ne 0 ]; then
        echo "Installation failed. Exiting"
        exit 1
    fi
fi
echo "###########################################"

dd="$HOME/stretch_user/stretch_deep_perception_models"
if [ -d "$dd" ]; then
    echo "Directory $dd exists."
    cd $dd
    git pull
    if [ $? -ne 0 ]; then
        echo "Installation failed. Exiting"
        exit 1
    fi
else
    echo "Cloning stretch_deep_perception_models into standard location."
    cd $HOME/stretch_user/
    git clone https://github.com/hello-robot/stretch_deep_perception_models
    if [ $? -ne 0 ]; then
        echo "Installation failed. Exiting"
        exit 1
    fi
fi
echo "###########################################"


echo "Setting up local copy of robot factory data if not already there"
#Take care to not copy over existing user data if doing update

if [ "$UPDATING" = true ]; then
     echo "stretch_user data present: not updating"
else
    echo "Setting up stretch user $HELLO_FLEET_ID"
    cp -rf /etc/hello-robot/$HELLO_FLEET_ID $HOME/stretch_user
    chmod a-w $HOME/stretch_user/$HELLO_FLEET_ID/udev/*.rules
    chmod a-w $HOME/stretch_user/$HELLO_FLEET_ID/calibration_steppers/*.yaml
fi
echo "###########################################"

# set up the robot's code to run automatically on boot
echo "Setting up this machine to start the robot's code automatically on boot..."
mkdir -p ~/.config/autostart
cp ~/repos/stretch_install/factory/hello_robot_audio.desktop ~/.config/autostart/
cp ~/repos/stretch_install/factory/hello_robot_xbox_teleop.desktop ~/.config/autostart/
cp ~/repos/stretch_install/factory/hello_robot_lrf_off.desktop ~/.config/autostart/
cp ~/repos/stretch_install/factory/hello_robot_pimu_ping.desktop ~/.config/autostart/
echo "Done."
echo "###########################################"


echo "Updating media assets"
sudo cp $HOME/repos/stretch_install/images/stretch_about.png /etc/hello-robot

echo "Adding user hello to the dialout group to access Arduino..."
sudo adduser $USER dialout
echo "This is unlikely to take effect until you log out and log back in."
echo "Done."
echo "###########################################"

echo "Adding user hello to the plugdev group to access serial..."
sudo adduser $USER plugdev
echo "This is unlikely to take effect until you log out and log back in."
echo "Done."
echo "###########################################"

echo "Adding user hello to the input group to access input devices (e.g. gamepad)..."
sudo adduser $USER input
echo "This is unlikely to take effect until you log out and log back in."
echo "Done."
echo "###########################################"

echo "###########################################"
echo "INSTALLATION OF PIP PACKAGES"
echo ""
echo "Upgrade pip3"
python3 -m pip install --user --upgrade pip
echo "Install setuptools"
python2 -m pip install setuptools-scm==5.0.2
echo "###########################################"
echo "Install Stretch Body, Stretch Factory, and Stretch ToolShare  via pip (Python2)"
python2 -m pip install hello-robot-stretch-body
python2 -m pip install hello-robot-stretch-body-tools
python2 -m pip install hello-robot-stretch-factory
python2 -m pip install hello-robot-stretch-tool-share
echo "###########################################"
echo "Install Pathlib"
python -m pip install pathlib
python3 -m pip install pathlib
echo "###########################################"
cd $HOME/repos/usb_4_mic_array/
echo " - Flashing Respeaker with 6 channel firmware"
sudo python2 dfu.py --download 6_channels_firmware.bin
echo "###########################################"
echo ""
echo "DONE WITH PIP PACKAGES"
echo "###########################################"
echo ""

~/repos/stretch_install/factory/stretch_create_catkin_workspace.sh

echo "############ Finished Stretch New User Install ############"
