#!/bin/bash
set -e

echo ""
echo "WARNING: Run this installation for fresh Ubuntu installs only."
echo ""
#####################################################
DIR=`pwd`
echo -n "Enter fleet id xxxx for stretch-re1-xxxx> "
read id
pre="stretch-re1-"
HELLO_FLEET_ID="$pre$id"

read -p "HELLO_FLEET_ID will be $HELLO_FLEET_ID. Proceed with installation (y/n)? " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Confirmation failed. Will not proceed with installation."
    exit 1
fi

echo ""
echo "Setting up /etc/hello-robot directory..."
echo "HELLO_FLEET_ID=$HELLO_FLEET_ID">>hello-robot.conf
sudo mkdir /etc/hello-robot
sudo mv hello-robot.conf /etc/hello-robot
sudo cp $DIR/../images/stretch_about.png /etc/hello-robot/

echo ""
echo "Fetching robot's calibration data locally from $HOME/$HELLO_FLEET_ID directory..."
sudo cp -rf ~/$HELLO_FLEET_ID /etc/hello-robot
rm -rf ~/$HELLO_FLEET_ID

echo ""
echo "Setting up UDEV rules..."
sudo cp /etc/hello-robot/$HELLO_FLEET_ID/udev/*.rules /etc/udev/rules.d
sudo udevadm control --reload

echo ""
echo "Allow shutdown without password..."
sudo cp $DIR/hello_sudoers /etc/sudoers.d/

echo ""
echo "Setting up startup scripts..."
mkdir -p ~/.local/bin
sudo cp $DIR/xbox_dongle_init.py ~/.local/bin/
sudo cp $DIR/hello_robot_audio.sh /usr/bin/
sudo cp $DIR/hello_robot_lrf_off.py /usr/bin/
sudo cp $DIR/hello_robot_pimu_ping.py /usr/bin/
sudo cp $DIR/hello_robot_pimu_ping.sh /usr/bin/
sudo cp $DIR/hello_robot_xbox_teleop.sh /usr/bin/

echo ""
echo "Done with new robot setup."
echo ""

