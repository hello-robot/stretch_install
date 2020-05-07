#!/bin/bash

#####################################################
DIR=`pwd`

echo -n "Enter fleet id xxxx for stretch-re1-xxxx >"
read id
pre="stretch-re1-"
HELLO_FLEET_ID="$pre$id"

read -p "HELLO_FLEET_ID of $HELLO_FLEET_ID . Proceed with installation (y/n)?" -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    exit 1
fi

echo "HELLO_FLEET_ID=$HELLO_FLEET_ID">>hello-robot.conf
sudo mkdir /etc/hello-robot
sudo mv hello-robot.conf /etc/hello-robot


cd ~/
git clone https://github.com/hello-robot/stretch_fleet.git
sudo cp -rf ~/stretch_fleet/$HELLO_FLEET_ID /etc/hello-robot
rm -rf stretch_fleet

#Allow shutdown without password


#Startup scripts
sudo cp $DIR/hello_robot_audio.sh /usr/bin
sudo cp $DIR/hello_robot_lrf_off.py /usr/bin
sudo cp $DIR/hello_robot_xbox_teleop.sh /usr/bin
sudo cp $DIR/hello_sudoers /etc/sudoers.d/
#Store this in etc so can be copied to/from .config/autostart by user script to enable/disable it autostarting
#sudo cp $DIR/hello_robot_xbox_teleop.desktop /etc/hello-robot
echo "Done."
echo ""



