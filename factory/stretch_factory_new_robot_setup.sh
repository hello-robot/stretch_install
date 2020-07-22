#!/bin/bash

echo "Run this installation for fresh Ubuntu installs only."
#####################################################
DIR=`pwd`

PS3='Please enter batch name: '
options=("guthrie" "hank" "irma" "joplin")
select opt in "${options[@]}"
do
    case $opt in
        "guthrie")
            echo "you chose  guthrie"
            HELLO_BATCH_ID="guthrie"
            break
            ;;
        "hank")
            echo "you chose  hank"
            break
            ;;
        "irma")
            echo "you chose  irma"
            break
            ;;
        "joplin")
            echo "you chose joplin"
            break
            ;;
        *) echo "invalid option $REPLY";;
    esac
done


echo -n "Enter fleet id xxxx for stretch-re1-xxxx >"
read id
pre="stretch-re1-"
HELLO_FLEET_ID="$pre$id"

read -p "HELLO_FLEET_ID of $HELLO_FLEET_ID . HELLO_BATCH_ID of $HELLO_BATCH_ID. Proceed with installation (y/n)?" -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    exit 1
fi

echo "HELLO_FLEET_ID=$HELLO_FLEET_ID">>hello-robot.conf
echo "HELLO_BATCH_ID=$HELLO_BATCH_ID">>hello-robot.conf
sudo mkdir /etc/hello-robot
sudo mv hello-robot.conf /etc/hello-robot
sudo cp $DIR/../images/stretch_about.png /etc/hello-robot

cd ~/
git clone https://github.com/hello-robot/stretch_fleet.git
sudo cp -rf ~/stretch_fleet/robots/$HELLO_FLEET_ID /etc/hello-robot

echo "Setting up UDEV rules..."
sudo cp ~/stretch_fleet/robots/$HELLO_FLEET_ID/udev/*.rules /etc/udev/rules.d
sudo udevadm control --reload

rm -rf stretch_fleet

#Allow shutdown without password


#Startup scripts
sudo cp $DIR/hello_robot_audio.sh /usr/bin
sudo cp $DIR/hello_robot_lrf_off.py /usr/bin
sudo cp $DIR/hello_robot_xbox_teleop.sh /usr/bin
sudo cp $DIR/hello_sudoers /etc/sudoers.d/

echo "Done."
echo ""



