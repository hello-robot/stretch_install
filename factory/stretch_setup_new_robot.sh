#!/bin/bash
set -e

echo ""
echo "WARNING: Running a FACTORY install. This is only meant to be run at Hello Robot HQ."
echo "WARNING: Run this installation for fresh Ubuntu installs only."
#####################################################
echo -n "Enter fleet id xxxx for stretch-re1-xxxx> "
read id
if [[ ! $id =~ ^[0-9]{4}$ ]]; then
    echo "Input should be four digits. Exiting."
    exit 1
fi
pre="stretch-re1-"
HELLO_FLEET_ID="$pre$id"

read -p "HELLO_FLEET_ID will be $HELLO_FLEET_ID. Proceed with installation (y/n)? " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Confirmation failed. Will not proceed with installation."
    exit 1
fi

echo ""
DIR=`pwd`
echo "Checking Stretch Install cloned to right place..."
if [[ ! -d "$HOME/stretch_install" ]]; then
    echo "Expecting Stretch Install to be in home folder. Exiting."
    exit 1
fi

echo "Waiting to get online..."
while ! timeout 0.2 ping -c 1 -n google.com &> /dev/null
do
    sleep 1
    printf "%c" "."
done

echo "Checking install repo is up-to-date..."
git remote update > /dev/null
ATU="@{u}"
UPSTREAM=${1:-$ATU}
LOCAL=$(git rev-parse @)
REMOTE=$(git rev-parse "$UPSTREAM")
if [ ! $LOCAL = $REMOTE ]; then
    echo "Repo not up-to-date. Please perform a 'git pull'. Exiting."
    exit 1
fi

echo "Waiting for apt lock..."
while sudo fuser /var/{lib/{dpkg,apt/lists},cache/apt/archives}/lock >/dev/null 2>&1; do
    sleep 1
    printf "%c" "."
done

echo "Setting up /etc/hello-robot directory..."
echo "HELLO_FLEET_ID=$HELLO_FLEET_ID">>hello-robot.conf
sudo mkdir /etc/hello-robot
sudo mv hello-robot.conf /etc/hello-robot
sudo cp $DIR/../images/stretch_about.png /etc/hello-robot/

echo "Fetching robot's calibration data from Github..."
cd ~/
git config --global credential.helper store
git clone https://github.com/hello-robot/stretch_fleet.git
sudo cp -rf ~/stretch_fleet/robots/$HELLO_FLEET_ID /etc/hello-robot/
rm -rf stretch_fleet

echo "Setting up UDEV rules..."
sudo cp /etc/hello-robot/$HELLO_FLEET_ID/udev/*.rules /etc/udev/rules.d
sudo udevadm control --reload

echo "Allow shutdown without password..."
sudo cp $DIR/hello_sudoers /etc/sudoers.d/

echo "Setting up startup scripts..."
mkdir -p ~/.local/bin
sudo cp $DIR/xbox_dongle_init.py ~/.local/bin/
sudo cp $DIR/hello_robot_audio.sh /usr/bin/
sudo cp $DIR/hello_robot_lrf_off.py /usr/bin/
sudo cp $DIR/hello_robot_pimu_ping.py /usr/bin/
sudo cp $DIR/hello_robot_pimu_ping.sh /usr/bin/
sudo cp $DIR/hello_robot_xbox_teleop.sh /usr/bin/
