#!/bin/bash
set -o pipefail

source /etc/os-release
factory_osdir="$VERSION_ID"
if [[ ! $factory_osdir =~ ^(22.04)$ ]]; then
    echo "The web interface requires Ubuntu 22.04. Please install Ubuntu 22.04 and try again."
    exit 1
fi

WEB_INTERFACE_WSDIR="$HOME/web_interface_ws"
REDIRECT_LOGDIR="$HOME/stretch_user/log"
REDIRECT_LOGFILE="$REDIRECT_LOGDIR/stretch_web_interface_install.`date '+%Y%m%d%H%M'`_redirected.txt"

sudo echo "#############################################"
echo "STARTING WEB INTERFACE INSTALL"
echo "#############################################"

echo "Ensuring correct version of ROS2 is sourced..."
if [[ $ROS_DISTRO && ! $ROS_DISTRO = "humble" ]]; then
    echo "Cannot create workspace while a conflicting ROS2 version is sourced. ROS_DISTRO must be Humble. Exiting."
    exit 1
fi
source /opt/ros/humble/setup.bash

#################### Installing/Upgrading NodeJS #################### 




##################################################################### 

if [[ -d $WEB_INTERFACE_WSDIR ]]; then
    echo "You are about to delete and replace the existing web interface workspace. If you have any personal data in the workspace, please create a back up before proceeding."
    prompt_yes_no(){
    read -p "Do you want to continue? Press (y/n for yes/no): " x
    if [ $x = "n" ]; then
            echo "Exiting the script."
            exit 1
    elif [ $x = "y" ]; then
            echo "Continuing to create a new web interface workspace."
    else
        echo "Press 'y' for yes or 'n' for no."
        prompt_yes_no
    fi
    }
    prompt_yes_no
fi

echo "Deleting $WEB_INTERFACE_WSDIR if it already exists..."
sudo rm -rf $WEB_INTERFACE_WSDIR
echo "Creating the workspace directory..."
mkdir -p $HOME/web_interface_ws/src
echo "Cloning the workspace's packages..."
cd $HOME/web_interface_ws/src
vcs import --input ~/repos/stretch_install/factory/22.04/stretch_web_interface.repos &>> $REDIRECT_LOGFILE
echo "Installing package dependencies..."
pip install pyquaternion &>> $REDIRECT_LOGFILE
sudo apt-get install python3-pcl python3-pykdl screen -y &>> $REDIRECT_LOGFILE
rosdep install --from-paths . --ignore-src -y -r &>> $REDIRECT_LOGFILE
sudo npm install -g pm2 &&>> $REDIRECT_LOGFILE
cd /home/hello-robot/web_interface_ws/src/stretch_teleop_interface
npm install &&>> $REDIRECT_LOGFILE
npx playwright install &&>> $REDIRECT_LOGFILE
echo ""

echo "Installing mkcert..."
cd /home/hello-robot/web_interface_ws/src/stretch_teleop_interface/certificates
curl -JLO "https://dl.filippo.io/mkcert/latest?for=linux/amd64" &&>> $REDIRECT_LOGFILE
chmod +x mkcert-v*-linux-amd64
sudo cp mkcert-v*-linux-amd64 /usr/local/bin/mkcert
sudo apt-get install libnss3-tools &&>> $REDIRECT_LOGFILE
CAROOT=`pwd` mkcert --install
mkdir -p ~/.local/share/mkcert
sudo cp root* ~/.local/share/mkcert
echo "Generating certificates..."
mkcert ${HELLO_FLEET_ID} ${HELLO_FLEET_ID}.local ${HELLO_FLEET_ID}.dev localhost 127.0.0.1 0.0.0.0 ::1 &&>> $REDIRECT_LOGFILE
rm mkcert-v*-linux-amd64
echo "Adding certificates to environment file..."
cd /home/hello-robot/web_interface_ws/src/stretch_teleop_interface
touch .env
echo certfile=${HELLO_FLEET_ID}+6.pem >> .env
echo keyfile=${HELLO_FLEET_ID}+6-key.pem >> .env
echo ""

echo "Build workspace..."
cd /home/hello-robot/web_interface_ws
colcon build
source install/setup.bash

echo "Updating port privledges..."
sudo sysctl -w net.ipv4.ip_unprivileged_port_start=80
echo net.ipv4.ip_unprivileged_port_start=80 | sudo tee --append /etc/sysctl.d/99-sysctl.conf

echo ""
echo "The stretch web interface installation script has finished."
echo "Run ./launch_interface launch the interface."
echo ""

