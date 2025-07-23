#!/bin/bash
set -o pipefail

sudo sleep 0.1
source /etc/os-release
factory_osdir="$VERSION_ID"
if [[ ! $factory_osdir =~ ^(18.04|20.04|22.04|24.04)$ ]]; then
    echo "Could not identify OS. Please contact Hello Robot Support."
    exit 1
fi

function echo_failure_help {
    echo ""
    echo "#############################################"
    echo "FAILURE. UPDATING ROS WORKSPACE DID NOT COMPLETE."
    echo "Look at the troubleshooting guide for solutions to common issues: https://docs.hello-robot.com/0.3/installation/ros_workspace/"
    echo "or contact Hello Robot support"
    echo "#############################################"
    echo ""
    exit 1
}

cd $HOME/stretch_install/factory/$factory_osdir
if [[ $factory_osdir = "18.04" ]]; then
    ./stretch_create_catkin_workspace.sh -w "$HOME/catkin_ws"
elif [[ $factory_osdir = "20.04" ]]; then
    ./stretch_create_catkin_workspace.sh -w "$HOME/catkin_ws"
elif [[ $factory_osdir = "22.04" ]]; then
    ./stretch_create_ament_workspace.sh -w "$HOME/ament_ws"
elif [[ $factory_osdir = "24.04" ]]; then
    ./stretch_create_ament_workspace.sh -w "$HOME/ament_ws"
fi
if [ $? -ne 0 ]; then
    echo_failure_help
fi

echo ""
echo "#############################################"
echo "DONE! ROS WORKSPACE SET UP SUCCESSFULLY."
echo "#############################################"
echo ""
