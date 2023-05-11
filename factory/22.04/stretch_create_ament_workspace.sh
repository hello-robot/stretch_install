#!/bin/bash
set -e

REDIRECT_LOGDIR="$HOME/stretch_user/log"
AMENT_WSDIR="$HOME/ament_ws"
while getopts l:w: opt; do
    case $opt in
        l)
            if [[ -d $OPTARG ]]; then
                REDIRECT_LOGDIR=$OPTARG
            fi
            ;;
        w)
            AMENT_WSDIR=$OPTARG
            ;;
    esac
done
REDIRECT_LOGFILE="$REDIRECT_LOGDIR/stretch_create_ament_workspace.`date '+%Y%m%d%H%M'`_redirected.txt"

echo "###########################################"
echo "CREATING ROS 2 AMENT WORKSPACE at $AMENT_WSDIR"
echo "###########################################"

echo "Ensuring correct version of ROS is sourced..."
if [[ ! $ROS_VERSION -eq 2 ]]; then
    echo "Cannot create workspace while a conflicting ROS version is sourced. Exiting."
    exit 1
fi

if [ $ROS_DISTRO = "humble" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Please source a valid ROS 2 distro. Only ROS 2 Humble supported currently."
fi

echo "You are about to delete and replace the existing ament workspace. If you have any personal data in the workspace, please create a back up before proceeding."
prompt_yes_no(){
read -p "Do you want to continue? Press (y/n for yes/no): " x
if [ $x = "n" ]; then
		echo "Exiting the script."
		exit 1
elif [ $x = "y" ]; then
		echo "Continuing to create a new ament workspace."
else
	echo "Press 'y' for yes or 'n' for no."
	prompt_yes_no
fi
}

prompt_yes_no

echo "Deleting $AMENT_WSDIR if it already exists..."
sudo rm -rf $AMENT_WSDIR
echo "Creating the workspace directory..."
mkdir -p $AMENT_WSDIR/src
echo "Cloning the workspace's packages..."
cd $AMENT_WSDIR/src
vcs import --input ~/stretch_install/factory/22.04/stretch_ros2_humble.repos >> $REDIRECT_LOGFILE
echo "Fetch ROS packages' dependencies (this might take a while)..."
cd $AMENT_WSDIR/
rosdep install --rosdistro=humble -iyr --skip-keys="librealsense2" --from-paths src &>> $REDIRECT_LOGFILE

# TODO: replace these hacks
# echo "Fetch calibration data from catkin_ws..."
# if [ -f "$HOME/catkin_ws/src/stretch_ros/stretch_description/urdf/stretch.urdf" ]; then
#     cp ~/catkin_ws/src/stretch_ros/stretch_description/urdf/stretch.urdf $AMENT_WSDIR/src/stretch_ros2/stretch_description/urdf/
# fi
# if [ -f "$HOME/catkin_ws/src/stretch_ros/stretch_core/config/controller_calibration_head.yaml" ]; then
#     cp ~/catkin_ws/src/stretch_ros/stretch_core/config/controller_calibration_head.yaml $AMENT_WSDIR/src/stretch_ros2/stretch_core/config/
# fi

echo "Compile the workspace (this might take a while)..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release &>> $REDIRECT_LOGFILE
echo "Source setup.bash file..."
source $AMENT_WSDIR/install/setup.bash
echo "Update ~/.bashrc dotfile to source workspace..."
echo "#source $AMENT_WSDIR/install/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc

# TODO:
#echo "Updating meshes in stretch_ros to this robot's batch..."
#. /etc/hello-robot/hello-robot.conf
#export HELLO_FLEET_ID HELLO_FLEET_ID
#export HELLO_FLEET_PATH=${HOME}/stretch_user
#$CATKIN_WSDIR/src/stretch_ros/stretch_description/meshes/update_meshes.py >> $REDIRECT_LOGFILE
#echo "Setup uncalibrated robot URDF..."
#bash -i $CATKIN_WSDIR/src/stretch_ros/stretch_calibration/nodes/update_uncalibrated_urdf.sh >> $REDIRECT_LOGFILE
#echo "Setup calibrated robot URDF..."
#bash -i $CATKIN_WSDIR/src/stretch_ros/stretch_calibration/nodes/update_with_most_recent_calibration.sh >> $REDIRECT_LOGFILE
#echo "Compiling FUNMAP's Cython code..."
#cd $CATKIN_WSDIR/src/stretch_ros/stretch_funmap/src/stretch_funmap
#./compile_cython_code.sh &>> $REDIRECT_LOGFILE

