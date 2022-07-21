#!/bin/bash
set -e

if [ "$ROS_DISTRO" ]; then
    UPDATING=true
    echo "###########################################"
    echo "UPDATING EXISTING ROS WORKSPACE"
else
    UPDATING=false
    echo "###########################################"
    echo "CREATING NEW ROS WORKSPACE"
fi

if [ "$UPDATING" = true ]; then
     echo "Not updating ROS in .bashrc since it's already there"
else
    echo "UPDATE .bashrc for ROS"
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    echo "add catkin development workspace overlay to .bashrc"
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    echo "set log level for realsense camera"
    echo "export LRS_LOG_LEVEL=None #Debug" >> ~/.bashrc
    echo "source .bashrc"
    source ~/.bashrc
    source /opt/ros/melodic/setup.bash
    echo "DONE UPDATING .bashrc"
    echo ""
fi

# create the ROS workspace (see http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
echo "Creating the ROS workspace..."
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "Source .bash file"
source ~/catkin_ws/devel/setup.bash
echo "Make sure new ROS package is indexed"
rospack profile
echo "Install ROS package"
catkin_make install
echo "Done."
echo ""

echo "Installing Stretch ROS..."
dd="$HOME/catkin_ws/src/stretch_ros"
if [ -d "$dd" ]; then
    echo "The stretch_ros repositories exists at $dd. Performing git pull."
    cd $dd
    git pull
    if [ $? -ne 0 ]; then
        echo "Installation failed. Exiting"
        exit 1
    fi
else
    echo "Cloning stretch_ros repositories."
    cd $HOME/catkin_ws/src
    git clone https://github.com/hello-robot/stretch_ros.git
    if [ $? -ne 0 ]; then
        echo "Installation failed. Exiting"
        exit 1
    fi
fi

echo "Updating meshes in stretch_ros to this robot batch"
~/catkin_ws/src/stretch_ros/stretch_description/meshes/update_meshes.py
echo "Done."
echo ""

cd ~/catkin_ws/
echo "Make the workspace"
catkin_make
echo "Make sure new ROS packages is indexed"
rospack profile
echo "Install ROS packages. This is important for using Python modules."
catkin_make install
echo "Done."
echo ""

if [ "$UPDATING" = true ]; then
    echo "Not updating URDF."
else
    echo "Setup calibrated robot URDF..."
    rosrun stretch_calibration update_uncalibrated_urdf.sh
    #This will grab the latest URDF and calibration files from ~/stretch_user
    #rosrun stretch_calibration update_with_most_recent_calibration.sh
    #Force to run interactive so $HELLO_FLEET_ID is found
    echo "This may fail if doing initial robot bringup. That is OK."
    bash -i ~/catkin_ws/src/stretch_ros/stretch_calibration/nodes/update_with_most_recent_calibration.sh
    echo "Done."
fi
echo ""

echo "Compiling Cython code..."
cd ~/catkin_ws/src/stretch_ros/stretch_funmap/src/stretch_funmap
./compile_cython_code.sh
echo "Done."

# install scan_tools for laser range finder odometry
echo "Installing CSM..."
dd="$HOME/catkin_ws/src/csm"
if [ -d "$dd" ]
then
    echo "Directory $dd exists. Performing git pull."
    cd $dd
    git pull
    if [ $? -ne 0 ]; then
        echo "Installation failed. Exiting"
        exit 1
    fi
else
    echo "Cloning the csm github repository."
    cd $HOME/catkin_ws/src
    git clone https://github.com/AndreaCensi/csm
    if [ $? -ne 0 ]; then
        echo "Installation failed. Exiting"
        exit 1
    fi
fi


echo "Fetch csm dependencies."
cd ~/catkin_ws/
rosdep update
rosdep install --from-paths src --ignore-src -r -y
echo "Make csm."
sudo apt --yes install libgsl0-dev
cd ~/catkin_ws/csm/
cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/local .
make
echo "Install csm."
sudo make install
echo "Cloning the scan_tools github repository."
cd ~/catkin_ws/src/
git clone https://github.com/ccny-ros-pkg/scan_tools.git
cd scan_tools
git pull
echo "Make scan_tools."
cd ~/catkin_ws/
catkin_make
echo "Make sure new ROS packages are indexed"
rospack profile
echo "Done."
echo ""


echo "DONE CREATING ROS WORKSPACE"
echo "###########################################"
echo ""

