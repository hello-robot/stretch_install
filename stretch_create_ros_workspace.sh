sudo echo "###########################################"
echo "INSTALLATION OF ROS WORKSPACE"
 # update .bashrc before using catkin tools

if [ ${ROS_DISTRO+x} ]; then
     echo "Updating: Not updating ROS in .bashrc"
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

# create the ROS workspace
# see http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
echo "Creating the ROS workspace..."
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "Source .bash file"
source ~/catkin_ws/devel/setup.bash
echo "Make sure new ROS package is indexed"
rospack profile
echo "Done."
echo ""

echo "Make ROS package"
cd ~/catkin_ws/
catkin_make
echo "Install ROS package"
catkin_make install
echo "Make sure new ROS package is indexed"
rospack profile
echo "DONE INSTALLING ROS_NUMPY FROM GITHUB"
echo ""

# clone the Hello Robot ROS repository
echo "Install the Hello Robot ROS repository"
cd ~/catkin_ws/src/

echo "Cloning stretch_ros repository"
git clone https://github.com/hello-robot/stretch_ros.git
cd stretch_ros
git pull

echo "Updating meshes in stretch_ros to this robot batch"
~/catkin_ws/src/stretch_ros/stretch_description/meshes/update_meshes.py

cd ~/catkin_ws/
echo "Make the ROS repository"
catkin_make
echo "Make sure new ROS package is indexed"
rospack profile
echo "Install ROS packages. This is important for using Python modules."
catkin_make install
echo ""

echo "Setup calibrated robot URDF"
rosrun stretch_calibration update_uncalibrated_urdf.sh
#This will grab the latest URDF and calibration files from ~/stretch_user
#rosrun stretch_calibration update_with_most_recent_calibration.sh
#Force to run interactive so $HELLO_FLEET_ID is found
echo "This may fail if doing initial robot bringup. That is OK."
source $HOME/catkin_ws/devel/setup.bash
bash -i ~/catkin_ws/src/stretch_ros/stretch_calibration/nodes/update_with_most_recent_calibration.sh
echo "--Done--"

# compile Cython code
echo "Compiling Cython code"
cd ~/catkin_ws/src/stretch_ros/stretch_funmap/src/stretch_funmap
./compile_cython_code.sh
echo "Done"

# install scan_tools for laser range finder odometry
echo "INSTALL SCAN_TOOLS FROM GITHUB"
cd ~/catkin_ws/
echo "Cloning the csm github repository."
git clone https://github.com/AndreaCensi/csm
cd csm
git pull

echo "Handle csm dependencies."
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
echo ""

echo "DONE WITH SETTING UP ROS WORKSPACE"
echo "###########################################"
echo ""