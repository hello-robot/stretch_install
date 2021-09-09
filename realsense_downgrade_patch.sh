# This script downgrades and pins the RealSense packages to 2.45
# It is intended that this scripts runs after a full installation

echo "Update latest apt indices"
sudo apt update

echo "Remove newest librealsense2 binaries"
sudo apt remove librealsense2 librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg librealsense2-udev-rules librealsense2-gl librealsense2-gl-dev librealsense2-gl-dbg librealsense2-net librealsense2-net-dev librealsense2-net-dbg

echo "Install 2.45 librealsense2 binaries"
sudo apt install librealsense2-dkms=1.3.18-0ubuntu1 librealsense2=2.45.0-0~realsense0.4551 librealsense2-utils=2.45.0-0~realsense0.4551 librealsense2-dev=2.45.0-0~realsense0.4551 librealsense2-dbg=2.45.0-0~realsense0.4551 librealsense2-gl=2.45.0-0~realsense0.4551 librealsense2-net=2.45.0-0~realsense0.4551 librealsense2-udev-rules=2.45.0-0~realsense0.4551

echo "Remove realsense-ros binaries"
sudo apt remove ros-melodic-realsense2-camera ros-melodic-realsense2-description ros-melodic-librealsense2

echo "Building realsense-ros 2.3.0 from source"
cd ~/catkin_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b 2.3.0
cd ~/catkin_ws
catkin_make
rospack profile

echo "Copying apt pinning file into standard location"
sudo cp $HOME/stretch_install/factory/librealsense2_packages /etc/apt/preferences.d/

echo "NOTE: Please restart the robot. Then use 'realsense-viewer' to upgrade the camera's firmware to the latest versions."

