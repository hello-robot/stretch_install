#!/bin/bash
set -e
export COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification

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
echo "CREATING HUMBLE AMENT WORKSPACE at $AMENT_WSDIR"
echo "###########################################"

echo "Ensuring correct version of ROS is sourced..."
if [[ $ROS_DISTRO && ! $ROS_DISTRO = "humble" ]]; then
    echo "Cannot create workspace while a conflicting ROS version is sourced. Exiting."
    exit 1
fi
source /opt/ros/humble/setup.bash

if [[ -d $AMENT_WSDIR ]]; then
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
fi

echo "Downgrade to numpy 1.26.4..."
pip3 install numpy==1.26.4 &>> $REDIRECT_LOGFILE

export PATH=${PATH}:~/.local/bin
. /etc/hello-robot/hello-robot.conf
export HELLO_FLEET_ID=$HELLO_FLEET_ID
export HELLO_FLEET_PATH=${HOME}/stretch_user
echo "Updating rosdep indices..."
rosdep update --include-eol-distros &>> $REDIRECT_LOGFILE
echo "Deleting $AMENT_WSDIR if it already exists..."
sudo rm -rf $AMENT_WSDIR
echo "Creating the workspace directory..."
mkdir -p $AMENT_WSDIR/src
echo "Cloning the workspace's packages..."
cd $AMENT_WSDIR/src
vcs import --input ~/stretch_install/factory/22.04/stretch_ros2_humble.repos &>> $REDIRECT_LOGFILE
echo "Fetch ROS packages' dependencies (this might take a while)..."
cd $AMENT_WSDIR/
# The rosdep flags below have been chosen very carefully. Please review the docs before changing them.
# https://docs.ros.org/en/independent/api/rosdep/html/commands.html
rosdep install --rosdistro=humble -iy --skip-keys="librealsense2 realsense2_camera" --from-paths src &>> $REDIRECT_LOGFILE
sudo apt remove -y ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs &>> $REDIRECT_LOGFILE
pip3 cache purge &>> $REDIRECT_LOGFILE

echo "Install web interface dependencies..."
cd $AMENT_WSDIR/src/stretch_web_teleop
pip3 install -r requirements.txt &>> $REDIRECT_LOGFILE
npm install --force &>> $REDIRECT_LOGFILE
npx playwright install &>> $REDIRECT_LOGFILE
echo "Generating web interface certs..."
cd $AMENT_WSDIR/src/stretch_web_teleop/certificates
curl -JLO "https://dl.filippo.io/mkcert/latest?for=linux/amd64" &>> $REDIRECT_LOGFILE
chmod +x mkcert-v*-linux-amd64
sudo cp mkcert-v*-linux-amd64 /usr/local/bin/mkcert
CAROOT=`pwd` mkcert --install &>> $REDIRECT_LOGFILE
mkdir -p ~/.local/share/mkcert
rm -rf ~/.local/share/mkcert/root*
cp root* ~/.local/share/mkcert
mkcert ${HELLO_FLEET_ID} ${HELLO_FLEET_ID}.local ${HELLO_FLEET_ID}.dev localhost 127.0.0.1 0.0.0.0 ::1 &>> $REDIRECT_LOGFILE
rm mkcert-v*-linux-amd64
cd $AMENT_WSDIR/src/stretch_web_teleop
touch .env
echo certfile=${HELLO_FLEET_ID}+6.pem >> .env
echo keyfile=${HELLO_FLEET_ID}+6-key.pem >> .env
cd $AMENT_WSDIR/

echo "Install FUNMAP dependencies..."
curl -LsSf https://astral.sh/uv/install.sh | sh &>> $REDIRECT_LOGFILE
cd $AMENT_WSDIR/src/stretch_ros2/stretch_funmap
uv venv --system-site-packages --allow-existing .venv &>> $REDIRECT_LOGFILE
uv sync --frozen &>> $REDIRECT_LOGFILE
echo "Compile cython modules..."
pip3 install setuptools==59.6.0 &>> $REDIRECT_LOGFILE # must use <61 for Colcon to correctly build Stretch FUNMAP
uv run cythonize stretch_funmap/cython_min_cost_path.pyx -3 -i &>> $REDIRECT_LOGFILE
cd $AMENT_WSDIR/

echo "Compile the workspace (this might take a while)..."
pip3 install setuptools==59.6.0 &>> $REDIRECT_LOGFILE # must use <61 for Colcon to correctly build Stretch FUNMAP
pip3 uninstall -y setuptools-scm &>> $REDIRECT_LOGFILE
colcon build --symlink-install &>> $REDIRECT_LOGFILE
echo "Source setup.bash file..."
source $AMENT_WSDIR/install/setup.bash
echo "Updating port privledges..."
sudo sysctl -w net.ipv4.ip_unprivileged_port_start=80 &>> $REDIRECT_LOGFILE
echo net.ipv4.ip_unprivileged_port_start=80 | sudo tee --append /etc/sysctl.d/99-sysctl.conf &>> $REDIRECT_LOGFILE
echo "Update ~/.bashrc dotfile to source workspace..."
echo "source $AMENT_WSDIR/install/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "Updating meshes and xacros to ROS from stretch_urdf package."
/home/$USER/.local/bin/stretch_urdf_ros_update.py -y -v >> $REDIRECT_LOGFILE
echo "Setup uncalibrated robot URDF..."
ros2 run stretch_calibration update_uncalibrated_urdf >> $REDIRECT_LOGFILE
echo "Setup calibrated robot URDF..."
ros2 run stretch_calibration update_with_most_recent_calibration >> $REDIRECT_LOGFILE
colcon build --symlink-install &>> $REDIRECT_LOGFILE

echo "Amend FUNMAP executables to use venv..."
pip3 install -U hello-robot-stretch-factory &>> $REDIRECT_LOGFILE # Necessary for the below CLI
REx_amend_venv_execs.py stretch_funmap &>> $REDIRECT_LOGFILE

echo "Downgrade to numpy 1.26.4..."
pip3 install numpy==1.26.4 &>> $REDIRECT_LOGFILE
