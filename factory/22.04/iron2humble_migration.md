# Iron -> Humble Migration

The Stretch Install repo has switched the version of ROS2 installed with the Ubuntu 22.04 software stack. On Sept 7th, 2023, we switched it to ROS2 Iron. On Sept 14, 2023, we switched it back to ROS2 Humble. We expect some users to have upgraded in that week, so this guide serves to help migrate from Iron -> Humble.

## Steps

 0. Verify this guide applies to you
    ```
    lsb_release -d
    echo $ROS_DISTRO
    ```
    You should see "Description: Ubuntu 22.04.* LTS" and "iron". If not, you do not need this guide.

 1. Clone Stretch Install into your home folder:
    ```
    cd ~/
    git clone https://github.com/hello-robot/stretch_install.git
    cd ~/stretch_install
    git pull
    ```

 2. Run the system install script
    ```
    ./factory/22.04/stretch_install_system.sh
    ```
    The script can fails silently. If the last line printed out isn't "Install librealsense2 packages", something went wrong. Contact Hello Robot Support.

 3. Update the .bashrc script
    ```
    gedit ~/.bashrc
    ```
    A text editor will open. Scroll to the bottom of the file, and look for a section called "STRETCH BASHRC SETUP". Under this section, you'll see a line that says `source /opt/ros/iron/setup.bash`. Change this line to `source /opt/ros/humble/setup.bash`. Next, look for a line that says `source $HOME/ament_ws/install/setup.bash` or similar. Delete this line. Now save the file and close the editor.

 4. Refresh your terminal by closing your current terminal and opening a new one

 5. Run the ament workspace creation script
    ```
    cd ~/stretch_install
    ./factory/22.04/stretch_create_ament_workspace.sh
    ```
    The script can fails silently. If the last line printed out isn't "Setup calibrated robot URDF...", something went wrong. Contact Hello Robot Support.

 6. Refresh your terminal by closing your current terminal and opening a new one

Success! Your robot is now set up with ROS2 Humble correctly.
