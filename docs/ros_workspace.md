# Creating a new ROS Workspace

## Why

ROS1 and ROS2 organize software by "workspaces", where ROS packages are developed, compiled, and made available to run from the command line. By default, a ROS1 workspace called `catkin_ws` is available in the home directory. If your robot is running Ubuntu 20.04 (see [upgrade guide](./robot_install.md)), a ROS2 workspace called `ament_ws` is also available in the home directory. This guide will show you how to create new ROS1/2 workspaces to develop new ROS software.

## How

Open a terminal and execute the following to pull down the Stretch Install repository.

```bash
cd ~
git clone https://github.com/hello-robot/stretch_install
cd stretch_install
git pull
```

Choose one of the following commands to create either or both ROS1/ROS2 workspaces.

```bash
# Create a ROS1 workspace
./factory/stretch_create_catkin_workspace.sh

# Create a ROS2 workspace
./factory/stretch_create_ament_workspace.sh
```

Close your current terminal and open a new one. The new terminal will have automatically activated the ROS workspace(s).

Your new ROS workspace is now set up successfully!
