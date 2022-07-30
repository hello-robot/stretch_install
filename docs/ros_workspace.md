# Creating a new ROS Workspace

## Why

ROS1 and ROS2 organize software by "workspaces", where ROS packages are developed, compiled, and made available to run from the command line. By default, a ROS1 workspace called `catkin_ws` is available in the home directory. If your robot is running Ubuntu 20.04, an additional ROS2 workspace called `ament_ws` is also available in the home directory. This guide will show you how to replace existing or create new ROS1/2 workspaces for developing ROS software.

## How

Open a terminal and execute the following to pull down the Stretch Install repository.

```bash
cd ~
git clone https://github.com/hello-robot/stretch_install
cd stretch_install
git pull
```

### Ubuntu 18.04

Run the following command to create a ROS1 workspace (replacing `<optional-path-to-ws>` with a filepath to the workspace. Not providing it defaults to `~/catkin_ws`).

```bash
./factory/18.04/stretch_create_catkin_workspace.sh <optional-path-to-ws>
```

### Ubuntu 20.04

Choose one of the following commands to create either or both ROS1/ROS2 workspaces (replacing `<optional-path-to-ws>` with a filepath to the workspace. Not providing it defaults to `~/catkin_ws` or `~/ament_ws`).

```bash
# Create a ROS1 workspace
./factory/20.04/stretch_create_catkin_workspace.sh <optional-path-to-ws>

# Create a ROS2 workspace
./factory/20.04/stretch_create_ament_workspace.sh <optional-path-to-ws>
```

### Wrap up

Close your current terminal and open a new one. The new terminal will have automatically activated the ROS workspace(s).

Your new ROS workspace is now set up successfully!
