# Updating your ROS Workspace

## Why

ROS1 and ROS2 organize software by "workspaces", where ROS packages are developed, compiled, and made available to run from the command line. By default, a ROS1 workspace called `catkin_ws` is available in the home directory. Similarly, a ROS2 workspace called `ament_ws` is available in the home directory. The operating system installed (and therefore version of ROS installed) on your robot dictates whether you'll have a `catkin_ws` or `ament_ws` folder.

This guide will show you how to replace existing or create new ROS1/2 workspaces for developing ROS software.

## How

Open a terminal and execute the following.

```bash
cd ~
git clone https://github.com/hello-robot/stretch_install
cd stretch_install
git pull
./stretch_update_ros_workspace.sh
```

## Wrap up

Close your current terminal and open a new one. The new terminal will have automatically activated the ROS workspace(s).

Your new ROS workspace is now set up successfully!

---

## Troubleshooting

This section provides suggestions for common errors that occur during installation. If you become stuck and don't find an answer here, please email us or contact us through [the forum](https://forum.hello-robot.com/).

### Rosdep failure on RTabMap

After a failure, if you see the following error in your log file (from `~/stretch_user/log/stretch_create_ament_workspace.<timestamp>_log.txt`):

```
ERROR: the following rosdeps failed to install
  apt: command [sudo -H apt-get install -y ros-humble-rtabmap-ros] failed
```

Open a terminal, run `sudo apt install ros-humble-rtabmap-ros`, and then try updating the ROS workspace again.

### 'Conflicting ROS version sourced' error

If you are seeing the following error:

```
###########################################
CREATING <ROS VERSION> WORKSPACE at <WS DIR>
###########################################
[...]
Ensuring correct version of ROS is sourced...
Cannot create workspace while a conflicting ROS version is sourced. Exiting.
```

The ROS workspace is not created because the check that a conflicting ROS version isn't already sourced has failed. For example, if you're creating an ROS2 Ament workspace, but ROS1 Noetic was previously sourced in the same environment, the check will error out since the new ROS2 workspace would fail to find its dependencies correctly in this environment. Sourcing a version of ROS typically happens using the following command: `source /opt/ros/<ros version>/setup.bash`. If you ran this command to source a conflicting version previously, simply open a new terminal and the new environment won't have the conflicting ROS version sourced. If you didn't run this command and you're still getting the error, it's likely because the command exists in the `~/.bashrc` dotfile. Every new bash shell (i.e. the terminal you open when searching for 'Terminal' in system applications) runs the commands in the `~/.bashrc` dotfile. Look at the bottom of this dotfile for this command, comment it out temporarily, and open a new terminal. This new shell environment should have no trouble creating the ROS workspace.

### 'ROS_DISTRO was set before' warning

If you are seeing the following warning:

```
ROS_DISTRO was set to '<ROS VERSION>' before. Please make sure that the environment does not mix paths from different distributions.
```

Multiple versions of ROS are being sourced in the same environment. This is known to cause issues with the `rosdep` tool, and might cause issues elsewhere as well. If you haven't explicitly sourced conflicting versions by using the `source /opt/ros/<ros version>/setup.bash` (a variant on this command could look like `source ~/<ws dir>/develop/setup.bash`) command twice, then it's likely that one or two versions of ROS are implicitly being sourced in the `~/.bashrc` dofile. Every new bash shell (i.e. the terminal you open when searching for 'Terminal' in system applications) runs the commands in the `~/.bashrc` dotfile. Look at the bottom of this dotfile for the `source` command and ensure conflicting versions aren't being sourced.

------
<div align="center"> All materials are Copyright 2020-2024 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks.</div>
