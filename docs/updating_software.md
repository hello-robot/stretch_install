# Updating Stretch Software

Stretch's software is improved with new features and bug fixes with each update. In this guide, we cover when and how to update the various software components on your Stretch RE1.

## When to Update

We develop our software publicly on Github, allowing anyone to follow/propose the development of a code feature or bug fix. While we wholeheartedly welcome collaboration on Github, it is not necessary to be active on Github to follow our software releases. We announce every major release of software on our [forum](https://forum.hello-robot.com/c/announcements). These are stable releases with code that has been extensively tested on many Stretch RE1s. To be notified of new releases, create an account on the forum and click the bell icon in the top left of the [announcements section](https://forum.hello-robot.com/c/announcements/6). The forum is also available to report issues and ask questions about any of our software packages.

## How to Update

Each Stretch RE1 is shipped with firmware, a Python SDK, and ROS packages developed specifically for Stretch. There are separate processes for updating each of these components.

### Stretch ROS

Stretch ROS is the [Robot Operating System](https://www.ros.org/about-ros/) (ROS) interface to the robot. Many robotics developers find ROS useful to bootstrap their robotics software developments. You may update it using the following commands:

```console
roscd stretch_core
git pull
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

### Stretch Python Modules

There are a few Python modules that allow users to work with the robot in pure Python2/3. These modules are:

 - Stretch Body is the Python SDK to the robot. It abstracts away the low level details of communication with the embedded devices and provides an intuitive API to working with the robot.
 - Stretch Body Tools is a set of command line tools that builds on Stretch Body. They allow you to quickly home, stow, command joints, read sensors, and more on the robot.
 - Stretch Tool Share is a repository of accessories (e.g. expo marker gripper, docking station) for Stretch, created by the community and Hello Robot.
 - Stretch Factory is a library/set of command line tools that enable calibrations, firmware upgrading, and other factory related tasks.

On Ubuntu 18.04, update them using:

```console
python -m pip install -U hello-robot-stretch-body hello-robot-stretch-body-tools hello-robot-stretch-tool-share hello-robot-stretch-factory
```

On Ubuntu 20.04, update them using:

```console
python3 -m pip install -U hello-robot-stretch-body hello-robot-stretch-body-tools hello-robot-stretch-tool-share hello-robot-stretch-factory
```

### Stretch Firmware

The firmware and the Python SDK (called Stretch Body) communicate on an established protocol. Therefore, it is important to maintain a protocol match between the different firmware and Stretch Body versions. Fortunately, there is a tool that handles this automatically. In the command line, run the following command:

```console
RE1_firmware_updater.py --recommended
```

This script will automatically determine what version is currently running on the robot and provide a recommendation for a next step. Follow the next steps provided by the firmware updater script.

### Ubuntu

The operating system upon which Stretch RE1 is built is called Ubuntu. This operating system provides the underlying packages that power Stretch's software packages. Furthermore, users of Stretch depend on this operating system and the underlying packages to develop software on Stretch. Therefore, it is important to keep the OS and these underlying packages up to date. In the command line, run the following command:

```console
sudo apt update
sudo apt upgrade
sudo apt autoremove
```

[Apt](https://en.wikipedia.org/wiki/APT_(software)) is the package manager that handles updates for all Ubuntu packages.

### Verify updated successfully

Finally, reboot your robot and execute the following to confirm that all software updated successfully.

```console
stretch_robot_system_check.py
```

If you run into any errors, see the troubleshooting guide below or contact [Hello Robot Support](https://forum.hello-robot.com).

## Troubleshooting

### Param Migration Error

If you see the following error:

```
Please run tool RE1_migrate_params.py before continuing. For more details, see https://forum.hello-robot.com/t/425
```

This error appears because the organization of Stretch's parameters has changed since Stretch Body v0.3 and requires a migration of these parameters to the new organization system. Executing the following command will automaticaly migrate your parameters over:

```console
RE1_migrate_params.py
```

To learn more about Stretch's parameter system, see [this tutorial](https://docs.hello-robot.com/0.1/parameters_tutorial/).

### Firmware Mismatch Error

If you see the following error:

```
----------------
Firmware protocol mismatch on /dev/XXXX.
Protocol on board is pX.
Valid protocol is: pX.
Disabling device.
Please upgrade the firmware and/or version of Stretch Body.
----------------
```

This error appears because the low level Python SDK and the firmware cannot communicate to each other. There is a protocol mismatch preventing communication between the two. Simply run the following script and follow its recommendations to upgrade/downgrade the firmware as necessary to match the protocol level of Stretch Body.

```console
$ RE1_firmware_updater.py --status
```
