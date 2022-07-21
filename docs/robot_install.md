# Performing a Robot Installation 

# Why

A fresh robot-level install should only be done under the guidance of [Hello Robot Support](https://forum.hello-robot.com). This guide will lead you through a robot-level install, which can be used to:

 - Erase the previous OS and set up Stretch with an entirely fresh software stack
 - Erase a corrupted OS and set up Stretch with an entirely fresh software stack
 - Upgrade Stretch by installing the 20.04 software stack alongside the previous OS

Each OS installs on a separate partition on the hard drive. You can create as many robot-level installs as you want (i.e. new partitions), but you're limited by the robot's 500GB hard drive.

Currently, there are two available versions of the software stack, one with Ubuntu 18.04 LTS and another with Ubuntu 20.04 LTS. Ubuntu 18.04 LTS shipped on robots until summer 2022, and included software for ROS Melodic and Python2. Ubuntu 20.04 LTS, the newest software stack, comes with ROS Noetic, Python3, and experimental ROS2 Galactic/Humble software. You can look at your robot's About system settings page to identify which OS it is running.

# How

There are a few steps to performing a new robot install:

 - Backup robot configuration data
 - Setup the BIOS (only necessary for NUCs not previously configured by Hello Robot)
 - Install Ubuntu 18.04 or 20.04
 - Run the new robot installation script

## Back up robot configuration data

If your new robot install will replace a previous one, it is a good idea to backup all valuable data beforehand. **Data from the previous robot install will be deleted.** In particular, your new robot install will require the old robot install's robot configuration data. The steps to copy this material from an existing install is:

 1. Boot into the robot's original Ubuntu partition.
 2. Copy the /etc/hello-robot to a USB key.
    - For example, you can run `cp -r /etc/hello-robot /media/$USER/<USBKEY>` from the command line where `<USBKEY>` is replaced with the mounted USB key's name.
    - Or, you can open the file explorer to copy the directory.

If your previous partition is corrupted or inaccessible, contact Hello Robot support and they will be able to give a copy of this material.

## Setup the BIOS

This step can be skipped if your robot had an existing software install on it. Otherwise, follow the [guide to set up the BIOS](./docs/configure_BIOS.md).

## Install Ubuntu

Choose between the following guides based on whether you're installing Ubuntu 18.04 or Ubuntu 20.04 (see above for info on what software ships with each OS). Within these guides, you'll have the choice of whether to replace the previous OS partition or to install alongside it.

 - [Ubuntu 18.04 Installation guide](./install_ubuntu_18.04.md).
 - [Ubuntu 20.04 Installation guide](./install_ubuntu_20.04.md).

## Run the new robot installation script

Login to the `hello-robot` user account and install git:

```bash
sudo apt update
sudo apt install git
```

**Note**: The system may not be able to run 'apt' immediately after a reboot as the OS may be running automatic updates in the background. Typically, waiting 10-20 minutes will allow you to use 'apt' again.

Next, place the robot calibration data in the home folder (i.e. /home/$USER/) of your new Ubuntu parition:

 1. Copy the folder of the form "stretch-re1-xxxx" into the home folder (i.e. /home/$USER/). If you backed this up from `/etc/hello-robot` previously, it will be found within the `hello-robot` directory on the USB key.
    - For example, you can run a command similar to `cp -r /media/$USER/USBKEY/hello-robot/stretch-re1-xxxx /home/$USER/` where `xxxx` is replaced with your robot's serial number.
    - Or, you can use the visual file explorer.

Next, pull down the Stretch Install repository and being the installation process:

```bash
cd ~/
git clone https://github.com/hello-robot/stretch_install
cd stretch_install
./stretch_new_robot_install.sh
```
