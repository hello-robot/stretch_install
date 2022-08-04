# Performing a Robot Installation 

# Why

A fresh robot-level install should only be done under the guidance of [Hello Robot Support](https://forum.hello-robot.com). This guide will lead you through a robot-level install, which can be used to:

 - Erase the previous OS and set up Stretch with an entirely fresh software stack
 - Erase a corrupted OS and set up Stretch with an entirely fresh software stack
 - Upgrade Stretch by installing the 20.04 software stack alongside the previous OS

Each OS installs on a separate partition on the hard drive. You can create as many robot-level installs (i.e. new partitions) as will fit in your robot's 500GB hard drive.

Currently, there are two available versions of the software stack, one with Ubuntu 18.04 LTS and another with Ubuntu 20.04 LTS. Ubuntu 18.04 LTS shipped on robots until summer 2022, and included software for ROS Melodic and Python2. Ubuntu 20.04 LTS, the newest software stack, comes with ROS Noetic, Python3, and experimental ROS2 Galactic software. You can look at your robot's About page in system settings to identify which OS it is running.

# How

There are a few steps to performing a new robot install:

 1. Backup robot configuration data
 2. Setup the BIOS (only necessary for NUCs not previously configured by Hello Robot)
 3. Install Ubuntu 18.04 or 20.04
 4. Run the new robot installation script

## Back up robot configuration data

It is a good idea to backup all valuable data beforehand. If your new robot install will replace a previous one, **data from the previous robot install will be deleted.** Even if your new robot install will live alongside the previous one(s), **data from the previous robot install(s) can be lost.**

In particular, your new robot install will require the old install's robot calibration data. The steps to copy this material from an existing install is:

 1. Boot into the robot's original Ubuntu partition and plug in a USB key.
 2. The robot calibration data lives inside of a directory called `stretch-re1-<xxxx>`, where `<xxxx>` is your robot's serial number. There's a few versions of this directory and you will need to decide which version to backup. Each Ubuntu user has a version of this directory located at `/home/$USER/stretch_user/stretch-re1-<xxxx>`. These user versions are updated when the user runs a URDF calibration, swaps out an end effector, updates Stretch parameters, and more. There's also a system version located at `/etc/hello-robot/stretch-re1-<xxxx>`, which is likely the oldest version since it was created at Hello Robot HQ. If you're not sure which version to backup, use the version at `/etc/hello-robot/stretch-re1-<xxxx>` for the next step.
 3. Copy the `stretch-re1-<xxxx>` directory, where `<xxxx>` is your robot's serial number, to a USB key.
    - For example, if you're copying the system version, you can run a command similar to `cp -r /etc/hello-robot/stretch-re1-<xxxx> /media/$USER/<USBKEY>` from the command line, where `<USBKEY>` and `<xxxx>` is replaced with the mounted USB key's name and the robot's serial number, respectively.
    - Or, you can open the file explorer to copy the directory.

If your previous partition is corrupted or inaccessible, contact Hello Robot support and they will be able to supply an older version of the `stretch-re1-<xxxx>` directory.

## Setup the BIOS

This step can be skipped if your robot had an existing software install on it. Otherwise, follow the [guide to set up the BIOS](./configure_BIOS.md).

## Install Ubuntu

Choose between the following guides based on whether you're installing Ubuntu 18.04 or Ubuntu 20.04 (see above for info on what software ships with each OS). Within these guides, you'll have the choice of whether to replace the previous OS partition or to install alongside it. If you choose to install alongside it, you'll also be able to choose the size of each partition on the hard drive.

 - [Ubuntu 18.04 Installation guide](./install_ubuntu_18.04.md).
 - [Ubuntu 20.04 Installation guide](./install_ubuntu_20.04.md).

After the Ubuntu install, the default `hello-robot` user account will be set up.

## Run the new robot installation script

Login to the `hello-robot` user account on your new Ubuntu partition, open a terminal, and install git:

```bash
sudo apt update
sudo apt install git
```

**Note**: The system may not be able to run 'apt' immediately after a reboot as the OS may be running automatic updates in the background. Typically, waiting 10-20 minutes will allow you to use 'apt' again.

Next, place the robot's calibration data in the home folder using the following steps:

 1. Boot into the robot's new Ubuntu partition and plug in the USB key that contains the backed up calibration data.
 2. Copy the `stretch-re1-<xxxx>` directory, where `<xxxx>` is your robot's serial number, from the USB key into the home folder (i.e. `/home/$USER/`).
    - For example, you can run a command similar to `cp -r /media/$USER/<USBKEY>/stretch-re1-<xxxx> /home/$USER/` from the command line, where `<USBKEY>` and `<xxxx>` are replaced with your USB key's name and your robot's serial number, respectively.
    - Or, you can open the file explorer to copy the directory.

Next, pull down the Stretch Install repository and being the installation process:

```bash
cd ~/
git clone https://github.com/hello-robot/stretch_install
cd stretch_install
git pull
git checkout feature/install_reorg
./stretch_new_robot_install.sh
```

Once the script has started, it will ask you for your robot's serial number, Y/N confirmation, and the password. Then, the script will take 20-30 minutes to complete. Once it finishes, reboot your robot.

Finally, execute the following to confirm the new robot install was set up successfully.

```bash
stretch_robot_system_check.py
```

Your robot is now configured with a new robot install! In order to set up new users on this robot install, see the [Adding a New User](./add_new_user.md) guide.

# Troubleshooting

This section provides suggestions for common errors that occur during installation. If you become stuck and don't find an answer here, please email us or contact us through the forum.

### Firmware Mismatch Error

If you are seeing the following error:
```
----------------
Firmware protocol mismatch on hello-.
Protocol on board is pX.
Valid protocol is: pX.
Disabling device.
Please upgrade the firmware and/or version of Stretch Body.
----------------
```

Your version of Stretch Body does not align with the firmware installed with your robot. Run the firmware updater tool to automatically update the firmware to the required version for your software.

```
python3 -m pip install hello-robot-stretch-factory
RE1_firmware_updater.py --recommended
```

### Homing Error

If using `stretch_robot_home.py` does not result in the robot being calibrated, try running the command again. If this does not work, try shutting down the robot, turning off the robot with the power switch, waiting for a few seconds, and then powering it on again. Then, try `stretch_robot_home.py` again. For unknown reasons, one of our robots has needed to be homed, rebooted, and homed again after following this installation guide.

### ROS Launch File Fails

The launch files have nondeterministic behavior. Sometimes they need to be run more than once for the nodes to start in a successful order that works. For example, a common symptom of a failed launch is the visualization of the robot's body appearing white and flat in RViz.
