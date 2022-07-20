![](./images/banner.png)

## Overview

The stretch_install repository provides scripts required to install the Stretch software. The primary install scripts are:

| Script                           | Use                                                          |
| ------------------------------   | ------------------------------------------------------------ |
| stretch_new_robot_install.sh     | Installs the entire Stretch software stack on a fresh OS install of Ubuntu LTS |
| stretch_new_user_install.sh      | Installs the Stretch packages and robot configuration required for a new Ubuntu user account |
| stretch_new_dex_wrist_install.sh | Configures the robot to work with a new DexWrist (see [User Guide here](https://docs.hello-robot.com/dex_wrist_user_guide/) first) |
| stretch_update.sh                | Updates system packages (apt) and user packages (pip). Updates ROS and related packages. Run periodically when Stretch packages are available |

It is expected that `stretch_new_user_install.sh` and `stretch_update.sh` may be run periodically. In contrast, running `stretch_new_robot_install.sh` should be a rare event and should only be done under the guidance of Hello Robot support.

## Guides

| Guide                                       | Purpose                                                                    |
| ------------------------------------------- | -------------------------------------------------------------------------- |
| [Adding a New User](./docs/add_new_user.md) | Creating a new Ubuntu user and setting it up with Stretch packages and robot configuration |
| [Updating software](./docs/updating_software.md) | Updating the various components of Stretch's software + troubleshooting |

## Factory Install 


A fresh OS install should only be done under the guidance of Hello Robot Support. The steps are:

* [Setup the BIOS](./docs/configure_BIOS.md)  (only necessary for NUCs not previously configured by Hello Robot)
* [Install  Ubuntu 18.04LTS](./docs/install_ubuntu_18.04.md)

First you will need to know the serial number of your robot (eg, stretch-re1-1001). 

Login to the `hello-robot` user account and install git

```bash
sudo apt install git
```

**Note**: The system may not be able to run 'apt-get' immediately after a reboot as the OS may be running automatic updates in the background.

Next, pull down the `stretch_install` repository and being the installation process:

```bash
cd ~/
git clone https://github.com/hello-robot/stretch_install
cd stretch_install/factory
./stretch_install_factory.sh
```




## License

All Hello Robot installation materials are released under the GNU General Public License v3.0 (GNU GPLv3). Details can be found in the LICENSE file.

