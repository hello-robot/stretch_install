![](./images/HelloRobotLogoBar.png)

## Overview

The stretch_install repository provides scripts required to install the Stretch software. It also provides 'factory' tools for setting up a new machine. There are three primary install scripts:

| Script                     | Use                                                          |
| -------------------------- | ------------------------------------------------------------ |
| stretch_install_factory.sh | Run after a fresh OS install of Ubuntu 18.04LTS. Installs system level robot data from factory image. |
| stretch_install_system.sh  | Installs system wide packages via apt-get. Installs ROS and related packages. Run after stretch_install_factory.py or when package update is required. |
| stretch_install_factory.sh | Run when creating a new user account. Replicates robot factory image to local user account. Installs use Python packages and tools to allow robot develpment. |

## User Install 
When logged in as administrator, make a new user accout:
```
sudo adduser <new_user>
```

Ensure the user has sudo privileges:

```
sudo usermod -aG sudo <new_user>
```

Logout and the log back in as the new user. Ensure git is installed

```
sudo apt install git
```

Then pull down the Stretch_Install repository and run the install script

```bash
cd ~/
git clone https://github.com/hello-robot/stretch_install
cd stretch_install
./stretch_install_user.sh
```

This will install the packages and setup the robot configuration correctly for a new user on a Stretch RE1 computer.

Reboot your computer. After power up, check that the new install worked. For example:
```
>> source ~/.bashrc
>> stretch_robot_system_check.py
```

## System Install 

sudo apt-install git

\* updates installing, may 'Could not get lock'

## Factory Install 

Installing Ubuntu 18.04LTS

Bios setup

Running script

# Release Notes

Version 0.0.2 (2020/05/7)

* Add plugdev to user groups to ensure serial access on new accounts

Version 0.0.1 (2020/05/6)

* Initial product release.


## License

All Hello Robot installation materials are released under the GNU General Public License v3.0 (GNU GPLv3). Details can be found in the LICENSE file.

