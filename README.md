![](./images/HelloRobotLogoBar.png)

## Overview

The stretch_install repository provides scripts required to install the Stretch software. It also provides 'factory' tools for setting up a new machine.

## Installation for a New User 
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

Check that the new install worked. For example:
```
>> source ~/.bashrc
>> stretch_robot_system_check.py
```

# Release Notes
Version 0.0.1 (2020/05/6)

* Initial product release.


## License

All Hello Robot installation materials are released under the GNU General Public License v3.0 (GNU GPLv3). Details can be found in the LICENSE file.

