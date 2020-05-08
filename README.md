![](./images/HelloRobotLogoBar.png)

## Overview

The stretch_install repository provides scripts required to install the Stretch software. There are three primary install scripts:

| Script                     | Use                                                          |
| -------------------------- | ------------------------------------------------------------ |
| stretch_install_factory.sh | Installs system level robot data from factory image. Run only after a fresh OS install of Ubuntu 18.04LTS. |
| stretch_install_system.sh  | Installs system wide packages via apt-get. Installs ROS and related packages. Run after stretch_install_factory.py or when package update is required. |
| stretch_install_user.sh    | Installs robot factory image to local user account. Installs user Python packages and tools to allow robot development. Run when creating a new user account. |

It is expected that the User install will be run periodically as new users begin working with the robot. In contrast,  it is expected that the System and Factory installs are rarely run and only by qualified administrators.

## User Install 

While logged in as an administrator, make a new user accout:
```bash
sudo adduser <new_user>
```

Ensure the user has sudo privileges:

```bash
sudo usermod -aG sudo <new_user>
```

Logout and the log back in as the new user. Then pull down the Stretch_Install repository and run the install script

```bash
cd ~/
git clone https://github.com/hello-robot/stretch_install
cd stretch_install
./stretch_install_user.sh
```

This will install the packages and setup the robot configuration correctly for a new user on a Stretch RE1 computer.

Reboot your computer. After power up, check that the new install worked. For example:
```bash
>> source ~/.bashrc
>> stretch_robot_system_check.py
```

## System Install 

If running after a Factory install, first install git

```bash
sudo apt install git
```

If stretch_install isn't yet present, first install it. Then run the script.

```bash
cd ~/
git clone https://github.com/hello-robot/stretch_install
cd stretch_install
./stretch_install_system.sh
```

**Note**: When booting into Ubuntu immediately after Factory Install the system may not be able to run 'apt-get' immediately, as the OS is running automatic updates in the background.

## Factory Install 

More coming soon:

* Installing Ubuntu 18.04LTS

* Bios setup

Run the Factory install. You will need to know the serial number of your robot (eg, stretch-re1-1001).

```bash
cd ~/
git clone https://github.com/hello-robot/stretch_install
cd stretch_install/factory
./stretch_install_factory.sh
```



# Release Notes

Version 0.0.2 (2020/05/7)

* Add plugdev to user groups to ensure serial access on new accounts

Version 0.0.1 (2020/05/6)

* Initial product release.


## License

All Hello Robot installation materials are released under the GNU General Public License v3.0 (GNU GPLv3). Details can be found in the LICENSE file.

