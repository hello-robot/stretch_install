# Ubuntu 18.04 Installation

This guide describes how to perform an OS installation of Ubuntu 18.04 LTS onto Stretch.

## Ubuntu Image

Download the 18.04.1 amd64 Ubuntu desktop image by clicking this link:

[http://old-releases.ubuntu.com/releases/18.04.1/ubuntu-18.04.1-desktop-amd64.iso](http://old-releases.ubuntu.com/releases/18.04.1/ubuntu-18.04.1-desktop-amd64.iso)

Create a bootable drive with this Ubuntu image. There are many ways to do this, but the recommended way is to use [Etcher](https://www.balena.io/etcher/) on your personal machine. Open the Etcher software and follow it's instructions to create the bootable drive. There is a good video tutorial [available here](https://youtu.be/c0TK0ynXLOo) that explains the procedure.

## Installation

Insert a bootable drive into the USB hub in the robot's trunk, as well as a monitor and keyboard. Next, power on the robot and at the bios startup screen (shown below) press *F10* when prompted to enter the boot menu.

![](./images/NUC_startup.png)

!!! note

    If you're using a Bluetooth keyboard, the BIOS likely won't recognize the F10 keypress.

From the boot menu, select 'OS BOOTLOADER' or look for a similar option that mentions "USB", "LIVE INSTALLATION", or "UBUNTU".

![](./images/BIOS_boot.png)

From here, the monitor should show the grub bootloader and display a menu similar to what is shown below:

![](./images/18.04/grub.png)

From this menu select 'Install Ubuntu'. The Ubuntu 18.04 installer will be launched.

At the first screen you will be prompted to select a language for the system. Select 'English' as shown below

![](./images/18.04/installer_language.png)

Next you will be prompted to select a keyboard layout. Select 'English(US)'.

![](./images/18.04/installer_keyboard.png)

The next page will show a menu to select a wifi network if you are not already connected.

![](./images/18.04/installer_network.png)

It is suggested to use a wired connection if possible for a faster install; your connection status should be visible in the top right of the display.

![](./images/18.04/wifi.png)

![](./images/18.04/ethernet.png)

On the next page titled, 'Updates and other software', select 'Minimal Installation' under 'What apps would you like to install to start with?'

Also, check the box next to 'Download updates while installing Ubuntu' (this option will be unavailable if there is no interent connection) and, uncheck 'Install third-party software for graphics and Wi-Fi hardware and additional media formats'.

![](./images/18.04/installer_software.png)

### Erase & Reinstall vs Install Alongside

On the next page titled 'Installation type', you may choose between 'Erase disk and reinstall Ubuntu' or 'Install Ubuntu 18.04 alongside Ubuntu XX.04'. If you've already backed up data from the previous partition, or the previous partition is corrupted, select the erase & reinstall option. If you'd like to preserve your previous Ubuntu partition, select the alongside option. If you choose the alongside option, another screen will allow you to change the size of each partition. It's recommended to give each partition at least 50GB.

Here's what the Erase & Reinstall option will look like, and an screenshot of the Install Alongside option is shown below.

![](./images/18.04/erase_reinstall_disk.png)

![](./images/20.04/install_alongside_disk.jpg)

There will be a prompt to confirm you wish to create the appropriate partitions for the ubuntu install.

If you've chosen the erase & reinstall option, ensure there is nothing on the hard drive you wish to save before selecting continue

![](./images/18.04/installer_disk_prompt.png)

Next, select your timezone.

![](./images/18.04/installer_location.png)

Finally, enter the identifying information as written below, replacing 'stretch-yyy-xxxx' with the appropriate name for the robot. `yyy` is your robot model number ("re1" for a Stretch RE1, "re2" for a Stretch 2, or "se3" for a Stretch 3), and `xxxx` is your robot's serial number. The robot's serial number can be found on a sticker on the left wall of the robot's trunk.

 - **name:** Hello Robot Inc.
 - **computer name:** stretch-yyy-xxxx
 - **username:** hello-robot
 - **password:** choose your own

Also select the 'Log in automatically' option. When finished the 'Who are you' page should look like the picture below.

![](./images/18.04/installer_identity.png)

Ubuntu will now be installed.

![](./images/18.04/installing.png)

After the installation is completed, you will be prompted to remove the installation medium and restart.

![](./images/18.04/installer_finished.png)

Remove the installation medium and turn off the robot.

**Ubuntu 18.04 is now installed successfully.**

------
<div align="center"> All materials are Copyright 2020-2024 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks.</div>
