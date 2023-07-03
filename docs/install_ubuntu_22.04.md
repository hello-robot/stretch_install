# Ubuntu 22.04 Installation

This guide describes how to perform an OS installation of Ubuntu 22.04 LTS onto Stretch.

## Ubuntu Image

Download the [22.04.2 amd64 Ubuntu desktop image](https://ubuntu.com/download/desktop).

Create a bootable drive with this Ubuntu image. There are many ways to do this, but the recommended way is to use [Etcher](https://www.balena.io/etcher/) on your personal machine. Open the Etcher software and follow it's instructions to create the bootable drive.

## Installation

 1. Insert a bootable drive into the USB hub in the robot's trunk, as well as a monitor and keyboard. Next, power on the robot and at the bios startup screen (shown below) press *F10* when prompted to enter the boot menu.
    - ![](./images/22.04/NUC_startup_splash.jpg)

 2. From the boot menu, select 'OS BOOTLOADER' or look for a similar option that mentions "USB", "LIVE INSTALLATION", or "UBUNTU". This will take you to the grub menu.
    - ![](./images/22.04/boot_select.jpg)

 3. From the grub menu, select 'Ubuntu' or look for a similar option that mentions "Install Ubuntu".
    - ![](./images/22.04/grub_menu.jpg)

 4. A disk errors checker will start and then the Ubuntu 22.04 installer will be launched.
    - ![](./images/22.04/installer_system_check.jpg)

 5. The first screen of the installer will prompt you to select a language for the system and choose between trying or installing Ubuntu. Select 'English' and "Install Ubuntu".
    - ![](./images/22.04/installer_language.jpg)

 6. Next you will be prompted to select a keyboard layout. Select 'English(US)'.
    - ![](./images/22.04/installer_keyboard.jpg)

 7. The next page will show a menu to select a Wifi network if you are not already connected. For a faster and more reliable install, we suggest using a wired connection if one is available to you.
    - ![](./images/20.04/installer_network.png)
    - Your connection status will be visible in the top right of the display.
        | Wifi      | Ethernet |
        | ----------- | ----------- |
        | ![](./images/20.04/wifi.png) | ![](./images/20.04/ethernet.png) |

 8. The next page configures what gets installed. Select 'Minimal Installation' under 'What apps would you like to install to start with?' Check the box next to 'Download updates while installing Ubuntu' (this option will be unavailable if there is no interent connection) and uncheck 'Install third-party software for graphics and Wi-Fi hardware and additional media formats'.
    - ![](./images/22.04/installer_software.jpg)

### Erase & Reinstall vs Install Alongside

 9. On the next page titled 'Installation type', you may choose between 'Erase disk and reinstall Ubuntu' or 'Install Ubuntu 22.04 alongside Ubuntu XX.04'. If you've already backed up data from the previous partition, or the previous partition is corrupted, select the erase & reinstall option. If you'd like to preserve your previous Ubuntu partition, select the alongside option. If you choose the alongside option, another screen will allow you to change the size of each partition. It's recommended to give each partition at least 50GB.
    | Erase & Reinstall      | Install Alongside |
    | ----------- | ----------- |
    | ![](./images/18.04/erase_reinstall_disk.png) | ![](./images/20.04/install_alongside_disk.jpg) |

 10. There will be a prompt to confirm you wish to create the appropriate partitions for the Ubuntu install. Clicking 'Continue' will begin making changes to your robot's hard drive, so ensure there is nothing on the hard drive you wish to save before selecting continue.
     - ![](./images/20.04/installer_disk_prompt.png)

 11. Next, select your timezone.
     - ![](./images/22.04/installer_timezone.jpg)

 12. Finally, enter the identifying information as written below, replacing 'stretch-rey-xxxx' with the appropriate serial number for the robot. The robot's serial number can be found on a sticker on the left wall of the robot's trunk. Also select the 'Log in automatically' option.
     - **name:** Hello Robot Inc.
     - **computer name:** stretch-rey-xxxx
     - **username:** hello-robot
     - **password:** choose your own
     - ![](./images/22.04/installer_identity.jpg)

 13. Ubuntu will now be installed.
     - ![](./images/22.04/installer_waiting.jpg)

 14. After the installation is completed, you will be prompted to restart. Select 'Restart Now'.
     - ![](./images/22.04/installer_complete.jpg)

 15. Remove the installation medium and press ENTER to restart.
     - ![](./images/22.04/installer_removeusb.jpg)

**Ubuntu 22.04 is now installed successfully.**
