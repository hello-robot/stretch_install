#!/bin/bash
set -e

REDIRECT_LOGDIR="$HOME/stretch_user/log"
if getopts ":l:" opt && [[ $opt == "l" && -d $OPTARG ]]; then
    REDIRECT_LOGDIR=$OPTARG
fi
REDIRECT_LOGFILE="$REDIRECT_LOGDIR/stretch_install_firmware.`date '+%Y%m%d%H%M'`_redirected.txt"

echo "###########################################"
echo "INSTALLATION OF STRETCH FIRMWARE"
echo "###########################################"
echo "Refresh path"
export PATH=${PATH}:~/.local/bin
echo "Reload udev rules"
sudo udevadm control --reload-rules
sudo udevadm control --reload
sudo udevadm trigger
echo "Add to dialout group"
groups &>> $REDIRECT_LOGFILE
exec newgrp dialout
groups &>> $REDIRECT_LOGFILE
echo "Read ttyACMx mapping"
REx_firmware_flash.py --map &>> $REDIRECT_LOGFILE
echo "Perform firmware update (this might take a while)"
REx_firmware_updater.py --install --no_prompts &>> $REDIRECT_LOGFILE
