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
echo "Skipping on Ubuntu 18.04. Contact Hello Robot Support."
