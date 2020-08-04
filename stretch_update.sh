#!/bin/bash
git pull

timestamp=`date '+%Y%m%d%H%M'`;
logfile_system="$HOME/stretch_user/log/stretch_system_update_$timestamp.txt"
logfile_user="$HOME/stretch_user/log/stretch_user_update_$timestamp.txt"
echo "#############################################"
echo "Starting Stretch Update"
echo "Generating log $logfile_system"
echo "Generating log $logfile_user"
echo "#############################################"

./stretch_install_system.sh |& tee $logfile_system
./stretch_install_user.sh |& tee  $logfile_user

echo "#############################################"
echo "A full reboot is recommended. Shutdown Ubuntu"
echo "as usual, then use the power switch in the"
echo "robot's trunk to cut power. Note that the"
echo "lift will drop when power is cut. Use the"
echo "included clamp to secure the lift."
echo "#############################################"
