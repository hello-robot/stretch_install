#!/bin/bash
git pull

timestamp=`date '+%Y%m%d%H%M'`;
logdir="$HOME/stretch_user/log/$timestamp"
logfile_system="$logdir/stretch_system_update.txt"
logfile_user="$logdir/stretch_user_update.txt"
logfile_debug="$logdir/stretch_debug.txt"
logzip="$logdir/stretch_logs.zip"
echo "#############################################"
echo "Starting Stretch Update"
echo "Generating log $logfile_system"
echo "Generating log $logfile_user"
echo "Generating log $logfile_debug"
echo "#############################################"
echo ""

mkdir -p $logdir
./stretch_install_system.sh |& tee $logfile_system
./stretch_install_user.sh |& tee $logfile_user
./stretch_debug.sh |& tee $logfile_debug

echo "Generating $logzip. Include with any support tickets."
zip $logzip $logfile_system $logfile_user $logfile_debug
echo ""

echo "#############################################"
echo "A full reboot is recommended. Shutdown Ubuntu"
echo "as usual, then use the power switch in the"
echo "robot's trunk to cut power. Note that the"
echo "lift will drop when power is cut. Use the"
echo "included clamp to secure the lift."
echo "#############################################"
echo ""
