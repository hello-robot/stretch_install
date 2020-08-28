#!/bin/bash

echo "#############################################"
echo "Starting installation for a new robot."
echo "#############################################"
cd $HOME/stretch_install/factory
./stretch_setup_new_robot.sh
echo "#############################################"
echo "Done with initial setup. Starting software install."
echo "#############################################"

timestamp=`date '+%Y%m%d%H%M'`;
logdir="$HOME/stretch_user/log/$timestamp"
logfile_system="$logdir/stretch_system_update.txt"
logfile_user="$logdir/stretch_user_update.txt"
logfile_qc="$logdir/stretch_user_update.txt"
logzip="$logdir/stretch_logs.zip"
echo "#############################################"
echo "Generating log $logfile_system"
echo "Generating log $logfile_user"
echo "#############################################"
echo ""

mkdir -p $logdir
cd $HOME/stretch_install
./stretch_install_system.sh |& tee $logfile_system
cd $HOME/stretch_install
./stretch_install_user.sh |& tee $logfile_user
cd $HOME/stretch_install/factory
./stretch_install_qc.sh |& tee $logfile_qc
echo "Generating $logzip. Include with any support tickets."
zip $logzip $logfile_system $logfile_user $logfile_qc
echo ""

echo "#############################################"
echo "Done. A full reboot is recommended."
echo "#############################################"
echo ""
