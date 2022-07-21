#!/bin/bash
set -e

do_factory_install='false'
if getopts ":f:" opt && [[ $opt == : ]]; then
    do_factory_install='true'
fi

echo "#############################################"
echo "Starting installation for a new robot."
echo "#############################################"
cd $HOME/stretch_install/factory
if $do_factory_install; then
    ./stretch_setup_new_robot.sh
else
    ./stretch_setup_existing_robot.sh
fi
# Check if the previous script exit with failure
if [ $? -ne 0 ]
then
	exit 1
fi

echo "#############################################"
echo "Done with initial setup. Starting software install."
echo "#############################################"

timestamp='stretch_install_'`date '+%Y%m%d%H%M'`;
logdir="$HOME/stretch_user/log/$timestamp"
logfile_system="$logdir/stretch_system_install.txt"
logfile_user="$logdir/stretch_user_install.txt"
logfile_dev_tools="$logdir/stretch_dev_tools_install.txt"
logzip="$logdir/stretch_logs.zip"
echo "#############################################"
echo "Generating log $logfile_system"
echo "Generating log $logfile_user"
echo "Generating log $logfile_dev_tools"
echo "#############################################"
echo ""

mkdir -p $logdir
cd $HOME/stretch_install/factory
./stretch_install_system.sh |& tee $logfile_system
cd $HOME/stretch_install
./stretch_new_user_install.sh |& tee $logfile_user
if $do_factory_install; then
	cd $HOME/stretch_install/factory
	./stretch_install_dev_tools.sh |& tee $logfile_dev_tools
fi
echo "Generating $logzip. Include with any support tickets."
zip $logzip $logfile_system $logfile_user $logfile_dev_tools
echo ""

echo "#############################################"
echo "Done. A full reboot is recommended."
echo "#############################################"
echo ""
