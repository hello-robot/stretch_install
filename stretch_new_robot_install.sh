#!/bin/bash
set -e
set -o pipefail

do_factory_install='false'
if getopts ":f:" opt && [[ $opt == : ]]; then
    do_factory_install='true'
fi

echo "#############################################"
echo "Starting new robot install."
echo "#############################################"

timestamp='stretch_install_'`date '+%Y%m%d%H%M'`;
logdir="$HOME/stretch_user/log/$timestamp"
logfile_initial="$logdir/stretch_initial_configuration.txt"
logfile_system="$logdir/stretch_system_install.txt"
logfile_user="$logdir/stretch_user_install.txt"
logfile_dev_tools="$logdir/stretch_dev_tools_install.txt"
logzip="$logdir/stretch_logs.zip"
mkdir -p $logdir

echo ""
echo "Running initial configuration script (will log to $logfile_initial)..."
cd $HOME/stretch_install/factory
if $do_factory_install; then
    ./stretch_setup_new_robot.sh |& tee $logfile_initial
else
    ./stretch_setup_existing_robot.sh |& tee $logfile_initial
fi

echo ""
echo "Running system install script (will log to $logfile_system)..."
cd $HOME/stretch_install/factory
./stretch_install_system.sh |& tee $logfile_system

echo ""
echo "Running user install script (will log to $logfile_user)..."
cd $HOME/stretch_install
./stretch_new_user_install.sh |& tee $logfile_user


if $do_factory_install; then
    echo ""
    echo "Running dev tools install script (will log to $logfile_dev_tools)..."
    cd $HOME/stretch_install/factory
    ./stretch_install_dev_tools.sh |& tee $logfile_dev_tools
fi

echo ""
echo "Generating $logzip. Include with any support tickets."
zip $logzip $logfile_initial $logfile_system $logfile_user $logfile_dev_tools

echo ""
echo "#############################################"
echo "Done. A full reboot is recommended."
echo "#############################################"
echo ""

