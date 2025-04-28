#!/bin/bash
set -o pipefail

do_factory_install='false'
do_update='false'
hello_fleet_id=''
hello_model=''
fleet_number=''

# Display help function
show_help() {
    echo "Usage: $0 [options]"
    echo
    echo "Options:"
    echo "  -h                       Display this help message"
    echo "  -f                       Run factory install (Hello Robot HQ only)"
    echo "  -u                       Run system update only"
    echo "  -m MODEL                 Set model type (stretch-re1, stretch-re2, stretch-se3)"
    echo "  -i ID                    Set 4-digit fleet ID"
    echo "  -H HELLO_FLEET_ID        Set complete fleet ID (e.g., stretch-re2-1234)"
    echo
    exit 0
}

while getopts ":fuhm:i:H:" opt; do
    case $opt in
        h)
            show_help
            ;;
        f)
            do_factory_install='true'
            ;;
        u)
            do_update='true'
            ;;
        m)
            hello_model="$OPTARG"
            ;;
        i)
            fleet_number="$OPTARG"
            ;;
        H)
            hello_fleet_id="$OPTARG"
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            show_help
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            show_help
            ;;
    esac
done

if $do_factory_install && $do_update; then
    echo "Cannot both do_factory_install and do_update."
    exit 1
fi

# Prepare HELLO_FLEET_ID
if [[ -n "$hello_fleet_id" ]]; then
    export HELLO_FLEET_ID="$hello_fleet_id"
elif [[ -n "$hello_model" && -n "$fleet_number" ]]; then
    export HELLO_FLEET_ID="${hello_model}-${fleet_number}"
    export HELLO_MODEL="$hello_model"
    export HELLO_FLEET_NUMBER="$fleet_number"
fi

# Validate HELLO_FLEET_ID if provided
if [[ -n "$HELLO_FLEET_ID" ]]; then
    if [[ ! $HELLO_FLEET_ID =~ ^stretch-(re1|re2|se3)-[0-9]{4}$ ]]; then
        echo "Error: HELLO_FLEET_ID '$HELLO_FLEET_ID' does not match required format stretch-(re1|re2|se3)-XXXX. Run 'stretch_new_robot_install.sh -h' for help."
        exit 1
    fi
    echo "Using HELLO_FLEET_ID: $HELLO_FLEET_ID"
fi

source /etc/os-release
factory_osdir="$VERSION_ID"
if [[ ! $factory_osdir =~ ^(18.04|20.04|22.04)$ ]]; then
    echo "Could not identify OS. Please contact Hello Robot Support."
    exit 1
fi


sudo echo "#############################################"
echo "STARTING NEW ROBOT INSTALL"
echo "#############################################"

timestamp='stretch_install_'`date '+%Y%m%d%H%M'`;
logdir="$HOME/stretch_user/log/$timestamp"
logfile_initial="$logdir/stretch_initial_configuration.txt"
logfile_system="$logdir/stretch_system_install.txt"
logfile_user="$logdir/stretch_user_install.txt"
logfile_dev_tools="$logdir/stretch_dev_tools_install.txt"
logfile_firmware="$logdir/stretch_firmware_install.txt"
logzip="$logdir/stretch_robot_install_logs.zip"
mkdir -p $logdir

function echo_failure_help {
    zip -r $logzip $logdir/ > /dev/null
    echo ""
    echo "#############################################"
    echo "FAILURE. INSTALLATION DID NOT COMPLETE."
    echo "Look at the troubleshooting guide for solutions to common issues: https://docs.hello-robot.com/0.3/installation/robot_install/#troubleshooting"
    echo "or contact Hello Robot support and include $logzip"
    echo "#############################################"
    echo ""
    exit 1
}

if ! $do_update; then
    cd $HOME/stretch_install/factory/$factory_osdir
    ./stretch_initial_setup.sh $do_factory_install |& tee $logfile_initial
    if [ $? -ne 0 ]; then
        echo_failure_help
    fi
fi

echo ""
cd $HOME/stretch_install/factory/$factory_osdir
./stretch_install_system.sh -l $logdir |& tee $logfile_system
if [ $? -ne 0 ]; then
    echo_failure_help
fi

echo ""
cd $HOME/stretch_install
./stretch_new_user_install.sh -l $logdir |& tee $logfile_user
if [ $? -ne 0 ]; then
    echo_failure_help
fi

if $do_factory_install; then
    echo ""
    cd $HOME/stretch_install/factory/$factory_osdir
    ./stretch_install_dev_tools.sh -l $logdir |& tee $logfile_dev_tools
    if [ $? -ne 0 ]; then
        echo_failure_help
    fi
fi

# # Update Aug 24th 2023: Automatic firmware updating during the robot install causes devices to soft brick.
# # This functionality is disabled until the problem can be identified.
# echo ""
# cd $HOME/stretch_install/factory/$factory_osdir
# bash -i ./stretch_install_firmware.sh -l $logdir |& tee $logfile_firmware
# if [ $? -ne 0 ]; then
#     echo_failure_help
# fi

zip -r $logzip $logdir/ > /dev/null

echo ""
echo "#############################################"
echo "DONE! INSTALLATION COMPLETED SUCCESSFULLY."
echo "Perform post install steps: https://docs.hello-robot.com/0.2/stretch-install/docs/robot_install/#post-install-steps"
echo "#############################################"
echo ""
