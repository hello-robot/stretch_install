#!/bin/bash
git pull
./stretch_install_system.sh |& tee ~/stretch_user/log/stretch_system_update.log
./stretch_install_user.sh |& tee ~/stretch_user/log/stretch_user_update.log

echo "#############################################"
echo "A full reboot is recommended. Shutdown Ubuntu"
echo "as usual, then use the power switch in the"
echo "robot's trunk to cut power. Note that the"
echo "lift will drop when power is cut. Use the"
echo "included clamp to secure the lift."
echo "#############################################"
