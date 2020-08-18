#!/bin/bash

echo "###########################################"
echo "STRETCH DEBUG INFORMATION"
echo ""

echo "Installed Apt Packages"
apt list --installed
echo ""

echo "Installed Python2 Pip Packages"
python -m pip freeze
echo ""

echo "Installed Python3 Pip Packages"
python3 -m pip freeze
echo ""

echo "Output of dmesg"
dmesg
echo ""

echo "The groups the user is part of"
groups
echo ""

echo "Output of stretch_robot_system_check.py"
stretch_robot_system_check.py
echo ""

echo "Output of stretch_hardware_echo.py"
stretch_hardware_echo.py
echo ""

echo "END OF STRETCH DEBUG INFORMATION"
echo "###########################################"
echo ""
