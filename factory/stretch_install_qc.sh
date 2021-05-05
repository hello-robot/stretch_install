#!/bin/bash

#####################################################
echo "Starting STRETCH_INSTALL_QC"
echo "To be run prior to system bringup at the factory "


#sudo apt install --yes chromium-browser
sudo snap install pycharm-community --classic
pip2 install hello-robot-stretch-factory
pip2 install gspread
pip2 install gspread-formatting
pip2 install oauth2client rsa==3.4
#oauth2client-4.1.3
#pip uninstall stretch-body
echo "Cloning repos."
cd ~/repos/
git clone https://github.com/hello-robot/stretch_fleet.git
git clone https://github.com/hello-robot/stretch_fleet_tools.git
echo "Done."
echo ""
echo "Installing Arduino CLI"
./stretch_install_arduino.sh
