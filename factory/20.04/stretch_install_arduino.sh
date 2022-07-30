#!/bin/bash

#To install IDE
# ./arduino_install_ide.sh -c 64 1.8.12 ~/
#cd ~/arduino
#sudo ./install.sh
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin/ sh
$HOME/.local/bin/arduino-cli config init
$HOME/.local/bin/arduino-cli core install arduino:samd@1.6.21
cp arduino-cli.yaml ~/.arduino15/
