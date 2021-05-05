#!/bin/bash

curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin/ sh
arduino-cli config init
arduino-cli core install arduino:samd@1.6.21
cp arduino-cli.yaml ~/.arduino15/
