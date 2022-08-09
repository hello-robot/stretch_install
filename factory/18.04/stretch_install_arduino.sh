#!/bin/bash

ARDUINOCLI_YAML = "
board_manager:
  additional_urls: []
daemon:
  port: '50051'
directories:
  data: $HOME/.arduino15
  downloads: $HOME/.arduino15/staging
  user: $HOME/repos/stretch_firmware/arduino
library:
  enable_unsafe_install: false
logging:
  file: ''
  format: text
  level: info
sketch:
  always_export_binaries: false
telemetry:
  addr: :9090
  enabled: true
"

curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin/ sh
$HOME/.local/bin/arduino-cli config init
$HOME/.local/bin/arduino-cli core install arduino:samd@1.6.21
echo $ARDUINOCLI_YAML >> ~/.arduino15/arduino-cli.yaml
