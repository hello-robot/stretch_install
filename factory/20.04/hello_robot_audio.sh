#! /bin/bash
pactl set-default-sink alsa_output.pci-0000_00_1f.3.analog-stereo
amixer set Master 100%
canberra-gtk-play -f /usr/share/sounds/ubuntu/stereo/desktop-login.ogg

