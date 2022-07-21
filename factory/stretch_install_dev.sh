#!/bin/bash

#####################################################
echo "Installs common developer tools for Hello Robot internal production"


read -p "Proceed with installation (y/n)?" -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    exit 1
fi

echo "Install Typora"
wget -qO - https://typora.io/linux/public-key.asc | sudo apt-key add -
sudo add-apt-repository 'deb https://typora.io/linux ./'
sudo apt-get update
sudo apt install --yes typora

echo "Install Chromium"
sudo apt install --yes chromium-browser

echo "Install Arduino"
./stretch_install_arduino.sh

echo "Install PyCharm"
sudo snap install pycharm-community --classic

echo "Install tools for system QC and bringup "
pip2 install twine
pip2 install gspread
pip2 install gspread-formatting
pip2 install oauth2client rsa==3.4
pip3 install mkdocs mkdocs-material mkdocstrings==0.17.0 pytkdocs[numpy-style] jinja2=3.0.3

echo "Cloning repos."
cd ~/repos/
git clone https://github.com/hello-robot/stretch_install.git
git clone https://github.com/hello-robot/stretch_factory.git
git clone https://github.com/hello-robot/stretch_body.git
git clone https://github.com/hello-robot/stretch_firmware.git
git clone https://github.com/hello-robot/stretch_fleet.git
git clone https://github.com/hello-robot/stretch_fleet_tools.git
git clone https://github.com/hello-robot/hello-robot.github.io
git clone https://github.com/hello-robot/stretch_docs

echo "Done."
echo ""


# update .bashrc to add body code directory to the Python path

#echo "export PATH=\${PATH}:~/repos/stretch_body/python/bin" >> ~/.bashrc
#echo "export PATH=\${PATH}:~/repos/stretch_factory/python/tools" >> ~/.bashrc

#echo "export PYTHONPATH=\${PYTHONPATH}:~/repos/stretch_factory/python/tools" >> ~/.bashrc
#echo "export PYTHONPATH=\${PYTHONPATH}:~/repos/stretch_body/python" >> ~/.bashrc

#echo "Source new bashrc now...Done."
#echo ""
