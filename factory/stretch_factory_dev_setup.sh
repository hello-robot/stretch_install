#!/bin/bash


pip install --user --upgrade twine

#####################################################
echo "To be run after a user install "
echo "Configures to use local stretch_body"
echo "Pulls down factory repos"

read -p "Proceed with installation (y/n)?" -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    exit 1
fi

pip uninstall stretch-body
echo "Cloning repos."
cd ~/repos/
git clone https://github.com/hello-robot/stretch_factory.git
git clone https://github.com/hello-robot/stretch_install.git
git clone https://github.com/hello-robot/stretch_body.git
git clone https://github.com/hello-robot/stretch_firmware.git
git clone https://github.com/hello-robot/stretch_fleet.git
git clone https://github.com/hello-robot/stretch_sandbox.git
echo "Done."
echo ""


# update .bashrc to add body code directory to the Python path

echo "export PATH=\${PATH}:~/repos/stretch_body/python/bin" >> ~/.bashrc
echo "export PATH=\${PATH}:~/repos/stretch_factory/python/tools" >> ~/.bashrc

echo "export PYTHONPATH=\${PYTHONPATH}:~/repos/stretch_factory/python/tools" >> ~/.bashrc
echo "export PYTHONPATH=\${PYTHONPATH}:~/repos/stretch_body/python" >> ~/.bashrc

echo "Source new bashrc now...Done."
echo ""