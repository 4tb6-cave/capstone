#!/bin/bash

#To run: `./install.sh`
#Sets up device environment on an Ubuntu host
# 1. sets up Python environment
# 2. configures cave.py to run on power on
#To disable, run 'sudo systemctl disable cave'

pushd ~/capstone/src/script
#install Docker packages
if dpkg -s docker &> /dev/null; then
    echo "Docker already installed"
else
    curl -sSL https://get.docker.com | sh
fi

#install Python packages
sudo apt-get -y --ignore-missing install $(< packages.txt)
sudo systemctl start docker
sudo systemctl enable docker
sudo usermod -aG docker $USER

sudo cp cave.py /usr/bin/cave.py

#startup configuration- !!expects the capstone directory to be check out at ~/
#suspend systemd process if existing
sudo systemctl disable cave
sudo systemctl stop cave
sudo cp cave.service /etc/systemd/system/cave.service
sudo systemctl start cave
sudo systemctl enable cave
#To check the status of the process, use: 'journalctl -f -u cave.service'
popd

#docker compose build to create containers- as long as its persistent between boots
#rerun whenever code updated
pushd ~/capstone/src
docker build -t cave -f docker/Dockerfile . #downloads Docker packages
popd

echo 'Installation complete!'
