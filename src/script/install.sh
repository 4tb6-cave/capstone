#!/bin/bash

#To run: `sudo ./install.sh`
#Sets up device environment on an Ubuntu host
# 1. sets up Python environment
# 2. configures cave.py to run on power on

pushd ~/capstone/src/scripts
apt-get -y --ignore-missing install $(< packages.list)
systemctl start docker
usermod -aG docker $USER

#startup configuration- !!expects the capstone directory to be check out at ~/
cp launch.desktop ~/.config/autostart
popd

#docker compose build to create containers- as long as its persistent between boots
#rerun whenever code updated
pushd ~/capstone/src
docker build -t cave -f docker/Dockerfile . #downloads Docker packages
popd
