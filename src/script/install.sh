#!/bin/bash

#Sets up device environment on an Ubuntu host
# 1. sets up Python environment
# 2. configures cave.py to run on power on

#install docker
#install python3
#install python3-docker

apt-get -y --ignore-missing install $(< packages.list)

#startup configuration
cp launch.desktop ~/.config/autostart