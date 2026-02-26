#!/bin/bash

#Sets up device environment on an Ubuntu host
# 1. sets up Python environment
# 2. configures cave.py to run on power on

apt-get -y --ignore-missing install $(< packages.list)

#startup configuration- !!expects the capstone directory to be check out at ~/
cp launch.desktop ~/.config/autostart

#docker compose build to create containers- as long as its persistent between boots
#rerun whenever code updated