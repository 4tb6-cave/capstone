#!/bin/bash

#To run: `./install.sh`
#!!! Assumes capstone is checked out at the /home/$USER directory
#Sets up device environment on an Ubuntu host
# 1. Installs Docker
# 2. Installs python packages
# 3. Moves project sources to constant directory
# 3. Builds Docker image
# 4. Sets up systemd service to run at boot
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
sudo usermod -aG docker $USER #probably spurious, cave.py will run as root
popd

#move scripts to non-user directories
pushd ~/capstone
git submodule update --init --recursive
mkdir /etc/cave
cp -r src/ /etc/cave
sudo cp src/script/cave.py /usr/bin/cave.py
popd

#docker compose build to create containers- as long as its persistent between boots
#rerun whenever code updated
pushd /etc/cave/src
docker compose -f record-compose.yml build
popd

#startup configuration
#suspend systemd process if existing
sudo systemctl disable cave
sudo systemctl stop cave
sudo cp script/cave.service /etc/systemd/system/cave.service
sudo systemctl start cave
sudo systemctl enable cave
#To check the status of the process, use: 'journalctl -f -u cave.service'

echo 'Installation complete!'
