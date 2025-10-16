# Launch files and other code for June cave expedition
We stopped using ros for the second expedition, for a couple of reasons, and so the final setup was different.
## Initial setup of Pi
Running Ubuntu 24.04 desktop on Raspberry Pi 5, 8 GB  
Install docker: https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository  
Add ssh key to account, for private git repo: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account  
``` shell
sudo apt update && sudo apt upgrade -y
git clone git@github.com:ARCO-Lab/2024-brink-caves-summer.git
cd ~/2024-brink-caves-summer/docker_record
sudo docker compose build
```
## Recording data
``` shell
cd ~/2024-brink-caves-summer
git pull origin main

cd ~/2024-brink-caves-summer/docker_record

# for default configuration (active stereo)
sudo docker compose up
# for passive stereo or other configurations if made
sudo docker compose --file compose_passive.yaml up
```
## Changing parameters
See [here](https://docs.luxonis.com/software/ros/depthai-ros/driver/#List%20of%20parameters) for all depthai parameters.
``` shell
# make a new camera configuration
cp compose_passive.yaml compose_new.yaml
cp depthai_driver/config/active_stereo.yaml depthai_driver/config/new.yaml
# change file name referenced in compose file
sed -i 's/passive_stereo.yaml/new.yaml' compose_new.yaml

# edit depthai_driver/config/new.yaml with desired parameters
```
## Copying data back to laptop
``` shell
# as desktop
scp -rC slam@192.168.6.7:~/2024-brink-caves-summer/docker_record/data_recording/data/* ~/Desktop/rosbags

# as pi
scp -rC ~/2024-brink-caves-summer/docker_record/data_recording/data/* arco3@192.168.6.249:~/Desktop/rosbags

# free up space
sudo rm -r ~/2024-brink-caves-summer/docker_record/data_recording/data/*

```