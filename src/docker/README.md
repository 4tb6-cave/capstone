# To build and run container
```bash

cd src

# using docker compose
sudo docker compose up # starts everything
sudo docker compose down # run in another terminal to stop it
sudo docker compose up tof # just start one service
sudo docker compose run --rm debug bash # to get to command line and manually run commands

# without docker compose
sudo docker build -t <tag> -f docker/Dockerfile . # downloads packages, compiles TOF package
sudo docker run -it --rm -v /dev:/dev --device-cgroup-rule='c *:* rmw' <tag> bash # run container

# other useful commands

# check all running docker containers
sudo docker ps
# enter running container
sudo docker exec -it <container-name-or-id> bash



```

# Next steps
Add more services to docker-compose.yml for other components of the system.  
Currently, only the tof sensor package is built. Add the rest of the required packages, either in 
the same docker image or in seperate ones.  
Remove need for sudo.  