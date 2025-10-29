# To build and run container
```bash
cd src
sudo docker build -t <tag> -f docker/Dockerfile . # downloads packages, compiles TOF package
sudo docker run -it --rm -v /dev:/dev --device-cgroup-rule='c *:* rmw' <tag> bash # run container

# other useful commands

# check all running docker containers
sudo docker ps
# enter running container
sudo docker exec -it <container-name-or-id> bash

```

# Next steps
Find out how to use display  
Use docker compose  