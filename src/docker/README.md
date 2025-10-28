# To build and run container
```bash
cd src
sudo docker build -t <tag> -f docker/Dockerfile . # downloads packages, compiles TOF package
sudo docker run -it --rm <tag> bash # run container
```
