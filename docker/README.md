# Docker for CommonRoad Search
In the following, we explain how to pull the prebuilt docker image for CommonRoad Search. Special thanks to Tom Dörr, who helped us in creating this.

### Install docker
On Ubuntu/Debian/Linux-Mint etc.:
```
sudo apt-get install docker.io
```
For other platforms, visit https://docs.docker.com/get-docker/.


### Run the docker container
In the **root folder of the CommonRoad Search repository** run the following commands, depending on your platform:

**On Linux/MacOS**:
```
sudo docker run -it -p 9000:8888 --mount src="$(pwd)",target=/commonroad/commonroad-search,type=bind gitlab.lrz.de:5005/tum-cps/commonroad-search:2025_AI
```

**On Windows**:
```
docker run -it -p 9000:8888 --mount src="%cd%",target=/commonroad/commonroad-search,type=bind gitlab.lrz.de:5005/tum-cps/commonroad-search:2025_AI
```

You can now access the Jupyter Notebook by opening `localhost:9000` in your browser.
