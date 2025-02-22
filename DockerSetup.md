# Build the docker image and (cross-)compile the code


This tutorial was tested on Ubuntu-like distros


## Install docker


```
sudo apt install docker.io
sudo usermod -aG docker ${USER}
```


Reboot in order to update the user groups


## Get the sources


Open a terminal and create a folder `~/.config/miam_docker` to store the history of the docker commands, create a `miam_workspace` folder and download the git repo and its submodules:


```
mkdir -p ~/.config/miam_docker
mkdir -p ~/miam_workspace/src/
cd ~/miam_workspace/src/
git clone git@github.com:matthieuvigne/MiAM_robotics.git
git submodule init
git submodule update
```


## Build the image


```
docker build -t miam .  
```


Create a alias in order to launch the docker image more easily: add the following in `~/.bashrc`:


`~/.bashrc`
```
generate_miam_docker()
{
    xhost +local:docker &&
    # Connect to existing docker if it exists
   if [[ $(docker ps --filter ancestor=miam --format '{{.Names}}') ]]; then
        docker exec -it `docker ps --filter "ancestor=miam" --format '{{.Names}}'`  /bin/bash
    else
        docker run --privileged \
              --rm -e DISPLAY=$DISPLAY \
              -p 127.0.0.1:8080:8080 \
              -v ~/.ssh:/root/ssh_source:ro \
              -v ~/.config/miam_docker:/root/commandhistory:rw \
              -v /tmp/.X11-unix:/tmp/.X11-unix \
              -v ~/miam_workspace/:/miam_workspace:Z \
              -it miam
    fi
}

alias miam_docker=generate_miam_docker
```


Relaunch the terminal or `source ~/.bashrc` ; you can launch the docker image using:


```
miam_docker
``` 


## Compile the code


Inside the image, configure build the project using:


```
/miam_workspace/src/MiAM_robotics/configure_docker.sh
```


## Access Teleplot


When the simulation runs, access Teleplot within your web navigator at `http://127.0.0.1:8080/`
