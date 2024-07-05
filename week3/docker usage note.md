# Docker Usage Notes

Since the ego-planner project is developed based on Ubuntu 18.04 and ROS, its version is incompatible with the current Ubuntu 22.04. Therefore, I plan to use Docker for development.

The official Docker download site, [download.docker.com](http://download.docker.com/), is not accessible, and using a proxy often fails as well. So, I decided to use the Fishros download source to download the package.

**`wget http://fishros.com/install -O fishros && . fishros`**

If the Fishros toolkit cannot be used to download properly, please go to the directories /etc/apt/sources.list.d/ or /etc/apt/sources.list/ and delete the existing Docker-related apt sources. The reason is that the Fishros toolkit detects that you currently have Docker-related apt sources and attempts to download from them. After deletion, the Fishros toolkit won't detect any sources and will add Chinese mirror sites to download from.

Docker downloaded through the Fishros toolkit cannot pull images from Docker Hub normally via the terminal. You need to use Docker Desktop to pull images. First, set up a proxy in Docker Desktop to access foreign image sources.

First, find the settings button in the user avatar section -> then, in the left menu, find "Resources" -> click on "Resources" and then click on the "Proxies" tab. Fill in your proxy in the "Web Server" and "Secure Web Server" fields on the right. Generally, it is [http://127.0.0.1](http://127.0.0.1/):proxy_port. You can find the proxy port in your machine's network settings, under "Network" -> "Proxies".

Then, in Docker Desktop, find the image you want to pull, set the desired tag, and click the pull button.

Since developing ego-planner requires using the visualization tool RViz, you need to use the X11 visualization tool in Docker.

```bash
sudo xhost +local:
sudo docker run -it --device=/dev/dri --group-add video --volume=/tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=$DISPLAY" --name=rocker osrf/ros:melodic-desktop-full /bin/bash

```

This is the code needed to set up the relevant Docker container. Although I haven't successfully used this container, theoretically it should work. X11 is the related visualization protocol. The `-it` flag indicates an interactive terminal, corresponding to `/bin/bash` that follows. The `env` sets the container's DISPLAY variable to be consistent with the host. You might also need to open a port for this container, using the `-p 80:80` command to map the host's port 80 to the container's port 80. However, Docker Desktop shows that the image I am using does not support port mapping. After experiments, I found that this indeed doesn't work, which might be the reason why the container cannot be visualized, although the exact reason is unknown.

Therefore, I turned to using a virtual machine for Ubuntu 18.04 development. The virtual machine's snapshot and restore capabilities can help in testing difficult-to-configure environments, which is very helpful for using the older Ubuntu 18.04 version. The reason is that the Python version may struggle to support other software requirements.