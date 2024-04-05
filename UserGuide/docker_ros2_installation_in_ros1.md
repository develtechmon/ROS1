# Getting Started

This is my userguide based on `Articulated Robotics` docker tutorial guide on how to install
ROS2 docker in ROS1 host machine. Check out below video

```
https://www.youtube.com/watch?v=dihfA7Ol6Mw&t=384s
```

This is my setup as follow
```
Ubuntu 18.04 in my laptop
```
## Step 1: Install docker. 

To install docker in `Ubuntu 18.04` please follow below and installation step accordingly
```
sudo apt update
sudo apt install apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu bionic stable"
sudo apt update
apt-cache policy docker-ce
sudo apt install docker-ce
```
To test our `docker daemon`, open new terminal and run below command. You should see daemon
started to indicate our docker is working. Quit the process once you're done verifying.
```
sudo systemctl status docker
```

Output should be similar as follow
```
docker.service - Docker Application Container Engine
   Loaded: loaded (/lib/systemd/system/docker.service; enabled; vendor preset: enabled)
   Active: active (running) since Mon 2021-08-09 19:42:32 UTC; 33s ago
     Docs: https://docs.docker.com
 Main PID: 5231 (dockerd)
    Tasks: 7
   CGroup: /system.slice/docker.service
           └─5231 /usr/bin/dockerd -H fd:// --containerd=/run/containerd/containerd.sock
```

## Step 2: Sudo Access

By default, the `docker` command can ony run as a `root` user, or by a user in the `docker` group.
You can refer to below link to properly setup and install your docker.
```
https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-18-04
```

If you attempt to run the `docker command` without sudo or without in the docker group, you'll get an output
as follow
```
Output
docker: Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: Post "http://%2Fvar%2Frun%2Fdocker.sock/v1.24/containers/create": dial unix /var/run/docker.sock: connect: permission denied.
See 'docker run --help'.
```

To avoid typing `sudo` command whenver you run the docker command, add your `username` to the docker group as follow.
```
sudo usermod -aG docker ${USER}
```
To apply the new group membership, log out of the server and back in, or type the following
```
su - ${USER}
```
You might encounter `permission denied` when you run docker from new terminal. To solve this, run
```
su - ${USER}
```
You now should be able to run `docker`. Please confirm if you username now is added to the docker group
by typing
```
id -nG
```
While will result as follow
```
jlukas adm dialout cdrom sudo dip plugdev lpadmin sambashare docker
```

## Step 3: VS Code setup

Bear in mind, we're going to use VSCode for our container instead of running it using conventional approach.
To start, open any `docker_test` directory. This later used for mounting and 
Please install following extension in VScode
```
Dev Containers
Remote Development
```
You should be able to see some changes in VSCode once you install this extension.

## Step 4: Download 'ros2_humble' docker image
From `vscode` search for `Reopen in Container` and choose non workspace of your location which is `docker_test`.
From `search bar` search for ros and it should show you `ROS ijnek`. Select the `ROS2 humble` from the drop down list. This will pull the image from docker and it'll automatically mount on your `docker_test` directory.

## Step 4: Enable rviz2 GUI

To enable rviz2 GUI there are two approaches, both simple and easy to use. By default launching `rviz` in docker is quite problematic. To launch rviz2

Open new terminal in your local machine and run following command
```
su - ${USER}
xhost +local: <------ This the command 
```
Meanwhile in your `docker` machine, `rebuild container` again.
