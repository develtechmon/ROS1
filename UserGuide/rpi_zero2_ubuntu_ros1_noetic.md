# Getting Started

This is userguide on how to install `ros1 noetic` into `raspberry pi zero 2`.

## Hardware Requirements
* 64 GB SD Card
* Recommended power supply

## Software Requirements
* Install `rpi imager` from official raspberry portal
* Please use `Ubuntu Server 20.04.5 LTS (32) bit` which support RPI 2 Zero


## Setup Wifi Connection using `Command`

Please refer to this link on how to connect from command line
```
https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line
```
* sudo vi /etc/netplan/50-cloud-init.yaml (and insert WIFI and password details). See below example.
  ```
    wifis:
        wlan0:
            optional: true
            access-points:
                "SSID-NAME-HERE":
                    password: "PASSWORD-HERE"
            dhcp4: true

  ```
* save and quit
* sudo netplan apply
* sudo netplan --debug apply


## Install ROS1 Noetic

Please refer to below link for details on how to install `ros noetic`
```
https://roboticsbackend.com/install-ros-on-raspberry-pi-3/
```

Enable and allow below repository
```
$ sudo add-apt-repository universe
$ sudo add-apt-repository restricted
$ sudo add-apt-repository multiverse
```

Execute 3 command in terminals
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt install curl # if you haven't already installed curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Now update the sources to find new packages
```
$ sudo apt update
```

Install `ROS Core` packages
```
sudo apt install ros-noetic-ros-base
```

Desktop Install
```
sudo apt install ros-noetic-desktop
```

Desktop-Full
```
sudo apt install ros-noetic-desktop-full
```

## Setup Rosdep

A final step before installation is complete
```
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

To install `ros` package, for example
```
sudo apt install ros-noetic-robot-state-publisher
```

## Start ROS on RPi

Simply execute
```
source /opt/ros/noetic/setup.bash
```

## Enable SSH On Focal Ubuntu

This is bit tricky because it's not very staright forward.
Please follow this step from this userguide first
```
https://github.com/develtechmon/ROS2/blob/main/UserGuide/ssh_installation.md
```

In general step as follow
```
sudo apt install openssh-server
sudo systemctl status ssh
sudo systemctl enable ssh
sudo systemctl start sh
```

Enable Firewall and open port 22
```
sudo ufw allow ssh
sudo ufw enable
sudo ufw status
```

However, afer installation, you will observe `permission denied` when you try to ssh
from your laptop to rpi zero. To solve this problem, do the following in rpi.

```
sudo vi /etc/ssh/sshd_config
```

and set the following inside this file
```
PasswordAuthentication no
ChallengeResponseAuthentication yes

save and quit
```

Run following command
```
sudo systemctl try-reload-or-restart ssh
```

Then try to ssh from your `pc` to `rpi`. It should work now.
```
ssh jlukas@192.168.8.165
```
# Install Opencv

15

You should remember that Ubuntu 20.04 uses python3 by default, because python 2 is considered deprecated.
That's why you should install this package using
```
sudo apt-get install python3-opencv
```
