# Getting Started

This is userguide on how to install `ros1 noetic` into `raspberry pi zero 2` that is using
`buster 10`.

## Hardware Requirements
* 64 GB SD Card
* Recommended power supply

## Software Requirements
* Install `rpi imager` from official raspberry portal
* Please use `buster 10` by referring to `raspbian_buster_download.md` guideline first.

## Install ROS1 Noetic

## Step 1: Please refer to below link for details on how to install `ros noetic`
```
https://varhowto.com/install-ros-noetic-raspberry-pi-4/
```

## Step 2: Open terminal and check for version
```
lsb_release -sc

Output is --> buster
```

## Step 3: Add `ros1 noetic` repository
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-noetic.list'
```

If the command succeeds, you won’t see any input. However, you can double check if the repo is added using the cat command:
```
cat /etc/apt/sources.list.d/ros-noetic.list
```
## Step 4: Add official ROS Key

```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

You should see below output
```
Executing: /tmp/apt-key-gpghome.SG8pNIqy0T/gpg.1.sh --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
gpg: key F42ED6FBAB17C654: public key "Open Robotics info@osrfoundation.org" imported
gpg: Total number processed: 1
gpg: imported: 1
```

## Step 5:  Pull all meta info of ROS Noetic packages

```
sudo apt update
sudo apt upgrade
```

## Step 6:  Install build dependencies on Raspberry Pi 2

```
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

```

## Step 7: Set up ROS Noetic dependency sources/repos
```
sudo rosdep init
```

You will see below output
```
Wrote /etc/ros/rosdep/sources.list.d/20-default.list
Recommended: please run

         rosdep update
```

Then inspect file in the list to ensure it's fetch from right source
```
cat /etc/ros/rosdep/sources.list.d/20-default.list
```

## Step 8: rosdep update to fetch package information from the repo

Next we run rosdep update to fetch package information from the repos that are just initialized.
```
rosdep update
```

## Step 9: Fetch and install ROS Noetic Depedencies

Before we begin, we will need to create a catkin workspace by:
```
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
```

Then we use rosinstall_generator to generate a list of Noetic dependencies for different Noetic variants, such as desktop-full, desktop, and ros_comm.

Here we will use the ros_comm variant, which has no GUI components, such as rqt and rviz. Note that wet packages are the ROS packages that have been released. There won’t be any output if it succeeds.

```
rosinstall_generator ros_comm --rosdistro noetic --deps --wet-only --tar > noetic-ros_comm-wet.rosinstall
```

Next we will use the wstool to fetch all the remote repos specified from the noetic-ros_comm-wet.rosinstall file locally to the src folder:
```
wstool init src noetic-ros_comm-wet.rosinstall
```

Then before compiling the packages in the src folder, we install all system dependencies using rosdep install:
```
rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster
```

## Step 10: Compiling Noetic Packages on RPI Zero 2

Turn off swap file

```
sudo dphys-swapfile swapoff
```

Open Swap File
```
sudo vi /etc/dphys-swapfile
```

Then change `CONF_SWAPSIZE=100` to `2048` for 2GB
```
CONF_SWAPSIZE=2048
```

Then run the setup and turn on the swap again
```
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

We can check if the swap status using free -m, you will see the `swap` line like below:
```
free -m
```

Next, we're going to compile the `ROS Noetic` package using below command. This can take a while, bear with it.
```
sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Step 11: Verify Noetic Installation

Let's source the bash
```
source /opt/ros/noetic/setup.bash
```

and run
```
roscd
```

and run below command to start `ros` service.
```
roscore
```

Yeahh !!

## Step 11: Install depedencies
```
sudo apt-get install ros-$(rosversion -d)-cv-bridge
sudo apt-get install python3-sensor-msgs
```
