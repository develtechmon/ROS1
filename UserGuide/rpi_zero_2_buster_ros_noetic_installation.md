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

## Step 8: rosde update to fetch package information from the repo

Next we run rosdep update to fetch package information from the repos that are just initialized.
```
rosdep update
```

