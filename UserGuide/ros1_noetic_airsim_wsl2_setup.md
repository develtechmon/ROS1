## Getting Started

In this userguide, i'm going to show how i install `ros1 noetic` and `Ubuntu 20.04.6 LTS` `Focal` into my `ROG Zephyrus Laptop` to setup environment with `AirSim` package for my research

### Ubuntu Focal Installation
To install `Ubuntu 20.04.6` version, you can do the following

#### Step 1: Ensure WSL is enabled (one time setup)

Open powersheel as administrator. This enables WSL and install require components and reboot if windows asked. 
```
wsl --install
```
After reboot, open powershell (admin) again to update:
```
wsl --update
wsl --set-default-version 2 <--- This sensure Ubuntu will be WSL 2 not WSL 1
```

### Step 2 : Install `Ubuntu 20.04 (Focal)`

This downloads Ubuntu 20.04 LTS (Focal Fossa) and installs it as a WSL distro.
```
wsl --install -d Ubuntu-20.04
```

### Step 3 : Launch Ubuntu Focal

After installation finishes:
```
wsl -d Ubuntu-20.04
```

### Step 4 : Verify you're on Ubuntu

Inside the Ubuntu terminal:
```
lsb_release -a
```
Expected Output
```
Description: Ubuntu 20.04 LTS
Codename:    focal
```
or use below
```
cat /etc/os-release
```

### Step 5 : Verify you are on WSL 2.

Back in PowerShell run the following command
```
wsl -l -v
```
You will see the following output where it show Version 2 to indicate WSL2
```
NAME            STATE     VERSION
Ubuntu-20.04    Running   2
```
### Step 6 : To test a GUI App (Important)

xeyes
```
sudo apt-get update && sudo apt-get upgrade
sudo apt-get install x11-apps
xeyes
```

neofetch
```
sudo apt-get install neofetch
```

terminator
```
sudo apt-get install terminator
terminator
```

### ROS Noetic Installation

### Step 1 : Perform update and upgrade 
```
sudo apt-get update -y && sudo apt-get ugprade -y
```

### Step 2 : Install Ros Noetic and Its packages

Please follow below step
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if you haven't already installed curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt upgrade -y

sudo apt install ros-noetic-desktop-full -y

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo apt install python3-rosdep

sudo rosdep init

rosdep update

sudo apt install python3-pip
whereis pip
```

Install below ros1 packages and python packages into root. But later we're going to 
reinstall it again in our `virtual` environment
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python3-wxgtk4.0 -y
sudo apt-get install git
sudo apt-get install gedit
sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins
sudo apt-get install ros-noetic-rqt-ez-publisher
rosrun rqt_ez_publisher rqt_ez_publisher --force-discover

sudo pip install pexpect
sudo pip install future
sudo pip install pyserial
sudo pip install dronekit
sudo pip install MAVProxy
sudo pip install keyboard
sudo pip install pymavlink==2.4.37
sudo pip install simple_pid
```

To ensure works with ros1 indoor drone, especially the aruco detection. We have to install this specific opencv
```
Step 1: Uninstall Current OpenCV
pip3 uninstall opencv-python opencv-contrib-python

Step 2: Install OpenCV 4.2.0
pip3 install opencv-python==4.2.0.34 opencv-contrib-python==4.2.0.34

Step 3: Verify Installation
python3 -c "import cv2; print(cv2.__version__)"
```

### Step 3 : Install Python 3.11 From Source and Install PIP

Install this prequisite packages
```
sudo apt install build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev libreadline-dev libffi-dev libsqlite3-dev wget libbz2-dev liblzma-dev -y
```

Download the python `3.11.9` from this link into your workspace
```
wget https://www.python.org/ftp/python/3.11.9/Python-3.11.9.tgz
```

and run the following command
```
tar -xf Python-3.11.9.tgz

cd Python-3.11.9

./configure --enable-optimizations

make -j12

sudo make altinstall

python3.11 --version

python3.11 -m ensurepip --upgrade

sudo apt update

sudo apt install -y build-essential cmake pkg-config     libgl1-mesa-dev libosmesa6-dev libglfw3 libglfw3-dev     libglew-dev libx11-dev libxrandr-dev libxi-dev libxinerama-dev     libxcursor-dev libfreetype6-dev libssl-dev zlib1g-dev     libbz2-dev liblzma-dev

pip install --upgrade pip setuptools setuptools_scm wheel build
```

### Ardupilot Installation

We're going to install `Ardupilot` which will be used for simulation later. Since Ubuntu 20.04 is end of support, checkout a version that still supports it. This version support ROS1 indoor drone code. Latest version of ardupilot didn't support it.

#### Step 1: Clone Ardupilot packages and install the following

Since Ubuntu 20.04 is "end of support", checkout a version that still supports it:

```
git clone https://github.com/ArduPilot/ardupilot

cd ardupilot

git checkout Copter-4.3.7

git submodule update --init --recursive

git describe --tags

./Tools/environment_install/install-prereqs-ubuntu.sh -y

. ~/.profile
```

Alternatively, you can try to install this version which is also working with ros1 indoor drone code esepcially to solve rangefinder issue.
```
git clone https://github.com/ArduPilot/ardupilot

cd ardupilot

git checkout AP_Periph-1.5.0

git submodule update --init --recursive

git describe --tags

./Tools/environment_install/install-prereqs-ubuntu.sh -y

. ~/.profile
```

## Step 2: Launch `Sim Vehicle`

Once done with the installation, we can launch the SITL using this command
```
cd ardupilot/Arducopter

cd ArduCopter/

sim_vehicle.py -w
```

### Ardupilot Gazebo Installation

Next we're going to setup `Gazebo` environment setup. Here we can display the drone in simulated environment

### Step 1 : Clone Ardupilot Gazebo packages and install the following

```
git clone https://github.com/dronedojo/ardupilot_gazebo

cd ardupilot_gazebo

mkdir build

cd build

sudo apt install cmake

# Configure with cmake
cmake ..

# Build with all cores (fast)
make -j4

or

make -j$(nproc)

# Install plugin system-wide
sudo make install

clear
```

### Step 2: Once done with the installation we've to modify our `bashrc` script as follow

```
sudo vi ~jlukas/.bashrc
```

add the following line into `bashrc` script
```
source /opt/ros/noetic/setup.bash
source "/home/jlukas/ardupilot/Tools/completion/completion.bash"

export GAZEBO_RESOURCE_PATH="/home/jlukas/ardupilot_gazebo":"/home/jlukas/ardupilot_gazebo/worlds":"/usr/share/gazebo-11"
export GAZEBO_MODEL_PATH="/home/jlukas/ardupilot_gazebo/models":"/usr/share/gazebo-11/models"
source /usr/share/gazebo-11/setup.sh
```

Next we've to disable the default `gazebo 11` environment so that it won't conflict with above as follow

open thie file 
```
sudo vi  /usr/share/gazebo-11/setup.sh
```

and disable the following
```
export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://localhost:11345}
export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org
#export GAZEBO_RESOURCE_PATH=${installPrefix}/share/gazebo-11:${GAZEBO_RESOURCE_PATH} <----------- Disable this
export GAZEBO_PLUGIN_PATH=${installPrefix}/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH}
#export GAZEBO_MODEL_PATH=${installPrefix}/share/gazebo-11/models:${GAZEBO_MODEL_PATH} <----------- Disable this
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${GAZEBO_PLUGIN_PATH}
export OGRE_RESOURCE_PATH=/usr/lib/x86_64-linux-gnu/OGRE-1.9.0
```

Then source the `bashrc` to take effective environment
```
source ~jlukas/.bashrc
```

### Step 3 : Change the Gazebo world 

here we want run our prefer world as follow
```
roscd gazebo_ros
cd launch/
sudo cp -r empty_world.launch iris_world.launch
sudo gedit iris_world.launch
```

Inside `iris_world.launch` modify the following as follow, where we set it into `aruco_landing.world`.
```
<arg name="world_name" default="worlds/aruco_landing.world"/> <!-- Note: the world_name is with respect to GAZEBO_RESOURCE_PATH environmental variable -->

```
To launch our `customized world` in Gazebo. Please do the following
```
roslaunch gazebo_ros iris_world.launch
```

To launch with `ros` environment. Please do the following:
```
gazebo iris_arducopter_runaway.world
```

or 

```
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

Then launch the `SITL`
```
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

if there is no `established` connection between `sim_vehicle.py` and `gazebo`. You have to perform `cmake` again as follow:
```
cd ardupilot_gazebo
mkdir build
cd build
sudo apt install cmake
cmake ..
make -j4
sudo make install
```

We can check published topic
```
cd ~

ls

rostopic list
```

### Create Virtual Environment

To avoid conflict with `root` access system, it's recommend to work on virtual environment. Please do the following
```
mkdir airsim
cd airsim/
```

To create the enviroment please do:
```
py -m venv airsimcode
```

Then activate our environment
```
python3.11 -m venv airsimcode
source airsimcode/bin/activate
```

The install the following drone packages and RL in our virtual environment
```
pip install --upgrade pip setuptools setuptools_scm wheel build
pip list
pip install swig
pip install msgpack==0.6.2 msgpack-python==0.5.6 msgpack-rpc-python==0.4.1
pip install airsim
pip install --upgrade numpy
pip install airsim
pip uninstall numpy
pip install numpy
pip install airsim
pip install numpy
pip install airsim --no-build-isolaation

pip install --no-build-isolation --force-reinstall   "gymnasium[all]"   "stable-baselines3[extra]"   tensorboard   ale-py

sed -i 's/collections.MutableMapping/collections.abc.MutableMapping/g' ~/airsim/airsimcode/lib/python3.11/site-packages/dronekit/__init__.py

clear
pip install pexpect future pyserial dronekit
pip install MAVProxy keyboard pymavlink==2.4.37
clear -altr
ls -altr
rqt
roslaunch gazebo_ros iris_world.launch
clear

roslaunch gazebo_ros iris_world.launch

python
```


