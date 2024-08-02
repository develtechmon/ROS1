melodic installation

sudo sh -c 'echo "deb http://packages.ros.org.jsk.imi.i.u-tokyo.ac.jp/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://ros.jsk.imi.i.u-tokyo.ac.jp/jsk.key -O - | sudo apt-key add -

sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade
sudo apt install ros-melodic-ros-base
