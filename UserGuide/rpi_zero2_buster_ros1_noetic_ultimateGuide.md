# Getting Started

This is userguide on how to flash existing `buster RPi` with `ROS1 Noetic` into `raspberry pi zero 2`.

## Hardware Requirements
* 64 GB SD Card
* Recommended power supply

## Software Requirements
* Install `balena etcher` 
* Please use `clover image v0.24`. This RPI image contains all the necessary software with less effort to setup and install.
* Please read page for details [rpi_image](https://clover.coex.tech/en/image.html)
* This image is based on `ROS Noetic` and `Python 3`. You can download this image as follow
```
https://github.com/CopterExpress/clover/releases/download/v0.24/clover_v0.24.img.zip
```
* Once done download, plese flash SD Card with above image


## Browser Login to RPI

* You should new `clover wifi`. Please connect to it and use password `cloverwifi`.
* From browser just run this IP address ` http://192.168.11.1`

## Alternative Login to RPI using SSH

* You can also login to RPI using `ssh pi@192.168.11.1`
```
login : pi
pwd : raspberry
```

# Install Drone Kit Package

Please ensure to increase  your swap file to ensure a successful installation. Please refer to  below userguide
```
https://github.com/develtechmon/ROS1/blob/master/UserGuide/rpi_zero2_swap_guide.md
```

Please ensure that you modify below file and add your wifi details to connect to internet to ensure your
RPI is in `client mode`. By default, your rpi is `access point` mode.
```
sudo vi /etc/wpa_supplicant/wpa_supplicant.conf --> To add home wifi network
sudo vi /etc/dhcpcd.conf --> Disable the static IP
```

This step is well explained here [configuring wifi](https://clover.coex.tech/en/network.html)

You should remember that this RPI Image uses python3 by default, because python 2 is considered deprecated.
That's why you should install this package using

```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python-pip3
sudo apt-get install python-dev
sudo apt-get install screen python-wxgtk4.0 python-lxml
sudo apt-get install git

sudo apt-get install git cmake
sudo apt-get install libatlas-base-dev gfortran
sudo apt-get install libhdf5-serial-dev hdf5-tools
sudo apt-get install python3-dev
sudo apt-get install nano locate
sudo apt-get install libfreetype6-dev python3-setuptools
sudo apt-get install protobuf-compiler libprotobuf-dev openssl
sudo apt-get install libssl-dev libcurl4-openssl-dev
sudo apt-get install cython3
sudo apt-get install libxml2-dev libxslt1-dev
sudo apt-get install build-essential pkg-config
sudo apt-get install libtbb2 libtbb-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install libxvidcore-dev libavresample-dev
sudo apt-get install libtiff-dev libjpeg-dev libpng-dev
sudo apt-get install python-tk libgtk-3-dev
sudo apt-get install libcanberra-gtk-module libcanberra-gtk3-module
sudo apt-get install libv4l-dev libdc1394-22-dev

sudo apt-get install python3-opencv

sudo pip3 install pexpect==4.6.0
sudo pip3 install future==0.18.3
sudo pip3 install pyserial==3.5b0
sudo pip3 install dronekit==2.9.2
sudo pip3 install MAVProxy==1.8.60
sudo pip3 install keyboard==0.13.5
sudo pip3 install numpy==1.24.3
sudo pip3 install picamera==1.13
sudo pip3 install pycoral==0.1.0
sudo pip3 install pymavlink==2.4.41
sudo pip3 install v4l2-python3==0.3.4
sudo pip3 install gpiozero==2.0
sudo pip3 install tflite-runtime==2.13.0
sudo pip3 install tflite-support==0.4.4
sudo pip3 install RPi.GPIO==0.7.1
```

