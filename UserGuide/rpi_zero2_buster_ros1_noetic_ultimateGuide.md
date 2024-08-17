# Getting Started

This is userguide on how to flash existing `buster RPi` with `ROS1 Noetic` into `raspberry pi zero 2`.

## 1. Hardware Requirements
* 64 GB SD Card
* Recommended power supply

## 2. Software Requirements
* Install `balena etcher` 
* Please use `clover image v0.24`. This RPI image contains all the necessary software with less effort to setup and install.
* Please read page for details [rpi_image](https://clover.coex.tech/en/image.html)
* This image is based on `ROS Noetic` and `Python 3`. You can download this image as follow
```
https://github.com/CopterExpress/clover/releases/download/v0.24/clover_v0.24.img.zip
```
* Once done download, plese flash SD Card with above image


## 3. Browser Login to RPI

* You should new `clover wifi`. Please connect to it and use password `cloverwifi`.
* From browser just run this IP address ` http://192.168.11.1`

## 4. Alternative Login to RPI using SSH

* You can also login to RPI using `ssh pi@192.168.11.1`
```
login : pi
pwd : raspberry
```

## 5. Enable Serial in RPI

To enable a successful `UART` connection, we have to enable below configuration.
* Please follow below step.

```
* sudo raspi-config
```

and then go to
```
interface_options --> Serial Port
```
You will then serve with `Would you like a login shell to be accessible over serial ?`
```
Choose <No>
```

You will then serve with `Would you like the serial port hardware to be enabled ?`
```
Choose <Yes>
```

Then `reboot` the RPI

Next, we have to disable the bluetooth as follow, you have open this file.
```
sudo vi /boot/config.txt 
```

And inside this file, `go to last line` and do the following
```
* dtoverlay=disable-bt (disable the bluetooth by adding this line)
* enable_uart = 1 (enable this line or add this line if it's missing)

* save the file and reboot RPI
```

you should see `ttyAMA0` if you run following command
```
ls /dev/ttyAMA0
```

## 6. Wifi Configuration for Client Mode

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

Then reboot your RPI

## 7. Install Drone Kit Package

You should remember that this RPI Image uses python3 by default, because python 2 is considered deprecated.
That's why you should install this package using

```
sudo apt-get update
sudo apt-get upgrade
sudo pip3 install dronekit==2.9.2
sudo pip3 install pymavlink==2.4.41
sudo pip3 install MAVProxy==1.8.60
sudo pip3 install keyboard==0.13.5

# Optional
sudo pip3 install pexpect==4.6.0
sudo pip3 install future==0.18.3
sudo pip3 install pyserial==3.5b0
sudo pip3 install numpy==1.24.3
sudo pip3 install picamera==1.13
sudo pip3 install pycoral==0.1.0
sudo pip3 install v4l2-python3==0.3.4
sudo pip3 install gpiozero==2.0
sudo pip3 install tflite-runtime==2.13.0 (To install this package in your rpi zero2, please increase the swap size to 2GB, plese refer to rpi_zero2_swap_guide)
sudo pip3 install tflite-support==0.4.4 (To install this package in your rpi zero2, please increase the swap size to 2GB, plese refer to rpi_zero2_swap_guide)
sudo pip3 install RPi.GPIO==0.7.1
```

## 8. Test Connection

To test if connection is working. Run below command using mavproxy. You should see hearbeart data from flight controller
```
mavproxy.py --master=/dev/ttyACM0,921600
```

## 9.  Wifi Configuration for Access Mode

For this settings, you just have to redo back on step 6. This way, you're connecting to RPI wifi only.



