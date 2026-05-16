# Getting Started

This is userguide on how to `attach` PIXHAWk from windows to wsl.

## Step 1 : Install the USBIPD tool
Open PowerShell as an Administrator and install the tool by running the following command:
```
winget install --id Microsoft.winget.USBIPD
```

Restart your terminal to complete the installaation

## Step 2: Share the USB device in windows

1. Make sure your WSL distro is running (open your Ubuntu or Linux terminal)
2. Open PowerShell as an Administrator and list your connected USB devices with:
`usbipd list`

You will see below output
```
BUSID  VID:PID    DEVICE                                                        STATE
1-1    26ac:0011  USB Serial Device (COM3)                                      Attached
1-2    1532:00b8  USB Input Device, Razer Viper V3 HyperSpeed                   Not shared
1-3    0489:e0f6  MediaTek Bluetooth Adapter                                    Not shared
1-4    0b05:19b6  USB Input Device                                              Not shared
1-5    0b05:193b  USB Input Device                                              Not shared
2-1    3277:0051  USB2.0 FHD UVC WebCam, USB2.0 IR UVC WebCam, Camera DFU D...  Not shared
```
Identify the BUSID of the USB device you want to attach and share it by running:
`usbipd bind --busid <BUSID>`
```
usbipd bind --busid 1-1
```

## Step 3: Attach the device to WSL
1. In the same Administrator PowerShell, attach the device to your WSL instance using:
usbipd attach --wsl --busid <BUSID>
```
usbipd attach --wsl --busid 1-1
```
2. Verify the device is connected inside WSL by running:
`lsusb`. You will see
```
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 005: ID 26ac:0011 3D Robotics PX4 FMU v2.x <------
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

Then verify in WSL2
```
ls /dev/ttyACM0
sudo chmod 666 /dev/ttyACM0
```
## Step 4: Detach the device

When you are finished using the USB device in WSL, unplug it physically or run the following command in an Administrator PowerShell on Windows:
`usbipd detach --busid <BUSID> `
```
usbipd detach --busid 1-1
```

## Step 5: Connect Pixhawk with Gazebo

1. Step 1: Build Gazebo Classic Plugins (one time)
```
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic
```

2. Edit iri_hitl.sdf
```
vi ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_hitl/iris_hitl.sdf
```
Find and set
```
<serialEnabled>1</serialEnabled>
<serialDevice>/dev/ttyACM0</serialDevice>
<baudRate>921600</baudRate>
<hil_mode>1</hil_mode>
```
save and exit.

3. Then follow this to launch
```
cd ~/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash \
  $(pwd) $(pwd)/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world --verbose
```

You will see the serial device connected.
