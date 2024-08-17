# Getting Started

This is userguide on how to setup and install `open cv` for Aruco used in `Windows` and `RPi zero 2`.

## For Windows

Spec as follow:
* Python Version : 3.8.10
* OpenCv Version : opencv-contrib-python==4.2.0.32
* Tested On : Windows 10 Machine  - 17-August-2024 (First test)

Install Using Below Command
```
py -3.8 -m pip install  opencv-contrib-python==4.2.0.32
py -3.8 -m pip install imutils
```

## For RPI Zero 2 (Bullseye 32 Bit)

Spec as follow:
* Python Version : 3.9.2
* OpenCv Version : opencv-contrib-python==4.4.0.46
* Tested On : Rpi Zero 2 Machine  - 17-August-2024 (First test) - Raspbian GNU/Liux 11 (Bullseye)

Please refer to [rpi_opencv_installation](https://github.com/develtechmon/Raspberry_Pi/blob/main/rpi_user_guide/rpi_opencv_installation.md) guide in raspberry pi repository.
Then install using following command
```
sudo python3 -m pip uninstall opencv-contrib-python
sudo python3 -m pip install opencv-contrib-python==4.4.0.46
sudo python3 -m pip install imutils
```
