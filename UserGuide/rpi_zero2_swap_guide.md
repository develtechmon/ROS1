# Getting Started

This is usergude on how to perform swap using `rpi zero 2` to install big package such as numpy
```
sudo dphys-swapfile swapoff 

sudo vi /etc/dphys-swapfile
Under this file --> change CONF_SWAPSIZE=100 to 1024 for 1GB

save and quit

sudo dphys-swapfile setup
sudo dphys-swapfile swapon
sudo reboot
```

Please refer to below link for more details
```
Link: https://pimylifeup.com/raspberry-pi-swap-file/
```
