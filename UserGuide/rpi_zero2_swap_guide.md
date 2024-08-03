# Getting Started

This is usergude on how to perform swap using `rpi zero 2` to install big package such as numpy or 
when you install package inside `Ubuntu 20` and `ros focal` inside rpi 2

Install the package if you havent have it
```
sudo apt install dphys-swapfile
```
Then run following commadn
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
