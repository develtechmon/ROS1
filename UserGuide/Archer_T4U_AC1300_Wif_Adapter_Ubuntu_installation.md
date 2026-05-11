# Getting Started

This is userguide on how to install `Archer T4U` wifi adapter to Ubuntu Local Fossa. Please follow below userguide.

## Step 1 : Clone the driver package and follow the rest
```
git clone https://github.com/morrownr/88x2bu-20210702.git
cd 88x2b-20210702
sudo ./install-driver.sh
```

## Step 2: Modify the `config` file and add the `options`
```
sudo vi /etc/modprobe.d/88x2b.conf

and add the following option in this *.conf file
# Edit the following line to change, add or delete options:
options 88x2bu  rtw_switch_usb_mode=0 rtw_led_ctrl=1
options 88x2bu  rtw_power_mgnt=0
options 88x2bu  rtw_enusbss=0
options 88x2bu  rtw_roaming_enable=0
options 88x2bu  rtw_country_code=MY

save and quit
```

## Step 3: Load the `driver` in terminal and reboot.
```
sudo modprobe -r 88x2bu
sudo modprobe 88x2bu
sudo reboot now

```

## Step 4 : Connect to the wifi using following command.
```
nmcli device status
nmcli connection show
nmcli device wifi rescan
nmcli device wifi list
nmcli connection delete "Galaxy S24 Ultra 0997"
nmcli connection connect "Galaxy S24 Ultra 0997"
nmcli device wifi connect "Galaxy S24 Ultra 0997"
```
