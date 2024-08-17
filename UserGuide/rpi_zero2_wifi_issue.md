# Getting Started

When you have issue connecting to your wifi please do the following.

## Step 1 : Open `WPA` file
```
cd /etc/wpa_supplicant/

sudo vi wpa_supplicant.conf
```

and ensure you have add the `country=MY`
```
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=MY <--- This one

network={
        ssid="WLAN1-0762EH"
        psk="Lukas@92"
}
```
save and reboot

