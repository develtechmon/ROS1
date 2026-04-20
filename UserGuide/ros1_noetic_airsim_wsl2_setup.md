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

### Step 5 : Verifiy you are on WSL 2.

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




