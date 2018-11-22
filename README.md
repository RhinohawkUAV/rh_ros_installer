# Installer for Rhinohawk ROS

This repo contains an installer for setting up a new system to run [Rhinohawk ROS](https://github.com/RhinohawkUAV/rh_ros). It's intended for provisioning fresh virtual machines and embedded devices. 

:warning: Use the installer at your own risk! :warning:

The installer will modify your system by installing many packages and checking out code into your home directory. 

We encourage you to read through the install script before running it.

## Preparing your system

1) The best way to run this system for development is by using a virtual machine, for example, VirtualBox, Parallels, or VMWare Fusion. Alternatively, you can install on bare metal, such as an UpBoard or any other PC capable of running Linux.
2) Download and install a version of Ubuntu 16. We recommend [Lubuntu 16.04](http://cdimage.ubuntu.com/lubuntu/releases/16.04/release).
3) If promoted after installation, do NOT upgrade to the next version of Ubuntu. Rhinohawk will only run on Ubuntu 16.
4) Install git (`sudo apt-get install -y git`)
5) We highly recommend that you configure a shared clipboard between the guest VM and your operating system.

## Running the installer

To begin the installation process:
```
git clone https://github.com/RhinohawkUAV/rh_ros_installer.git
cd rh_ros_installer
./install.sh
```

You'll be prompted to begin the installation, and then periodically by sudo for your password to escalate privilege.

