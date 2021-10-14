## yarp-devices: Installation from Source Code

First install the dependencies:

- [Install CMake 3.5+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-cmake.md/)
- [Install YCM 0.10+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-ycm.md/)
- [Install YARP 3.2+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md/)

The SoftNeckControl YARP device requires the following additional packages:
- [Install Eigen3](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-eigen.md)
- [Install kinematics-dynamics](https://github.com/roboticslab-uc3m/kinematics-dynamics)
- [Install fcontrol](https://github.com/munozyanez/fcontrol)

Space Navegator:
  Install dependencies:
  ```bash
  sudo apt install libmotif-dev 
  sudo apt install libspnav-dev 
  sudo apt install spacenavd
  ```
- [Drivers](https://www.3dconnexion.es/service/drivers.html)


### Install Additional Libraries for MPU9250 sensor:
In order for YARP to recognize the serial port of the Arduino, we need to:

Install ACE library
```bash
sudo apt-get install libace-dev
````

Activate serial-port and install the changes:
```bash
cd ~/repos/yarp/build
cmake .. -DENABLE_yarpmod_serialport=ON
make -j$(nproc) # Compile
sudo make install && sudo ldconfig && cd # Install and go home
```
### Install Additional Libraries for 3DMGX510 sensor:
```bash
sudo apt-get install libqt5serialport5-dev
sudo apt remove 'libboost.*-dev' # to remove old versions
sudo add-apt-repository ppa:mhier/libboost-latest # to install last version (1.68)
sudo apt update
sudo sudo apt install libboost1.73-dev
```
### Install NatNet Library for MOCAP sensor:
```bash
cd
sudo cp repos/yarp-devices/libraries/NatNetLib/libNatNetLibShared.so /usr/local/lib
```
Set up an environment variable for defining the path to the library file directory. You may also be interested in adding the following to your ~/.bashrc or ~/.profile:
```bash
export LD_LIBRARY_PATH="/usr/local/lib"
```

## Install yarp-devices on Ubuntu (working on all tested versions)

Our software integrates the previous dependencies. Note that you will be prompted for your password upon using `sudo` a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # create $HOME/repos if it does not exist; then, enter it
git clone --recursive https://github.com/HUMASoft/yarp-devices.git  # Download yarp-devices software from the repository
cd yarp-devices; mkdir build; cd build; cmake ..  # Configure the yarp-devices software
make -j$(nproc) # Compile
sudo make install  # Install :-)
cp ../scripts/gnome/yarp-devices.desktop $HOME/Desktop/
```
In order to get resourcefinder to locate those robot-specific directories, the environment variable `YARP_ROBOT_NAME` should be set accordingly.
Add the line `YARP_ROBOT_NAME=teoSoftNeck` to `/etc/environment` or `.bashrc`

For additional options use `ccmake` instead of `cmake`.
