## yarp-devices: Installation from Source Code

First install the dependencies:

- [Install CMake 3.5+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-cmake.md/)
- [Install YCM 0.10+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-ycm.md/)
- [Install YARP 3.2+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md/)
- [Install color-debug](https://github.com/roboticslab-uc3m/color-debug)

The SoftNeckControl YARP device requires the following additional packages:
- [Install Eigen3](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-eigen.md)
- [Install kinematics-dynamics](https://github.com/roboticslab-uc3m/kinematics-dynamics)
- [Install fcontrol](https://github.com/munozyanez/fcontrol)

### Install Additional Libraires: ACE

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

### Install yarp-devices on Ubuntu (working on all tested versions)

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

For additional options use `ccmake` instead of `cmake`.
