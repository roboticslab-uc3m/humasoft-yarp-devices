## yarp-devices: Installation from Source Code

First install the dependencies:

- [Install CMake 3.5+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-cmake.md/)
- [Install YCM 0.10+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-ycm.md/)
- [Install YARP 3.2+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md/)
- [Install color-debug](https://github.com/roboticslab-uc3m/color-debug)
- [Install kinematics-dynamics](https://github.com/roboticslab-uc3m/kinematics-dynamics) (required for SoftNeckControl device)

For unit testing, you'll need the googletest source package. Refer to [Install googletest](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-googletest.md/).

### Install yarp-devices on Ubuntu (working on all tested versions)

Our software integrates the previous dependencies. Note that you will be prompted for your password upon using `sudo` a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # create $HOME/repos if it does not exist; then, enter it
git clone --recursive https://github.com/HUMASoft/yarp-devices.git  # Download yarp-devices software from the repository
cd yarp-devices; mkdir build; cd build; cmake ..  # Configure the yarp-devices software
make -j$(nproc) # Compile
sudo make install  # Install :-)
```

For additional options use `ccmake` instead of `cmake`.
