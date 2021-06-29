# sensor-integration
Library for robot sensor data integration. Functions encapsulates data from sensors and offer meaningful results for the robot.

# Dependencies

The following dependencies are needed:

* Boost

In order to install, you can use the following code in a debian-like system:

```bash
sudo apt install libboost-all-dev
```


# How to use

This library is intended for the use with CMake build system. Under CMake building, it automatically holds all the include and link directories in variables:

```cmake
SUBDIR_INCLUDE_DIRECTORIES
SUBDIR_LINK_NAMES
```

Then, assuming the library is placed at "${PROJECT_SOURCE_DIR}/lib/sensor-integration/" (for example after clone with ``git clone https://github.com/HUMASoft/CiA402Device.git ``, it is enough to add the following lines to CMakeLists.txt to add includes:

```cmake
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/sensor-integration/)
INCLUDE_DIRECTORIES(${SUBDIR_INCLUDE_DIRECTORIES})
```

Also after "add_executable( ${name} ${sourcefile} )" line, add the following to link the library:

```cmake
target_link_libraries( ${PROJECT_NAME} ${SUBDIR_LINK_NAMES} )
```


# Main classes

The library is based on the use of different classes, then the use of the functionalities will be done trough an instance of that class.
