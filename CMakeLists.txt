# Copyright: Universidad Carlos III de Madrid (C) 2019
# Released under the terms of the GNU LGPL v2.1

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(HUMASOFT_YARP_DEVICES)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pick up our CMake modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/find-modules)

# Hard dependencies.
find_package(YCM 0.11 REQUIRED)
find_package(YARP 3.7 REQUIRED COMPONENTS os dev)

# SoftNeckControl/SoftArmControl dependencies.
find_package(Eigen3 REQUIRED)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
find_package(fcontrol REQUIRED)

# Always build YARP devices as MODULE libraries.
set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "Force dynamic plugins")

# Configure installation paths for YARP resources.
yarp_configure_external_installation(humasoft-yarp-devices WITH_PLUGINS)

# Standard installation directories.
include(GNUInstallDirs)

# Control where libraries and executables are placed during the build.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

# Create targets if specific requirements are satisfied.
include(CMakeDependentOption)

# Add main contents.
add_subdirectory(libraries)
add_subdirectory(programs)
add_subdirectory(share)
add_subdirectory(doc)

# Configure and create uninstall target.
include(AddUninstallTarget)
