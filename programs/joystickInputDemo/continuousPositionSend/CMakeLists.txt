cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# Start a project.
project(continuousPositionSend LANGUAGES CXX)

# Pick up our CMake modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../../../cmake/find-modules)

# Hard dependencies.
find_package(YARP 3.2 REQUIRED COMPONENTS os dev)
find_package(COLOR_DEBUG REQUIRED)

# SoftNeckControl dependencies.
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcontrol REQUIRED)

# Set up our main executable.
add_executable(continuousPositionSend continuousPositionSend.cpp)

target_link_libraries(continuousPositionSend YARP::YARP_os
                                             YARP::YARP_init
                                             YARP::YARP_dev
                                             Eigen3::Eigen
                                             FControl::fcontrol
                                             ROBOTICSLAB::ColorDebug
                                             ROBOTICSLAB::KinematicsDynamicsInterfaces
                                             ROBOTICSLAB::KinematicRepresentationLib)

include(GNUInstallDirs)

install(TARGETS continuousPositionSend
        DESTINATION ${CMAKE_INSTALL_BINDIR})
