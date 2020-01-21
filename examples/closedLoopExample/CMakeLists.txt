cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# Start a project.
project(closedLoopExample LANGUAGES CXX)

find_package(YARP 3.2 REQUIRED COMPONENTS OS dev)
find_package(COLOR_DEBUG REQUIRED)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)

# Set up our main executable.
add_executable(closedLoopExample closedLoopExample.cpp)

target_link_libraries(closedLoopExample YARP::YARP_OS
                                        YARP::YARP_init
                                        YARP::YARP_dev
                                        ROBOTICSLAB::KinematicsDynamicsInterfaces
                                        ROBOTICSLAB::ColorDebug)

include(GNUInstallDirs)

install(TARGETS closedLoopExample
        DESTINATION ${CMAKE_INSTALL_BINDIR})
