cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# Start a project.
project(buttonPositionSend LANGUAGES CXX)

find_package(YARP 3.2 REQUIRED COMPONENTS OS dev)
find_package(COLOR_DEBUG REQUIRED)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)

# Set up our main executable.
add_executable(buttonPositionSend buttonPositionSend.cpp)

target_link_libraries(buttonPositionSend YARP::YARP_OS
                                         YARP::YARP_init
                                         YARP::YARP_dev                                           
                                         ROBOTICSLAB::ColorDebug
                                         ROBOTICSLAB::KinematicsDynamicsInterfaces
                                         ROBOTICSLAB::KinematicRepresentationLib)

include(GNUInstallDirs)

install(TARGETS buttonPositionSend
        DESTINATION ${CMAKE_INSTALL_BINDIR})
