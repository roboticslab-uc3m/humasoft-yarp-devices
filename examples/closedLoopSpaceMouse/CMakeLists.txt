cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# Start a project.
project(closedLoopExample LANGUAGES CXX)

find_package(YARP 3.2 REQUIRED COMPONENTS OS dev)
find_package(COLOR_DEBUG REQUIRED)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)

# Set up our main executable.
add_executable(closedLoopSpaceMouse closedLoopSpaceMouse.cpp)

target_link_libraries(closedLoopSpaceMouse YARP::YARP_OS
                                           YARP::YARP_init
                                           YARP::YARP_dev                                           
                                           ROBOTICSLAB::ColorDebug
                                           ROBOTICSLAB::KinematicsDynamicsInterfaces
                                           ROBOTICSLAB::KinematicRepresentationLib)

include(GNUInstallDirs)

install(TARGETS closedLoopSpaceMouse
        DESTINATION ${CMAKE_INSTALL_BINDIR})
