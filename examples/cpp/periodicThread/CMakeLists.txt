cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(periodicThread LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.3 REQUIRED COMPONENTS os dev)
endif()

if(NOT TARGET ROBOTICSLAB::KinematicsDynamicsInterfaces)
    find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
endif()

add_executable(periodicThread periodicThread.cpp)

target_link_libraries(periodicThread YARP::YARP_os
                                     YARP::YARP_init
                                     YARP::YARP_dev
                                     ROBOTICSLAB::KinematicsDynamicsInterfaces)
