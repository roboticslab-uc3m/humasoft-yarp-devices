cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# Start a project.
project(tightenTendons LANGUAGES CXX)

# Dependencies
find_package(YARP 3.2 REQUIRED COMPONENTS os dev)

# Set up our main executable.
add_executable(tightenTendons tightenTendons.cpp)


target_link_libraries(tightenTendons YARP::YARP_os
                                     YARP::YARP_init
                                     YARP::YARP_dev
                                     )
include(GNUInstallDirs)

install(TARGETS tightenTendons
        DESTINATION ${CMAKE_INSTALL_BINDIR})
