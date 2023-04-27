cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# Start a project.
project(threadVelocityControl LANGUAGES CXX)

# Pick up our CMake modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../../cmake/find-modules)

# Dependencies
find_package(YARP 3.2 REQUIRED COMPONENTS os dev)
find_package(Eigen3 REQUIRED)
find_package(fcontrol REQUIRED)

# Create targets if specific requirements are satisfied.
include(CMakeDependentOption)

# Set up our main executable.
add_executable(threadVelocityControl threadVelocityControl.cpp)


target_link_libraries(threadVelocityControl 	YARP::YARP_os
						YARP::YARP_init
						YARP::YARP_dev
                                                Eigen3::Eigen
                                                FControl::fcontrol
						)
include(GNUInstallDirs)

install(TARGETS threadVelocityControl
        DESTINATION ${CMAKE_INSTALL_BINDIR})
