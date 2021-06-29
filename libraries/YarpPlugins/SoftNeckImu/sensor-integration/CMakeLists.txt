project(SensorIntegration)

# source files
aux_source_directory(. SensorIntegration_SRCS)
FILE(GLOB_RECURSE SensorIntegration_INCLUDE_LIST *.h *.hpp)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5SerialPort REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)

#upload include and link names
#set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES} ${CMAKE_CURRENT_SOURCE_DIR} ${Qt5SerialPort_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} PARENT_SCOPE)
set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES} ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
set(SUBDIR_LINK_NAMES ${SUBDIR_LINK_NAMES} ${PROJECT_NAME} PARENT_SCOPE)

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

#create the lib
ADD_LIBRARY(${PROJECT_NAME} STATIC ${SensorIntegration_SRCS} ${SensorIntegration_INCLUDE_LIST}) #Library type has been changed from SHARED to STATIC
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
qt5_use_modules(${PROJECT_NAME} Core SerialPort)
qt5_use_modules(${PROJECT_NAME} Widgets)

