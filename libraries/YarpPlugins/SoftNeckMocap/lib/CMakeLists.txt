project(NatNetLib)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lNatNetLibShared")
find_library (NATNET_LIB NatNetLibShared)


#add_library(NatNetLibShared SHARED IMPORTED GLOBAL)
#find_library (NATNET_LIB NatNetLibShared)
#set_target_properties(NatNetLibShared PROPERTIES IMPORTED_LOCATION ${NATNET_LIB})
#target_link_libraries(${KEYWORD} NatNetLibShared)



find_package(Qt5SerialPort REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)

#upload include and link names
set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES} ${CMAKE_CURRENT_SOURCE_DIR} ${Qt5SerialPort_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} PARENT_SCOPE)
set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES} ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
set(SUBDIR_LINK_NAMES ${SUBDIR_LINK_NAMES} ${PROJECT_NAME} PARENT_SCOPE)

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

#create the lib
ADD_LIBRARY(${PROJECT_NAME} STATIC ${NATNET_LIB}) #Library type has been changed from SHARED to STATIC
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
qt5_use_modules(${PROJECT_NAME} Core SerialPort)
qt5_use_modules(${PROJECT_NAME} Widgets)
