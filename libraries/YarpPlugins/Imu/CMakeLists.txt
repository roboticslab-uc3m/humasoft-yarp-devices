#Creation of our new device from imudevice.h and IMUdevice class within it
yarp_prepare_plugin(Imu
                    CATEGORY device
                    TYPE IMUdevice
                    INCLUDE imudevice.hpp
                    DEFAULT ON
)

if(NOT SKIP_Imu)


    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this


    yarp_add_plugin(Imu                               imudevice.hpp
                                                      DeviceDriverImuImpl.cpp
                                                      PeriodicThreadImu.cpp
                                                      Imu.cpp
                                                      )

    target_link_libraries(Imu                         YARP::YARP_os
                                                      YARP::YARP_dev
                                                      YARP::YARP_sig
                                                      YARP::YARP_init #key line to make network work
                                                      Eigen3::Eigen
                                                      SensorIntegration
                                                      )

    yarp_install(TARGETS Imu
                                      LIBRARY DESTINATION ${HUMASOFT-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                                      ARCHIVE DESTINATION ${HUMASOFT-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                                      YARP_INI DESTINATION ${HUMASOFT-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})


endif()



