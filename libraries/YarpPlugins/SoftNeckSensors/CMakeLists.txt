#Creation of our new device from imudevice.h and IMUdevice class within it
yarp_prepare_plugin(SoftNeckIMU
                    CATEGORY device
                    TYPE IMUdevice
                    INCLUDE imudevice.h
                    DEFAULT ON
                    #DEPENDS "EIGEN3_FOUND;ROBOTICSLAB_KINEMATICS_DYNAMICS_FOUND;fcontrol_FOUND"
#                    EXTRA_CONFIG WRAPPER=CartesianControlSolver)
)

if(NOT SKIP_SoftNeckIMU)


    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this


    yarp_add_plugin(SoftNeckIMU                       imudevice.h
                                                      DeviceDriverIMUImpl.cpp
                                                      SerialStreamResponder_IMU.cpp
                                                      Readings.cpp
                                                      PeriodicThreadIMU.cpp
                                                      )

    target_link_libraries(SoftNeckIMU                 YARP::YARP_os
                                                      YARP::YARP_dev
                                                      YARP::YARP_sig
                                                      YARP::YARP_init #key line to make network work
                                                      Eigen3::Eigen
                                                      SensorIntegration
                                                      )

    yarp_install(TARGETS SoftNeckIMU
                                      LIBRARY DESTINATION ${HUMASOFT-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                                      ARCHIVE DESTINATION ${HUMASOFT-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                                      YARP_INI DESTINATION ${HUMASOFT-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})


endif()



