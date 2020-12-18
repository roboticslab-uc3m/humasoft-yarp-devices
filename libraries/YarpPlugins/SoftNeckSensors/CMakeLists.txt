


find_package(YARP COMPONENTS os dev REQUIRED)


yarp_prepare_plugin(sensor-integration
                    CATEGORY device
                    TYPE IMUdevice
                    INCLUDE imudevice.h
#                    DEFAULT ON
                    #DEPENDS "EIGEN3_FOUND;ROBOTICSLAB_KINEMATICS_DYNAMICS_FOUND;fcontrol_FOUND"
#                    EXTRA_CONFIG WRAPPER=CartesianControlServer)
)
if(NOT SKIP_sensor-integration)



    yarp_add_plugin(sensor-integration)

    target_sources(yarp_rpLidar PRIVATE imudevice.h
                                        DeviceDriverIMUImpl.cpp)

#    target_link_libraries(sensor-integration          YARP::YARP_OS
#                                                      YARP::YARP_dev
#                                                      Eigen3::Eigen)
list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS YARP_os
                                                    YARP_sig
                                                    YARP_dev)


    yarp_install(TARGETS yarp_sensor-integration
        LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
        ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
        YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

    set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

    set_property(TARGET yarp_rpLidar PROPERTY FOLDER "Plugins/Device")
endif()







