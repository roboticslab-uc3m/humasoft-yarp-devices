yarp_prepare_plugin(SoftNeckControl
                    CATEGORY device
                    TYPE humasoft::SoftNeckControl
                    INCLUDE SoftNeckControl.hpp
                    DEFAULT ON
                    EXTRA_CONFIG WRAPPER=CartesianControlServer)

if(NOT SKIP_SoftNeckControl)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(SoftNeckControl SoftNeckControl.hpp
                                    DeviceDriverImpl.cpp
                                    ICartesianControlImpl.cpp
                                    SerialStreamResponder)

    target_link_libraries(SoftNeckControl YARP::YARP_OS
                                          YARP::YARP_dev
                                          ROBOTICSLAB::ColorDebug
                                          ROBOTICSLAB::KinematicsDynamicsInterfaces)

    yarp_install(TARGETS SoftNeckControl
                 LIBRARY DESTINATION ${HUMASOFT-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${HUMASOFT-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${HUMASOFT-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()

