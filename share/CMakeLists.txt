yarp_install(DIRECTORY applications/yarpmanager
             DESTINATION ${HUMASOFT-YARP-DEVICES_APPLICATIONS_INSTALL_DIR})

yarp_install(DIRECTORY contexts/launchCanBus
             DESTINATION ${HUMASOFT-YARP-DEVICES_CONTEXTS_INSTALL_DIR})

yarp_install(DIRECTORY robots/teoSoftNeck
             DESTINATION ${HUMASOFT-YARP-DEVICES_ROBOTS_INSTALL_DIR})
