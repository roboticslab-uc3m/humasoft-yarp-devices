// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <yarp/os/Property.h>

#include <ColorDebug.h>

using namespace humasoft;

// ------------------- DeviceDriver Related ------------------------------------

bool SoftNeckControl::open(yarp::os::Searchable & config)
{
    CD_DEBUG("%s.\n", config.toString().c_str());

    std::string prefix = config.check("prefix", yarp::os::Value(DEFAULT_PREFIX), "local port prefix").asString();
    std::string remoteRobot = config.check("remoteRobot", yarp::os::Value(DEFAULT_REMOTE_ROBOT), "remote head port").asString();

    yarp::os::Property robotOptions;
    robotOptions.put("device", "remote_controlboard");
    robotOptions.put("remote", remoteRobot);
    robotOptions.put("local", prefix + remoteRobot);
    robotOptions.setMonitor(config.getMonitor(), "remoteRobot");

    if (!robotDevice.open(robotOptions))
    {
        CD_ERROR("Unable to open robot device.\n");
        return false;
    }

    if (!robotDevice.view(iEncoders))
    {
        CD_ERROR("Unable to view IEncoders in robot device.\n");
        return false;
    }

    if (!robotDevice.view(iControlMode))
    {
        CD_ERROR("Unable to view IControlMode in robot device.\n");
        return false;
    }

    if (!robotDevice.view(iPositionControl))
    {
        CD_ERROR("Unable to view IPositionControl in robot device.\n");
        return false;
    }

    int numRobotJoints;

    if (!iEncoders->getAxes(&numRobotJoints))
    {
        CD_ERROR("Unable to retrieve number of robot joints.\n");
        return false;
    }

    if (numRobotJoints != NUM_ROBOT_JOINTS)
    {
        CD_ERROR("Expected %d robot joints, got %d.\n", NUM_ROBOT_JOINTS, numRobotJoints);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::close()
{
    return stopControl() && robotDevice.close();
}

// -----------------------------------------------------------------------------

