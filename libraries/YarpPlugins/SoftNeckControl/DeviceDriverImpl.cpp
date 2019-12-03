// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <ColorDebug.h>

using namespace humasoft;

// ------------------- DeviceDriver Related ------------------------------------

bool SoftNeckControl::open(yarp::os::Searchable & config)
{
    CD_DEBUG("%s.\n", config.toString().c_str());

    std::string prefix = config.check("prefix", yarp::os::Value(DEFAULT_PREFIX), "local port prefix").asString();
    std::string remoteRobot = config.check("remoteRobot", yarp::os::Value(DEFAULT_REMOTE_ROBOT), "remote head port").asString();

    double serialTimeout = config.check("serialTimeout", yarp::os::Value(DEFAULT_SERIAL_TIMEOUT), "serial timeout (seconds)").asFloat64();
    cmcPeriod = config.check("cmcPeriod", yarp::os::Value(DEFAULT_CMC_PERIOD), "CMC period (seconds)").asFloat64();
    waitPeriod = config.check("waitPeriod", yarp::os::Value(DEFAULT_WAIT_PERIOD), "CMC wait check period (seconds)").asFloat64();

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

    if (!robotDevice.view(iPositionDirect))
    {
        CD_ERROR("Unable to view IPositionDirect in robot device.\n");
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

    qRefSpeeds.resize(numRobotJoints);

    if (!iPositionControl->getRefSpeeds(qRefSpeeds.data()))
    {
        CD_ERROR("Unable to retrieve reference speeds.\n");
        return false;
    }

    if (config.check("remoteSerial", "remote serial port"))
    {
        std::string remoteSerial = config.find("remoteSerial").asString();

        if (!serialPort.open(prefix + "/imu:i"))
        {
            CD_ERROR("Unable to open local serial port.\n");
            return false;
        }

        if (!yarp::os::Network::connect(remoteSerial + "/out", serialPort.getName(), "udp"))
        {
            CD_ERROR("Unable to connect to remote serial port.\n");
            return false;
        }

        serialStreamResponder = new SerialStreamResponder(serialTimeout);
        serialPort.useCallback(*serialStreamResponder);
    }

    if (cmcPeriod != DEFAULT_CMC_PERIOD)
    {
        yarp::os::PeriodicThread::setPeriod(cmcPeriod);
    }

    return yarp::os::PeriodicThread::start();
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::close()
{
    stopControl();
    yarp::os::PeriodicThread::stop();
    serialPort.close();
    delete serialStreamResponder;
    robotDevice.close();
    return true;
}

// -----------------------------------------------------------------------------
