// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftArmControl.hpp"

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <yarp/os/LogStream.h>

using namespace sofia;

// ------------------- DeviceDriver Related ------------------------------------

bool SoftArmControl::open(yarp::os::Searchable & config)
{
    yDebug("%s.\n", config.toString().c_str());

    std::string prefix = config.check("prefix", yarp::os::Value(DEFAULT_PREFIX), "local port prefix").asString();
    std::string remoteRobot = config.check("remoteRobot", yarp::os::Value(DEFAULT_REMOTE_ROBOT), "remote head port").asString();

    double sensorTimeout = config.check("sensorTimeout", yarp::os::Value(DEFAULT_SENSOR_TIMEOUT), "sensor timeout (seconds)").asFloat64();
    cmcPeriod = config.check("cmcPeriod", yarp::os::Value(DEFAULT_CMC_PERIOD), "CMC period (seconds)").asFloat64();
    waitPeriod = config.check("waitPeriod", yarp::os::Value(DEFAULT_WAIT_PERIOD), "CMC wait check period (seconds)").asFloat64();

    geomA = config.check("geomA", yarp::os::Value(DEFAULT_GEOM_A), "distance between A and base (meters)").asFloat64();
    geomB = config.check("geomB", yarp::os::Value(DEFAULT_GEOM_B), "distance between B and mobile platform (meters)").asFloat64();
    geomL0 = config.check("geomL0", yarp::os::Value(DEFAULT_GEOM_L0), "arm length (meters)").asFloat64();
    geomLg0 = config.check("geomLg0", yarp::os::Value(DEFAULT_GEOM_LG0), "arm offset or thread length (meters)").asFloat64();
    winchRadius = config.check("radiusWinch", yarp::os::Value(DEFAULT_WINCH_RADIUS), "winch radius (meters)").asFloat64();

    if (config.check("tableIk", csvTableIk)){
        yInfo() << "Loaded IK table at: " << csvTableIk;
    }
    else
    {
        csvTableIk = NULL;
    }

    yarp::os::Property robotOptions;
    robotOptions.put("device", "remote_controlboard");
    robotOptions.put("remote", remoteRobot);
    robotOptions.put("local", prefix + remoteRobot);
    robotOptions.setMonitor(config.getMonitor(), "remoteRobot");


    if (!robotDevice.open(robotOptions))
    {
        yError() <<"Unable to open robot device.";
        return false;
    }

    if( ! robotDevice.isValid() )
    {
        std::printf("%s not available.\n", remoteRobot.c_str());
        robotDevice.close();
        yarp::os::Network::fini(); //disconnect from the YARP network
        return false;
    }

    if (!robotDevice.view(iEncoders))
    {
        yError() <<"Unable to view IEncoders in robot device.";
        return false;
    }

    if (!robotDevice.view(iControlMode))
    {
        yError() <<"Unable to view IControlMode in robot device.";
        return false;
    }

    if (!robotDevice.view(iPositionControl))
    {
        yError() <<"Unable to view IPositionControl in robot device.";
        return false;
    }

    if (!robotDevice.view(iVelocityControl))
    {
        yError() <<"Unable to view IVelocityControl in robot device.";
        return false;
    }

    if (!robotDevice.view(iTorqueControl))
    {
        yError() <<"Unable to view ITorqueControl in robot device.";
        return false;
    }

    if (!robotDevice.view(iPositionDirect))
    {
        yError() <<"Unable to view IPositionDirect in robot device.";
        return false;
    }

    int numRobotJoints;

    if (!iEncoders->getAxes(&numRobotJoints))
    {
        yError() <<"Unable to retrieve number of robot joints.";
        return false;
    }

    if (numRobotJoints != NUM_ROBOT_JOINTS)
    {
        yError("Expected %d robot joints, got %d.\n", NUM_ROBOT_JOINTS, numRobotJoints);
        return false;
    }

    qRefSpeeds.resize(numRobotJoints);

    if (!iPositionControl->getRefSpeeds(qRefSpeeds.data()))
    {
        yError() <<"Unable to retrieve reference speeds.";
        return false;
    }

    // New Yarp Sensor
    if (config.check("remoteSensor", "remote yarp port sensor")){
        std::string remoteSensorPort = config.find("remoteSensor").asString();

        if (!sensorPort.open(prefix + "/sensor:i"))
        {
            yError() <<"Unable to open local sensor port.";
            return false;
        }

        if (!yarp::os::Network::connect(remoteSensorPort, sensorPort.getName(), "udp")) // remoteSerial + "/out"
        {
            yError() <<"Unable to connect to remote sensor port.";
            return false;
        }
        sensorStreamResponder = new SensorStreamResponder(sensorTimeout);
        sensorPort.useCallback(*sensorStreamResponder);
    }


    if (cmcPeriod != DEFAULT_CMC_PERIOD)
    {
        yarp::os::PeriodicThread::setPeriod(cmcPeriod);
    }

    //testingFile.open("data.txt", ofstream::out);
    setupControllers();
    return yarp::os::PeriodicThread::start();
}

// -----------------------------------------------------------------------------

bool SoftArmControl::close()
{
    stopControl();
    yarp::os::PeriodicThread::stop();    
    delete sensorStreamResponder;
    robotDevice.close();
    sensorPort.close();
    //testingFile.close();
    return true;
}

// -----------------------------------------------------------------------------
