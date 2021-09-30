// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <yarp/os/LogStream.h>

using namespace humasoft;

// ------------------- DeviceDriver Related ------------------------------------

bool SoftNeckControl::open(yarp::os::Searchable & config)
{
    yDebug("%s.\n", config.toString().c_str());

    std::string prefix = config.check("prefix", yarp::os::Value(DEFAULT_PREFIX), "local port prefix").asString();
    std::string remoteRobot = config.check("remoteRobot", yarp::os::Value(DEFAULT_REMOTE_ROBOT), "remote head port").asString();

    double sensorTimeout = config.check("serialTimeout", yarp::os::Value(DEFAULT_SERIAL_TIMEOUT), "serial timeout (seconds)").asFloat64();
    cmcPeriod = config.check("cmcPeriod", yarp::os::Value(DEFAULT_CMC_PERIOD), "CMC period (seconds)").asFloat64();
    waitPeriod = config.check("waitPeriod", yarp::os::Value(DEFAULT_WAIT_PERIOD), "CMC wait check period (seconds)").asFloat64();

    geomA = config.check("geomA", yarp::os::Value(DEFAULT_GEOM_A), "distance between A and base (meters)").asFloat64();
    geomB = config.check("geomB", yarp::os::Value(DEFAULT_GEOM_B), "distance between B and mobile platform (meters)").asFloat64();
    geomL0 = config.check("geomL0", yarp::os::Value(DEFAULT_GEOM_L0), "neck length (meters)").asFloat64();
    geomLg0 = config.check("geomLg0", yarp::os::Value(DEFAULT_GEOM_LG0), "neck offset (meters)").asFloat64();
    winchRadius = config.check("radiusWinch", yarp::os::Value(DEFAULT_WINCH_RADIUS), "winch radius (meters)").asFloat64();


    controlPolarKp = config.check("controlPolarKp", yarp::os::Value(DEFAULT_POLAR_CONTROLLER_KP), "polar controller Kp param").asFloat64();
    controlPolarKd = config.check("controlPolarKd", yarp::os::Value(DEFAULT_POLAR_CONTROLLER_KD), "polar controller Kd param").asFloat64();
    controlPolarExp = config.check("controlPolarExp", yarp::os::Value(DEFAULT_POLAR_CONTROLLER_EXP), "polar controller exp param").asFloat64();

    controlAzimuthKp = config.check("controlAzimuthKp", yarp::os::Value(DEFAULT_AZIMUTH_CONTROLLER_KP), "azimuth controller Kp param").asFloat64();
    controlAzimuthKd = config.check("controlAzimuthKd", yarp::os::Value(DEFAULT_AZIMUTH_CONTROLLER_KD), "azimuth controller Kd param").asFloat64();
    controlAzimuthExp = config.check("controlAzimuthExp", yarp::os::Value(DEFAULT_AZIMUTH_CONTROLLER_EXP), "azimuth controller exp param").asFloat64();

    controlType = config.check("controlType",yarp::os::Value(DEFAULT_CONTROL_TYPE),"the /type of control to be used").asString();

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

    // Old Serial IMU
    if (config.check("ImuSparkfun", "remote serial port"))
    {
        std::string remoteSerial = config.find("ImuSparkfun").asString();

        if (!sensorPort.open(prefix + "/imu:i"))
        {
            yError() <<"Unable to open local serial port.";
            return false;
        }

        if (!yarp::os::Network::connect(remoteSerial, sensorPort.getName(), "udp"))
        {
            yError() <<"Unable to connect to remote serial port.";
            return false;
        }

        sensorType = '0';
        serialStreamResponder = new IMUSerialStreamResponder(sensorTimeout);
        sensorPort.useCallback(*serialStreamResponder);
    }

    // New Yarp Sensor
    if (config.check("Imu3DMGX510", "remote yarp port of IMU sensor")){
        std::string remoteSerial = config.find("Imu3DMGX510").asString();

        if (!sensorPort.open(prefix + "/imu:i"))
        {
            yError() <<"Unable to open local serial port.";
            return false;
        }

        if (!yarp::os::Network::connect(remoteSerial, sensorPort.getName(), "udp")) // remoteSerial + "/out"
        {
            yError() <<"Unable to connect to remote serial port.";
            return false;
        }
        sensorType = '1';
        immu3dmgx510StreamResponder = new IMU3DMGX510StreamResponder(sensorTimeout);
        sensorPort.useCallback(*immu3dmgx510StreamResponder);
    }

    // Mocap Sensor
    if (config.check("Mocap", "remote yarp port of Mocap sensor")){
        std::string remoteSerial = config.find("Mocap").asString();

        if (!sensorPort.open(prefix + "/mocap:i"))
        {
            yError() <<"Unable to open local serial port.";
            return false;
        }

        if (!yarp::os::Network::connect(remoteSerial, sensorPort.getName(), "udp")) // remoteSerial + "/out"
        {
            yError() <<"Unable to connect to remote serial port.";
            return false;
        }
        sensorType = '2';
        mocapStreamResponder = new MocapStreamResponder(sensorTimeout);
        sensorPort.useCallback(*mocapStreamResponder);
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

bool SoftNeckControl::close()
{
    stopControl();
    yarp::os::PeriodicThread::stop();
    delete controllerPolar;
    delete controllerAzimuth;
    delete controllerRollFracc;
    delete controllerPitchFracc;
    delete serialStreamResponder;
    delete immu3dmgx510StreamResponder;
    robotDevice.close();
    sensorPort.close();
    //testingFile.close();
    return true;
}

// -----------------------------------------------------------------------------
