﻿// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftArmControl.hpp"

#include <cmath> // std::isnormal

#include <KinematicRepresentation.hpp>
#include <yarp/os/LogStream.h>

//In order to clasify the system, we are using fstream library
#include <fstream>

using namespace sofia;
using namespace roboticslab::KinRepresentation;

// ------------------- PeriodicThread Related ------------------------------------

void SoftArmControl::run()
{
    !sensorPort.isClosed() ? handleMovjClosedLoop() : handleMovjOpenLoop();
}

// -----------------------------------------------------------------------------

void SoftArmControl::handleMovjOpenLoop()
{
    bool done;
    cout << "Bucle abierto" << endl;

    if (!iPositionControl->checkMotionDone(&done))
    {
        yError() <<"Unable to query current robot state.";
        cmcSuccess = false;
        stopControl();
        return;
    }

    if (done)
    {
        setCurrentState(VOCAB_CC_NOT_CONTROLLING);

        if (!iPositionControl->setRefSpeeds(qRefSpeeds.data()))
        {
            yWarning() <<"setRefSpeeds (to restore) failed.";
        }
    }
}

// -----------------------------------------------------------------------------

void SoftArmControl::handleMovjClosedLoop()
{
    std::vector<double> x_imu;
    double pitchError,
           yawError,
           pitchCs,
           yawCs
           = 0.0;

        if (!sensorStreamResponder->getLastData(x_imu))
        {
            yWarning() <<"Outdated sensor stream data.";
        }


    std::vector<double> xd = targetPose;
    std::vector<double> cs(2);
    pitchError = xd[0] - x_imu[0];
    yawError   = xd[1] - x_imu[1];


    pitchCs = pitchError > *fraccControllerPitch;
    if (!std::isnormal(pitchCs))
    {
        pitchCs = 0.0;
    }

    cs[0] = pitchCs;

    yawCs = yawError > *fraccControllerYaw;
    if (!std::isnormal(yawCs))
    {
        yawCs = 0.0;
    }

    cs[1] = yawCs;


    yDebug("- Pitch: target %f, sensor %f, error %f, cs: %f\n", targetPose[0], x_imu[0], pitchError, pitchCs);
    yDebug("- Yaw  : target %f, sensor %f, error %f, cs: %f\n", targetPose[1], x_imu[1], yawError, yawCs);

    double p1=0.001*( cs[0] / 1.5);
    // INVERTIDO
    double p2=0.001*( - (cs[0] / 3) - (cs[1] / 1.732) ); //Antiguo tendon 3
    double p3=0.001*( (cs[1] / 1.732) - (cs[0] / 3) ); //Antiguo tendon 2

    std::vector<double> qd={p1,p2,p3};

    if (!iPositionControl->positionMove(qd.data()))
    {
        yError() <<"positionMove failed.";
    }
}
