// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <cmath> // std::isnormal

#include <yarp/os/Time.h>

#include <KinematicRepresentation.hpp>
#include <ColorDebug.h>

using namespace humasoft;
using namespace roboticslab::KinRepresentation;

// ------------------- PeriodicThread Related ------------------------------------

void SoftNeckControl::run()
{
    const int currentState = getCurrentState();

    switch (currentState)
    {
    case VOCAB_CC_MOVJ_CONTROLLING:
        serialPort.isClosed() ? handleMovjOpenLoop() : handleMovjClosedLoop();
        break;
    default:
        break;
    }
}

// -----------------------------------------------------------------------------

void SoftNeckControl::handleMovjOpenLoop()
{
    bool done;

    if (!iPositionControl->checkMotionDone(&done))
    {
        CD_ERROR("Unable to query current robot state.\n");
        cmcSuccess = false;
        stopControl();
        return;
    }

    if (done)
    {
        setCurrentState(VOCAB_CC_NOT_CONTROLLING);

        if (!iPositionControl->setRefSpeeds(qRefSpeeds.data()))
        {
            CD_WARNING("setRefSpeeds (to restore) failed.\n");
        }
    }
}

// -----------------------------------------------------------------------------

void SoftNeckControl::handleMovjClosedLoop()
{
    if (yarp::os::Time::now() - targetStart > controlTimeout)
    {
        CD_ERROR("Control timeout (did not reach target in %f seconds).\n", controlTimeout);
        cmcSuccess = false;
        stopControl();
        return;
    }

    std::vector<double> rpy;

    if (!serialStreamResponder->getLastData(rpy))
    {
        CD_WARNING("Outdated serial stream data.\n");
    }

    double error = targetRPY[1] - rpy[1];

    if (std::abs(error) < controlEpsilon)
    {
        CD_SUCCESS("Target reached.\n");
        stopControl();

        if (!iPositionControl->setRefSpeeds(qRefSpeeds.data()))
        {
            CD_WARNING("setRefSpeeds (to restore) failed.\n");
        }

        return;
    }

    double cs = error > *controller;

    if (!std::isnormal(cs))
    {
        cs = 0.0;
    }

    CD_DEBUG("pitch: target %f, sensor %f, error %f, cs: %f\n", targetRPY[1], rpy[1], error, cs);

    rpy[0] = targetRPY[0];
    rpy[1] = cs;
    rpy[2] = targetRPY[2];

    std::vector<double> xd;

    if (!encodePose(rpy, xd, coordinate_system::NONE, orientation_system::RPY, angular_units::DEGREES))
    {
        CD_ERROR("encodePose failed.\n");
        cmcSuccess = false;
        stopControl();
        return;
    }

    if (!sendTargets(xd))
    {
        CD_WARNING("Command error, not updating control this iteration.\n");
    }
}

// -----------------------------------------------------------------------------
