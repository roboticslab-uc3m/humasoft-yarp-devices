// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <cmath> // std::isnormal

#include <yarp/os/Time.h>

#include <ColorDebug.h>

using namespace humasoft;

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
        CD_ERROR("Control timeout (did not reach target in %d seconds).\n", controlTimeout);
        cmcSuccess = false;
        stopControl();
        return;
    }

    double incl, orient;

    if (!serialStreamResponder->getLastData(&incl, &orient))
    {
        CD_WARNING("Outdated serial stream data.\n");
    }

    double targetIncl = targetPosition[0]; // FIXME
    double error = targetIncl - incl;

    if (std::abs(error) < controlEpsilon)
    {
        CD_SUCCESS("Target reached.\n");
        setCurrentState(VOCAB_CC_NOT_CONTROLLING);

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

    CD_DEBUG("incl: target %f, sensor %f, error %f, cs: %f\n", targetIncl, incl, error, cs);

    std::vector<double> xd(targetPosition);
    targetPosition[0] = cs; // FIXME

    if (!sendTargets(xd))
    {
        CD_WARNING("Command error, not updating control this iteration.\n");
    }
}

// -----------------------------------------------------------------------------
