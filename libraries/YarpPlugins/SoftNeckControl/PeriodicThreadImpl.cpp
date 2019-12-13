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
        !serialPort.isClosed() && !toggleOpenLoop ? handleMovjClosedLoop() : handleMovjOpenLoop();
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
        toggleOpenLoop = false;

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

    std::vector<double> xd = targetPose;
    std::vector<double> x_imu;

    if (!serialStreamResponder->getLastData(x_imu))
    {
        CD_WARNING("Outdated serial stream data.\n");
    }

    double error = xd[0] - x_imu[0];

    if (std::abs(error) < controlEpsilon)
    {
        CD_SUCCESS("Pitch reference reached.\n");
        xd[0] = x_imu[0];

        if (!sendTargets(xd))
        {
            CD_ERROR("Unable to toggle open-loop control.\n");
            cmcSuccess = false;
            stopControl();
            return;
        }

        toggleOpenLoop = true;
        return;
    }

    double cs = error > *controller;

    if (!std::isnormal(cs))
    {
        cs = 0.0;
    }

    CD_DEBUG("pitch: target %f, sensor %f, polarError %f, cs: %f\n", xd[0], x_imu[0], error, cs);

    xd[0] = cs;

    if (!encodePose(xd, xd, coordinate_system::NONE, orientation_system::POLAR_AZIMUTH, angular_units::DEGREES))
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
