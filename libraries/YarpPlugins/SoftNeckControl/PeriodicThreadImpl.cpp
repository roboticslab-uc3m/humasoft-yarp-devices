// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <cmath> // std::isnormal

#include <KinematicRepresentation.hpp>
#include <ColorDebug.h>

using namespace humasoft;
using namespace roboticslab::KinRepresentation;

// ------------------- PeriodicThread Related ------------------------------------

void SoftNeckControl::run()
{
    switch (getCurrentState())
    {
    case VOCAB_CC_MOVJ_CONTROLLING:
        !serialPort.isClosed() ? handleMovjClosedLoop() : handleMovjOpenLoop();
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
    std::vector<double> x_imu;

    if (!serialStreamResponder->getLastData(x_imu))
    {
        CD_WARNING("Outdated serial stream data.\n");
    }

    std::vector<double> xd = targetPose;
    double polarError   = xd[0] - x_imu[0];
    double azimuthError = xd[1] - x_imu[1];

    double polarCs   = polarError   > *controllerPolar;
    double azimuthCs = azimuthError > *controllerAzimuth;

    if (!std::isnormal(polarCs))
    {
        polarCs = 0.0;
    }

    if (!std::isnormal(azimuthCs))
    {
        azimuthCs = 0.0;
    }

    CD_DEBUG("- Polar:   target %f, sensor %f, error %f, cs: %f\n", xd[0], x_imu[0], polarError, polarCs);
    CD_DEBUG("- Azimuth: target %f, sensor %f, error %f, cs: %f\n", xd[0], x_imu[0], azimuthError, azimuthCs);

    xd[0] = polarCs;
    xd[1] = azimuthCs;

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
