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
        if(controlType=="docked")
            !serialPort.isClosed() ? handleMovjClosedLoopDocked() : handleMovjOpenLoop();
        else if(controlType=="undocked")
            !serialPort.isClosed() ? handleMovjClosedLoopUndocked() : handleMovjOpenLoop();
        else CD_ERROR("Control mode not defined\n");
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

void SoftNeckControl::handleMovjClosedLoopDocked()
{
    std::vector<double> x_imu;
    double polarError,
           azimuthError,
           polarCs,
           azimuthCs
           = 0.0;

    if (!serialStreamResponder->getLastData(x_imu))
    {
        CD_WARNING("Outdated serial stream data.\n");
    }

    std::vector<double> xd = targetPose;   
    polarError   = xd[0] - x_imu[0];
    azimuthError = xd[1] - x_imu[1];

    if(std::abs(azimuthError)> 180 )
    {
        if(azimuthError > 0)
            azimuthError = azimuthError - 360.0;
        else
            azimuthError = azimuthError + 360.0;
    }

    // controlamos siempre en inclinación
    polarCs   = polarError   > *controllerPolar;
    if (!std::isnormal(polarCs))
    {
        polarCs = 0.0;
    }

    xd[0] = polarCs;

    /* control en orientacion solo si:
     * (inclinacion > 5)
     */
    if(targetPose[0]>5)
    {
        CD_INFO_NO_HEADER("> Controlando en orientación\n");
        azimuthCs = azimuthError > *controllerAzimuth;

        if (!std::isnormal(azimuthCs))
        {
            azimuthCs = 0.0;
        }

        xd[1] = azimuthCs;
    }

    CD_DEBUG_NO_HEADER("- Polar:   target %f, sensor %f, error %f, cs: %f\n", targetPose[0], x_imu[0], polarError, polarCs);
    CD_DEBUG_NO_HEADER("- Azimuth: target %f, sensor %f, error %f, cs: %f\n", targetPose[1], x_imu[1], azimuthError, azimuthCs);

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

void SoftNeckControl::handleMovjClosedLoopUndocked()
{
    std::vector<double> x_imu;
    double polarError,
           azimuthError,
           polarCs,
           azimuthCs
           = 0.0;

    double cs1; // motor izq
    double cs2; // motor der
    int motores[2] = {1,2};
    double cs[2];

    if (!serialStreamResponder->getLastData(x_imu))
    {
        CD_WARNING("Outdated serial stream data.\n");
        iVelocityControl->stop();
    }

    std::vector<double> xd = targetPose;
    polarError   = (xd[0] - x_imu[0])*M_1_PI/180;
    azimuthError = (xd[1] - x_imu[1])*M_1_PI/180;

    polarCs   = polarError   > *incon;
    azimuthCs = azimuthError > *orcon;

    if (!std::isnormal(polarCs)) polarCs = 0;
    if (!std::isnormal(azimuthCs) || x_imu[1] <5) azimuthCs = 0;

    cs[0]=(polarCs-azimuthCs)/winchRadius;
    cs[1]=(polarCs+azimuthCs)/winchRadius;

    if (!iVelocityControl->velocityMove(2,motores,cs));
    {
        CD_ERROR("velocityMove failed.\n");
    }
}
