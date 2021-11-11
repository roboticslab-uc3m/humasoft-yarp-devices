// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

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
    switch (getCurrentState())
    {
    case VOCAB_CC_MOVJ_CONTROLLING:
        !sensorPort.isClosed() ? handleMovjClosedLoop() : handleMovjOpenLoop();
        // else yWarning() <<"Control mode not defined. Running in open loop...";
        break;
    default:
        break;
    }
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
    double polarError,
           azimuthError,
           polarCs,
           azimuthCs
           = 0.0;

    switch (sensorType) {
        case '1':
            if (!immu3dmgx510StreamResponder->getLastData(x_imu))
            {
                yWarning() <<"Outdated 3DMGX510IMU stream data.";
            } break;
        case '2':
            if (!mocapStreamResponder->getLastData(x_imu))
            {
                yWarning() <<"Outdated Mocap stream data.";
                iVelocityControl->stop();
            } break;
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
        yInfo() <<"> Controlando en orientación";
        azimuthCs = azimuthError > *controllerAzimuth;

        if (!std::isnormal(azimuthCs))
        {
            azimuthCs = 0.0;
        }

        xd[1] = azimuthCs;
    }

    yDebug("- Polar:   target %f, sensor %f, error %f, cs: %f\n", targetPose[0], x_imu[0], polarError, polarCs);
    yDebug("- Azimuth: target %f, sensor %f, error %f, cs: %f\n", targetPose[1], x_imu[1], azimuthError, azimuthCs);

    if (!encodePose(xd, xd, coordinate_system::NONE, orientation_system::POLAR_AZIMUTH, angular_units::DEGREES))
    {
        yError() <<"encodePose failed.";
        cmcSuccess = false;
        stopControl();
        return;
    }

    if (!sendTargets(xd))
    {
        yWarning() <<"Command error, not updating control this iteration.";
    }
}
