// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <cmath>

#include <yarp/os/LogStream.h>


using namespace humasoft;

// -----------------------------------------------------------------------------

void SoftNeckControl::setupControllers()
{
    controllerPolar   = new FPDBlock(controlPolarKp, controlPolarKd, controlPolarExp, cmcPeriod);
    controllerAzimuth = new FPDBlock(controlAzimuthKp, controlAzimuthKd, controlAzimuthExp, cmcPeriod);

    //controllerRoll = new PIDBlock(0.52,0.82,0.082,cmcPeriod);
    //controllerPitch = new PIDBlock(0.52,0.82,0.082,cmcPeriod);

    controllerRollFracc   = new FPDBlock(1.0659, 0.7593, -0.93, cmcPeriod);
    controllerPitchFracc = new FPDBlock(1.0659, 0.7593, -0.93, cmcPeriod);


    incon = new PIDBlock(0.5,0.1,0,cmcPeriod);
    orcon = new PIDBlock(0.05,0.0,0,cmcPeriod);
}

// -----------------------------------------------------------------------------

int SoftNeckControl::getCurrentState() const
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return currentState;
}

// -----------------------------------------------------------------------------

void SoftNeckControl::setCurrentState(int value)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    currentState = value;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::presetStreamingCommand(int command)
{
    setCurrentState(VOCAB_CC_NOT_CONTROLLING);

    switch (command)
    {
    case VOCAB_CC_MOVI:
        return setControlModes(VOCAB_CM_POSITION_DIRECT);
    case VOCAB_CC_TWIST:
        return setControlModes(VOCAB_CM_VELOCITY);
    case VOCAB_CC_WRENCH:
        return setControlModes(VOCAB_CM_TORQUE);
    default:
        yError() << "Unrecognized or unsupported streaming command vocab.";
    }

    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::setControlModes(int mode)
{
    std::vector<int> modes(NUM_ROBOT_JOINTS);

    if (!iControlMode->getControlModes(modes.data()))
    {
        yWarning() <<"getControlModes failed.";
        return false;
    }

    std::vector<int> jointIds;

    for (unsigned int i = 0; i < modes.size(); i++)
    {
        if (modes[i] != mode)
        {
            jointIds.push_back(i);
        }
    }

    if (!jointIds.empty())
    {
        modes.assign(jointIds.size(), mode);

        if (!iControlMode->setControlModes(jointIds.size(), jointIds.data(), modes.data()))
        {
            yWarning("setControlModes failed (%s).\n", yarp::os::Vocab32::decode(mode).c_str());
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

void SoftNeckControl::computeIsocronousSpeeds(const std::vector<double> & q, const std::vector<double> & qd,
        std::vector<double> & qdot)
{
    std::vector<double> distances(NUM_ROBOT_JOINTS);
    double maxTime = 0.0;

    //-- Find out the maximum time to move

    for (int joint = 0; joint < NUM_ROBOT_JOINTS; joint++)
    {
        if (qRefSpeeds[joint] <= 0.0)
        {
            yWarning("Zero or negative velocities sent at joint %d, not moving: %f.\n", joint, qRefSpeeds[joint]);
            return;
        }

        distances[joint] = std::abs(qd[joint] - q[joint]);
        double targetTime = distances[joint] / qRefSpeeds[joint];

        if (targetTime > maxTime)
        {
            maxTime = targetTime;
        }
    }

    //-- Compute, store old and set joint velocities given this time

    for (int joint = 0; joint < NUM_ROBOT_JOINTS; joint++)
    {
        qdot[joint] = distances[joint] / maxTime;
    }
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::sendTargets(const std::vector<double> & xd)
{
    std::vector<double> q(NUM_ROBOT_JOINTS);

    if (!iEncoders->getEncoders(q.data()))
    {
        yError() <<"getEncoders failed.";
        return false;
    }

    std::vector<double> qd;

    if (!inv(xd, qd))
    {
        yError() <<"inv failed.";
        return false;
    }

    std::vector<double> vmo(NUM_ROBOT_JOINTS);
    computeIsocronousSpeeds(q, qd, vmo);

    if (!iPositionControl->setRefSpeeds(vmo.data()))
    {
         yError() <<"setRefSpeeds failed.";
         return false;
    }

    if (!iPositionControl->positionMove(qd.data()))
    {
        yError() <<"positionMove failed.";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
