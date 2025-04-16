// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftArmControl.hpp"

#include <cmath>

#include <yarp/os/LogStream.h>


using namespace sofia;

// -----------------------------------------------------------------------------

void SoftArmControl::setupControllers()
{
    fraccControllerPitch  = new PIDBlock(0.1, 1, 0, cmcPeriod);
    fraccControllerYaw    = new PIDBlock(0.1, 1, 0, cmcPeriod);
}

// ---------------------------------------------------------------------------

int SoftArmControl::getCurrentState() const
{
    lock_guard<mutex> lock(stateMutex);
    return currentState;
}

// -----------------------------------------------------------------------------

void SoftArmControl::setCurrentState(int value)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    currentState = value;
}

// -----------------------------------------------------------------------------

bool SoftArmControl::presetStreamingCommand(int command)
{
    setCurrentState(VOCAB_CC_NOT_CONTROLLING);

    switch (command)
    {
    case VOCAB_CC_TWIST:
        return setControlModes(VOCAB_CM_VELOCITY);
    case VOCAB_CC_POSE:
        return setControlModes(VOCAB_CM_POSITION_DIRECT);
    default:
        yError() << "Unrecognized or unsupported streaming command vocab.";
    }

    return false;
}

// -----------------------------------------------------------------------------

bool SoftArmControl::setControlModes(int mode)
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

void SoftArmControl::computeIsocronousSpeeds(const std::vector<double> & q, const std::vector<double> & qd,
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

bool SoftArmControl::sendTargets(const std::vector<double> & xd)
{
    std::vector<double> q(NUM_ROBOT_JOINTS);

    if (!iEncoders->getEncoders(q.data()))
    {
        yError() <<"getEncoders failed.";
        return false;
    }

    std::vector<double> qd(NUM_ROBOT_JOINTS);

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
