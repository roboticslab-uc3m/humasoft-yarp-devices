// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <cmath>

#include <yarp/os/Time.h>

#include <ColorDebug.h>

using namespace humasoft;

// ------------------- ICartesianControl Related ------------------------------------

bool SoftNeckControl::stat(std::vector<double> & x, int * state, double * timestamp)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::inv(const std::vector<double> & xd, std::vector<double> & q)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::movj(const std::vector<double> & xd)
{
    std::vector<double> q(NUM_ROBOT_JOINTS);

    if (!iEncoders->getEncoders(q.data()))
    {
        CD_ERROR("getEncoders failed.\n");
        return false;
    }

    std::vector<double> qd(NUM_ROBOT_JOINTS);

    if (!inv(xd, qd))
    {
        return false;
    }

    std::vector<double> vmo(NUM_ROBOT_JOINTS);
    computeIsocronousSpeeds(q, qd, vmo);

    if (!iPositionControl->setRefSpeeds(vmo.data()))
    {
         CD_ERROR("setRefSpeeds failed.\n");
         return false;
    }

    if (!setControlModes(VOCAB_CM_POSITION))
    {
        CD_ERROR("Unable to set position mode.\n");
        return false;
    }

    if (!iPositionControl->positionMove(qd.data()))
    {
        CD_ERROR("positionMove failed.\n");
        return false;
    }

    cmcSuccess = true;
    setCurrentState(VOCAB_CC_MOVJ_CONTROLLING);
    CD_SUCCESS("Waiting\n");

    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::relj(const std::vector<double> & xd)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::movl(const std::vector<double> & xd)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::movv(const std::vector<double> & xdotd)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::gcmp()
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::forc(const std::vector<double> & td)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::stopControl()
{
    if (!iPositionControl->stop())
    {
        CD_WARNING("stop() failed.\n");
    }

    setCurrentState(VOCAB_CC_NOT_CONTROLLING);
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::wait(double timeout)
{
    int state = getCurrentState();

    if (state != VOCAB_CC_MOVJ_CONTROLLING)
    {
        return true;
    }

    double start = yarp::os::Time::now();

    while (state != VOCAB_CC_NOT_CONTROLLING)
    {
        if (timeout != 0.0 && yarp::os::Time::now() - start > timeout)
        {
            CD_WARNING("Timeout reached (%f seconds), stopping control.\n", timeout);
            stopControl();
            break;
        }

        yarp::os::Time::delay(DEFAULT_WAIT_PERIOD);
        state = getCurrentState();
    }

    return cmcSuccess;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::tool(const std::vector<double> & x)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::act(int command)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

void SoftNeckControl::twist(const std::vector<double> & xdot)
{
    CD_WARNING("Not implemented.\n");
}

// -----------------------------------------------------------------------------

void SoftNeckControl::pose(const std::vector<double> & x, double interval)
{
    CD_WARNING("Not implemented.\n");
}

// -----------------------------------------------------------------------------

void SoftNeckControl::movi(const std::vector<double> & x)
{
    CD_WARNING("Not implemented.\n");
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::setParameter(int vocab, double value)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::getParameter(int vocab, double * value)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::setParameters(const std::map<int, double> & params)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::getParameters(std::map<int, double> & params)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------
