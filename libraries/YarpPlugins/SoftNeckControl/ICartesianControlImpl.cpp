// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <cmath>

#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>

#include <KinematicRepresentation.hpp>
#include <ColorDebug.h>

using namespace humasoft;
using namespace roboticslab::KinRepresentation;

// ------------------- ICartesianControl Related ------------------------------------

bool SoftNeckControl::stat(std::vector<double> & x, int * state, double * timestamp)
{
    if (!serialPort.isClosed())
    {
        std::vector<double> rpy;

        if (!serialStreamResponder->getLastData(rpy))
        {
            CD_ERROR("Serial stream timed out.\n");
            return false;
        }

        if (!encodePose(rpy, x, coordinate_system::NONE, orientation_system::RPY, angular_units::DEGREES))
        {
            CD_ERROR("encodePose failed.\n");
            return false;
        }

        *state = getCurrentState();
        *timestamp = yarp::os::Time::now();
        return true;
    }

    CD_WARNING("Serial stream not available.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::inv(const std::vector<double> & xd, std::vector<double> & q)
{
    std::vector<double> x_out;

    if (!decodePose(xd, x_out, coordinate_system::NONE, orientation_system::RPY, angular_units::DEGREES))
    {
        CD_ERROR("decodePose failed.\n");
        return false;
    }

    computeIk(xd[1], xd[2], q);
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::movj(const std::vector<double> & xd)
{
    if (!setControlModes(VOCAB_CM_POSITION))
    {
        CD_ERROR("Unable to set position mode.\n");
        return false;
    }

    if (serialPort.isClosed())
    {
        if (!sendTargets(xd))
        {
            return false;
        }
    }
    else
    {
        resetController();

        if (!decodePose(xd, targetRPY, coordinate_system::NONE, orientation_system::RPY, angular_units::DEGREES))
        {
            CD_ERROR("decodePose failed.\n");
            return false;
        }

        targetStart = yarp::os::Time::now();
    }

    cmcSuccess = true;
    setCurrentState(VOCAB_CC_MOVJ_CONTROLLING);
    return true;
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

        yarp::os::Time::delay(waitPeriod);
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
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING || streamingCommand != VOCAB_CC_MOVI)
    {
        CD_ERROR("Streaming command not preset.\n");
        return;
    }

    std::vector<double> qd;

    if (!inv(x, qd))
    {
        CD_ERROR("inv failed.\n");
        return;
    }

    if (!iPositionDirect->setPositions(qd.data()))
    {
        CD_ERROR("setPositions failed.\n");
    }
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::setParameter(int vocab, double value)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING)
    {
        CD_ERROR("Unable to set config parameter while controlling.\n");
        return false;
    }

    switch (vocab)
    {
    case VOCAB_CC_CONFIG_CMC_PERIOD:
        if (!yarp::os::PeriodicThread::setPeriod(value * 0.001))
        {
            CD_ERROR("Cannot set new CMC period.\n");
            return false;
        }
        cmcPeriod = value * 0.001;
        break;
    case VOCAB_CC_CONFIG_WAIT_PERIOD:
        if (value <= 0.0)
        {
            CD_ERROR("Wait period cannot be negative nor zero.\n");
            return false;
        }
        waitPeriod = value * 0.001;
        break;
    case VOCAB_CC_CONFIG_STREAMING_CMD:
        if (!presetStreamingCommand(value))
        {
            CD_ERROR("Unable to preset streaming command.\n");
            return false;
        }
        streamingCommand = value;
        break;
    default:
        CD_ERROR("Unrecognized or unsupported config parameter key: %s.\n", yarp::os::Vocab::decode(vocab).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::getParameter(int vocab, double * value)
{
    switch (vocab)
    {
    case VOCAB_CC_CONFIG_CMC_PERIOD:
        *value = cmcPeriod * 1000.0;
        break;
    case VOCAB_CC_CONFIG_WAIT_PERIOD:
        *value = waitPeriod * 1000.0;
        break;
    case VOCAB_CC_CONFIG_STREAMING_CMD:
        *value = streamingCommand;
        break;
    default:
        CD_ERROR("Unrecognized or unsupported config parameter key: %s.\n", yarp::os::Vocab::decode(vocab).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::setParameters(const std::map<int, double> & params)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING)
    {
        CD_ERROR("Unable to set config parameters while controlling.\n");
        return false;
    }

    bool ok = true;

    for (std::map<int, double>::const_iterator it = params.begin(); it != params.end(); ++it)
    {
        ok &= setParameter(it->first, it->second);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::getParameters(std::map<int, double> & params)
{
    params.insert(std::make_pair(VOCAB_CC_CONFIG_CMC_PERIOD, cmcPeriod * 1000.0));
    params.insert(std::make_pair(VOCAB_CC_CONFIG_WAIT_PERIOD, waitPeriod * 1000.0));
    params.insert(std::make_pair(VOCAB_CC_CONFIG_STREAMING_CMD, streamingCommand));
    return true;
}

// -----------------------------------------------------------------------------
