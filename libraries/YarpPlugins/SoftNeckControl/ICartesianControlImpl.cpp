// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>
#include <math.h>
#include <KinematicRepresentation.hpp>
#include <yarp/os/LogStream.h>

using namespace humasoft;
using namespace roboticslab::KinRepresentation;

namespace
{
    const double EPSILON = 1e-6;
}

// ------------------- ICartesianControl Related ------------------------------------

bool SoftNeckControl::stat(std::vector<double> & x, int * state, double * timestamp)
{
    if (!sensorPort.isClosed())
    {
        std::vector<double> x_imu; // roll,pitch


        switch (sensorType) {
            case '0':
                if (!serialStreamResponder->getLastData(x_imu))
                {
                    yWarning() << "Outdated SparkfunIMU serial stream data.";
                }
            break;
            case '1':
                if (!immu3dmgx510StreamResponder->getLastData(x_imu))
                {
                    yWarning() << "Outdated 3DMGX510IMU stream data.";
                }
            break;
            case '2':
                if (!mocapStreamResponder->getLastData(x_imu))
                {
                    yWarning() << "Outdated Mocap stream data.";
                }
            break;
        }

        if(sensorType!=0){
            // transform [roll.pitch] to [inc, ori]
            double inc = sqrt(pow(x_imu[1], 2) + pow(x_imu[0], 2));
            double ori = fmod( (360 - (atan2(-x_imu[0], x_imu[1])) * 180/M_PI), 360);
            x_imu[0] = inc;
            x_imu[1] = ori;
        }

        if (!encodePose(x_imu, x, coordinate_system::NONE, orientation_system::POLAR_AZIMUTH, angular_units::DEGREES))
        {
            yError() <<"encodePose failed.";
            return false;
        }
        *state = getCurrentState();
        *timestamp = yarp::os::Time::now();
        return true;
    }

    yWarning() << "Serial stream not available.";
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::inv(const std::vector<double> & xd, std::vector<double> & q)
{
    std::vector<double> x_out;

    if (!decodePose(xd, x_out, coordinate_system::NONE, orientation_system::POLAR_AZIMUTH, angular_units::RADIANS))
    {
        yError() <<"decodePose failed.";
        return false;
    }

    computeIk(x_out[0], x_out[1], q);
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::movj(const std::vector<double> & xd)
{
    if(controlType=="ioCoupled")
    {
        if (!setControlModes(VOCAB_CM_POSITION))
        {
            yError() <<"Unable to set position mode.";
            return false;
        }
    }
    else if(controlType=="ioUncoupled")
    {
        if (!setControlModes(VOCAB_CM_VELOCITY))
        {
            yError() <<"Unable to set velocity mode.";
            return false;
        }
    }
    else if(controlType=="rpUncoupled")
    {
        if (!setControlModes(VOCAB_CM_POSITION))
        {
            yError() <<"Unable to set position mode.";
            return false;
        }
    }
    else yError() <<"Control mode not defined";


    if (sensorPort.isClosed()) //no IMU
    {
        if (!sendTargets(xd))
        {
            return false;
        }
    }
    else
    {
        if (!decodePose(xd, targetPose, coordinate_system::NONE, orientation_system::POLAR_AZIMUTH, angular_units::DEGREES)) // con IMU
        {
            yError() <<"decodePose failed.";
            return false;
        }

        // equations to solve the transformation: inclnation-orientation -> roll-pitch
        if(controlType == "rpUncoupled"){
            double pitch = targetPose[0] * cos(targetPose[1] * M_PI/180); // pitch
            double roll =  targetPose[0] * sin(targetPose[1] * M_PI/180); // roll
            targetPose[0] = -roll;
            targetPose[1] = -pitch;
        }

        else
        {
            if(targetPose[1]<0.0) targetPose[1]+= 360;
        }

    }
    cmcSuccess = true;
    setCurrentState(VOCAB_CC_MOVJ_CONTROLLING);
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::relj(const std::vector<double> & xd)
{
    yWarning() << "Not implemented.";
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::movl(const std::vector<double> & xd)
{
    yWarning() << "Not implemented.";
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::movv(const std::vector<double> & xdotd)
{
    yWarning() << "Not implemented." ;
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::gcmp()
{
    yWarning() << "Not implemented.";
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::forc(const std::vector<double> & td)
{
    yWarning() << "Not implemented.";
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::stopControl()
{
    if (!iPositionControl->stop())
    {
        yWarning() << "stop() failed.";
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
            yWarning("Timeout reached (%f seconds), stopping control.\n", timeout);
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
    yWarning() << "Not implemented.";
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::act(int command)
{
    yWarning() << "Not implemented.";
    return false;
}

// -----------------------------------------------------------------------------

void SoftNeckControl::twist(const std::vector<double> & xdot)
{
    yWarning() << "Not implemented.";
}

// -----------------------------------------------------------------------------

void SoftNeckControl::pose(const std::vector<double> & x, double interval)
{
    yWarning() << "Not implemented.";
}

// -----------------------------------------------------------------------------

void SoftNeckControl::movi(const std::vector<double> & x)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING || streamingCommand != VOCAB_CC_MOVI)
    {
        yError() <<"Streaming command not preset.";
        return;
    }

    std::vector<double> qd;

    if (!inv(x, qd))
    {
        yError() <<"inv failed.";
        return;
    }

    if (!iPositionDirect->setPositions(qd.data()))
    {
        yError() <<"setPositions failed.";
    }
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::setParameter(int vocab, double value)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING)
    {
        yError() <<"Unable to set config parameter while controlling.";
        return false;
    }

    switch (vocab)
    {
    case VOCAB_CC_CONFIG_CMC_PERIOD:
        if (!yarp::os::PeriodicThread::setPeriod(value * 0.001))
        {
            yError() <<"Cannot set new CMC period.";
            return false;
        }
        cmcPeriod = value * 0.001;
        break;
    case VOCAB_CC_CONFIG_WAIT_PERIOD:
        if (value <= 0.0)
        {
            yError() <<"Wait period cannot be negative nor zero.";
            return false;
        }
        waitPeriod = value * 0.001;
        break;
    case VOCAB_CC_CONFIG_FRAME:
        if (value != roboticslab::ICartesianSolver::BASE_FRAME)
        {
            yError() <<"Unrecognized or unsupported reference frame vocab.";
            return false;
        }
        break;
    case VOCAB_CC_CONFIG_STREAMING_CMD:
        if (!presetStreamingCommand(value))
        {
            yError() <<"Unable to preset streaming command.";
            return false;
        }
        streamingCommand = value;
        break;
    default:
        yError("Unrecognized or unsupported config parameter key: %s.\n", yarp::os::Vocab32::decode(vocab).c_str());
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
    case VOCAB_CC_CONFIG_FRAME:
        *value = roboticslab::ICartesianSolver::BASE_FRAME;
        break;
    case VOCAB_CC_CONFIG_STREAMING_CMD:
        *value = streamingCommand;
        break;
    default:
        yError("Unrecognized or unsupported config parameter key: %s.\n", yarp::os::Vocab32::decode(vocab).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::setParameters(const std::map<int, double> & params)
{
    if (getCurrentState() != VOCAB_CC_NOT_CONTROLLING)
    {
        yError() <<"Unable to set config parameters while controlling.";
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
    params.insert(std::make_pair(VOCAB_CC_CONFIG_FRAME, roboticslab::ICartesianSolver::BASE_FRAME));
    params.insert(std::make_pair(VOCAB_CC_CONFIG_STREAMING_CMD, streamingCommand));
    return true;
}

// -----------------------------------------------------------------------------
