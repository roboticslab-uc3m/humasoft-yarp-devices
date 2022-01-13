// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup humasoft-yarp-devices-examples
 * \defgroup exampleCartesianControlClient exampleCartesianControlClient
 *
 * <b>Legal</b>
 *
 * Copyright: (C) 2022 Universidad Carlos III de Madrid;
 *
 * Authors: Raul de Santos Rico
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <yarp/dev/PolyDriver.h>
#include <KinematicRepresentation.hpp>

#include <ICartesianControl.h>

using namespace roboticslab::KinRepresentation;

// Simplify encodePose function
std::vector<double> encPose(std::vector<double> pose)
{
    std::vector<double> x(6, 0.0);

    if (!encodePose(pose, x, coordinate_system::NONE, orientation_system::POLAR_AZIMUTH, angular_units::DEGREES))
    {
        yError() <<"encodePose failed.";
        return x;
    }
    return x;
}

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }


    yarp::os::Property options {
        {"device", yarp::os::Value("CartesianControlClient")},
        {"cartesianRemote", yarp::os::Value("/SoftArmControl")},
        {"cartesianLocal", yarp::os::Value("/SoftArmCartesianControlClient")},
        {"transform", yarp::os::Value(1)}
    };

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        yError() << "Device not available";
        return 1;
    }

    roboticslab::ICartesianControl *iCartesianControl;

    if (!dd.view(iCartesianControl))
    {
        yError() << "Problems acquiring interface";
        return 1;
    }


    yInfo() << "Step 1: poss (40 0)";

    if (!iCartesianControl->movj(encPose({40,0})))
    {
        yError() << "failure";
        return 1;
    }

    iCartesianControl->wait(3);

    yInfo() << "Step 2: make a circumference";
    
    // Configuration Position Direct
    double period = 0.005; // 5ms
    iCartesianControl->setParameter(VOCAB_CC_CONFIG_CMC_PERIOD, period);
    iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_MOVI);
    

    for(double ori=0.0; ori<=360; ori+=0.1)
    {
        yInfo() <<"orientation degrees: "<< ori;
        iCartesianControl->movi(encPose({40,ori}));
        yarp::os::Time::delay(period);
    }

    yInfo() << "Step 3: homing";

    if (!iCartesianControl->movj(encPose({0,0})))
    {
        yError() << "failure";
        return 1;
    }

    iCartesianControl->wait(3);
    dd.close();
    return 0;
}
