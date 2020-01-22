// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp-devices-humasoft-examples
 * \defgroup closedLoopSpaceMouse closedLoopSpaceMouse
 *
 * <b>Legal</b>
 *
 * Copyright: (C) 2019 Universidad Carlos III de Madrid;
 *
 * Authors: Raul de Santos Rico
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 */

#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include <ICartesianControl.h> // we need this to work with the CartesianControlClient device
#include <KinematicRepresentation.hpp> // encodePose, decodePose

#include <ColorDebug.h>

using namespace roboticslab::KinRepresentation;

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR("Please start a yarp name server first.\n");
        return 1;
    }

    // Config SpaceMouse
    yarp::os::Property options;
    options.put("device","SpaceNavigator");

    yarp::dev::PolyDriver spaceMouse(options); // spaceMouse.open(options)

    if (!spaceMouse.isValid())
    {
        std::printf("Device not available.\n");
        spaceMouse.close();
        yarp::os::Network::fini();
        return 1;
    }


    yarp::dev::IAnalogSensor *iAnalogSensor;
    if (!spaceMouse.view(iAnalogSensor) )
    {
        CD_ERROR("Problems acquiring interface\n");
        return 1;
    }

    CD_SUCCESS("Acquired interface [ok]\n");


    int channels = iAnalogSensor->getChannels();

    if(channels==0)
    {
        CD_ERROR("Failed number of channels\n");
        return 1;
    }

    yarp::sig::Vector mouseValues;
    std::vector<double> sumValues, outValues;
    sumValues.resize(channels);

    while(1)
    {
        if(iAnalogSensor->read(mouseValues) != yarp::dev::IAnalogSensor::AS_OK)
        {
            CD_ERROR("not values\n");
            break;
        }

        // suma
        for(int i=0; i< mouseValues.size(); i++)
            sumValues[i] += mouseValues[i];

        if (!decodePose(sumValues, outValues, coordinate_system::NONE, orientation_system::POLAR_AZIMUTH, angular_units::DEGREES))
        {
            CD_ERROR("decodePose failed.\n");
            return false;
        }

        printf("Inclinacion (%f) Orientacion (%f)\n", outValues[0], outValues[1]);

        yarp::os::Time::delay(0.005);
    }

    return 0;
}
