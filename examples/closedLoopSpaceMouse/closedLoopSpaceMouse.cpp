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
    double timeout = 10.0; // configuring timeout (s)
    std::vector<double> target = {0.0, 0.0};

    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR("Please start a yarp name server first.\n");
        return 1;
    }

    // Config SoftNeckControl
    yarp::os::Property snoptions;
    snoptions.put("device", "CartesianControlClient"); // our device (a dynamically loaded library)
    snoptions.put("cartesianRemote", "/SoftNeckControl"); // remote port through which we'll talk to the server
    snoptions.put("cartesianLocal", "/ClosedLoopExample");
    snoptions.put("transform", 1);  // Was yarp::os::Value::getNullValue()

    yarp::dev::PolyDriver device(snoptions);

    if (!device.isValid())
    {
        CD_ERROR("Device not available.\n");
        return 1;
    }

    roboticslab::ICartesianControl *iCartesianControl;

    if (!device.view(iCartesianControl))
    {
        CD_ERROR("Problems acquiring interface.\n");
        return 1;
    }

    CD_SUCCESS("Acquired interface.\n");


    // Config SpaceMouse
    yarp::os::Property smoptions;
    smoptions.put("device","SpaceNavigator");

    yarp::dev::PolyDriver spaceMouse(smoptions); // spaceMouse.open(options)

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
    bool sended;
    yarp::sig::Vector mouseValues;
    std::vector<double> inValues, outValues;
    inValues.resize(channels);

    while(1)
    {

        if(iAnalogSensor->read(mouseValues) != yarp::dev::IAnalogSensor::AS_OK)
        {
            CD_ERROR("not values\n");
            break;
        }

        if(mouseValues[7] == 0.0) sended = false; // flag para corregir la cola de lectura

        for(int i=0; i< mouseValues.size(); i++)
        {
            inValues[i] = mouseValues[i];
                        if(mouseValues[5]!=0.0) inValues[5] = 0.0;
        }

        printf("%f %f %f %f %f %f\n", inValues[0], inValues[1], inValues[2], inValues[3], inValues[4], inValues[5]);

        if (!decodePose(inValues, outValues, coordinate_system::NONE, orientation_system::POLAR_AZIMUTH, angular_units::DEGREES))
        {
            CD_ERROR("decodePose failed.\n");
            return false;
        }

        //printf("%f %f\n", outValues[0], outValues[1])

        if(outValues[0]>15.0) outValues[0]-= 15; // empezando a leer a partir de 15º para mayor precisión en orientación
        else
        {
            outValues[0] = 0.0;
            outValues[1] = 0.0;
        }

        if(outValues[0]> 40)  outValues[0] = 40;

        if(outValues[1]< 0.0) outValues[1] += 360; // corregimos la orientación negativa a partir de 180º

        std::vector<double> imu;
        iCartesianControl->stat(imu);
        if(imu[1]< 0.0) imu[1] += 360; // corregimos la orientación negativa a partir de 180º

        printf("-----------------------------------------------------------\n");
        CD_INFO_NO_HEADER("> Inclination: target(%.4f) sensor(%.4f)\n", target[0], imu[0]);
        CD_INFO_NO_HEADER("> Orientation: target(%.4f) sensor(%.4f)\n", target[1], imu[1]);
        CD_WARNING_NO_HEADER("> NaveSpace: Inclination (%f) Orientation (%f)\n", outValues[0], outValues[1]);

        if(mouseValues[7]!=0.0 && !sended) // botón 1 pulsado
        {
            CD_SUCCESS_NO_HEADER("\nSending position I(%f) O(%f) to SoftNeckControl\n");
            target = outValues;
            iCartesianControl->movj(target);
            sended = true;
        }

        yarp::os::Time::delay(0.005);

    }


    return 0;
}
