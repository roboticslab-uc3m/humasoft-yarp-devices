// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp-devices-humasoft-examples
 * \defgroup buttonPositionSend buttonPositionSend
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
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <ICartesianControl.h> // we need this to work with the CartesianControlClient device
#include <KinematicRepresentation.hpp> // encodePose, decodePose


using namespace roboticslab::KinRepresentation;

int main()
{    
    // double timeout = 10.0; // configuring timeout (s)
    std::vector<double> target = {0.0, 0.0};

    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork())
    {
        yError() <<"Please start a yarp name server first.";
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
        yError() <<"Device not available.";
        return 1;
    }

    roboticslab::ICartesianControl *iCartesianControl;

    if (!device.view(iCartesianControl))
    {
        yError() <<"Problems acquiring interface.";
        return 1;
    }

    printf("Acquired interface.\n");


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
        yError() <<"Problems acquiring interface";
        return 1;
    }

    printf("Acquired interface [ok]");


    int channels = iAnalogSensor->getChannels();

    if(channels==0)
    {
        yError() <<"Failed number of channels";
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
            yError() <<"not values";
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
            yError() <<"decodePose failed.";
            return 1;
        }

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
        yInfo("> Inclination: target(%.4f) sensor(%.4f)\n", target[0], imu[0]);
        yInfo("> Orientation: target(%.4f) sensor(%.4f)\n", target[1], imu[1]);
        yWarning("> NaveSpace: Inclination (%f) Orientation (%f)\n", outValues[0], outValues[1]);

        if(mouseValues[7]!=0.0 && !sended) // botón 1 pulsado
        {
            printf("\nSending position to SoftNeckControl\n");
            target = outValues;
            iCartesianControl->movj(target);
            sended = true;
        }

        yarp::os::Time::delay(0.005);

    }
    return 0;
}
