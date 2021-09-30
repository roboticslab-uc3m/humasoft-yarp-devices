// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp-devices-humasoft-examples
 * \defgroup continuousPositionSend continuousPositionSend
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
#include <yarp/os/LogStream.h>

#include "fcontrol.h"

using namespace roboticslab::KinRepresentation;

int main(int argc, char *argv[])
{

    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork())
    {
        yError() <<"Please start a yarp name server first.";
        return 1;
    }

    /* Sample time: 0.05 seconds
     * Discrete-time transfer function.
     */
     SystemBlock *mouseIncFilter = new SystemBlock(0.004988, 0, - 0.995, 1);
     SystemBlock *mouseOriFilter = new SystemBlock(0.004988, 0, - 0.995, 1);

    // CSV configuration
    FILE *file = 0;
        if(argc!=2)
        {
            yInfo() <<"running demo without csv results..";
        }

        else if(argv[1]==std::string("csv"))
        {
            yInfo() <<"running demo with csv results..";
            file = fopen("../data.csv","w+");
            fprintf(file, "time, mouse_inclination, filtered_inclination, sensor_inclination, mouse_orientation, filtered_orientation, sensor_orientation\n");
        }
        else
        {
            yError() <<"incorrect parameter";
            return 0;
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


    // Config SpaceMouse (joystick)
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

    printf("Acquired interface [ok]\n");


    int channels = iAnalogSensor->getChannels();

    if(channels==0)
    {
        yError() <<"Failed number of channels";
        return 1;
    }

    yarp::sig::Vector mouseValues;
    std::vector<double> inValues, outValues, filterValues;
    inValues.resize(channels);
    filterValues.resize(2);
    double initTime = yarp::os::Time::now();

    // Valores a los que inicializamos el filtro
    mouseIncFilter->Reset(6.0);
    mouseOriFilter->Reset(10.0);

    while(1)
    {

        if(iAnalogSensor->read(mouseValues) != yarp::dev::IAnalogSensor::AS_OK)
        {
            yError() <<"not values";
            break;
        }


        for(int i=0; i< mouseValues.size(); i++)
        {
            inValues[i] = mouseValues[i];
            if(mouseValues[5]!=0.0) inValues[5] = 0.0;
        }

        if (!decodePose(inValues, outValues, coordinate_system::NONE, orientation_system::POLAR_AZIMUTH, angular_units::DEGREES))
        {
            yError() <<"decodePose failed.";
            return false;
        }

        if(outValues[0]>15.0) outValues[0]-= 15; // empezando a leer a partir de 15º para mayor precisión en orientación
        else
        {
            outValues[0] = 6.0; // fuerzo inclinación a 5º para que no se vuelva loco
            outValues[1] = 10.0;
        }

        if(outValues[0]> 40)  outValues[0] = 40;

        if(outValues[1]< 0.0) outValues[1] += 360; // corregimos la orientación negativa a partir de 180º

        std::vector<double> imu;
        iCartesianControl->stat(imu);
        if(imu[1]< 0.0) imu[1] += 360; // corregimos la orientación negativa a partir de 180º


        filterValues[0] = mouseIncFilter->OutputUpdate(outValues[0]);
        filterValues[1] = mouseOriFilter->OutputUpdate(outValues[1]);

        if(file!=0)
        {
           fprintf(file,"%.2f, ", yarp::os::Time::now() - initTime);
           fprintf(file,"%.4f, %.4f, %.4f, ",outValues[0], filterValues[0] ,imu[0]);
           fprintf(file,"%.4f, %.4f, %.4f\n",outValues[1], filterValues[1] ,imu[1]);
        }

        printf("-----------------------------------------------------------\n");
        printf("> Inclination: mouse(%.4f) filtered(%.4f) sensor(%.4f)\n", outValues[0], filterValues[0], imu[0]);
        printf("> Orientation: mouse(%.4f) filtered(%.4f) sensor(%.4f)\n", outValues[1], filterValues[1], imu[1]);

        iCartesianControl->movj(filterValues);

        yarp::os::Time::delay(0.005);
    }
    return 0;
}
