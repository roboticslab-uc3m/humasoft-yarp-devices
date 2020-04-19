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
#include <ColorDebug.h>

#include "fcontrol.h"

using namespace roboticslab::KinRepresentation;

int main(int argc, char *argv[])
{

    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR("Please start a yarp name server first.\n");
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
            CD_INFO_NO_HEADER("running demo without csv results..\n");
        }

        else if(argv[1]==std::string("csv"))
        {
            CD_INFO_NO_HEADER("running demo with csv results..\n");
            file = fopen("../data.csv","w+");
            fprintf(file, "time, mouse_inclination, filtered_inclination, sensor_inclination, mouse_orientation, filtered_orientation, sensor_orientation\n");
        }
        else
        {
            CD_ERROR("incorrect parameter\n");
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
            CD_ERROR("not values\n");
            break;
        }


        for(int i=0; i< mouseValues.size(); i++)
        {
            inValues[i] = mouseValues[i];
            if(mouseValues[5]!=0.0) inValues[5] = 0.0;
        }

        if (!decodePose(inValues, outValues, coordinate_system::NONE, orientation_system::POLAR_AZIMUTH, angular_units::DEGREES))
        {
            CD_ERROR("decodePose failed.\n");
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
