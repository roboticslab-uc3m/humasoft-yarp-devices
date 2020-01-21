// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp-devices-humasoft-examples
 * \defgroup closedLoopExample closedLoopExample
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

#include <ICartesianControl.h> // we need this to work with the CartesianControlClient device

#include <ColorDebug.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR("Please start a yarp name server first.\n");
        return 1;
    }

    FILE *file = 0;
    if(argc!=2)
    {
        CD_INFO_NO_HEADER("running demo without csv results..\n");
    }

    else if(argv[1]=="csv")
    {
        CD_INFO_NO_HEADER("running demo with csv results..\n");
        file = fopen("../data.csv","w+");
        fprintf(file, "inclination target, inclination sensor, orientation target, orientation sensor\n");
    }

    yarp::os::Property options;
    options.put("device", "CartesianControlClient"); // our device (a dynamically loaded library)
    options.put("cartesianRemote", "/SoftNeckControl"); // remote port through which we'll talk to the server
    options.put("cartesianLocal", "/ClosedLoopExample");
    options.put("transform", 1);  // Was yarp::os::Value::getNullValue()

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        CD_ERROR("Device not available.\n");
        return 1;
    }

    roboticslab::ICartesianControl *iCartesianControl;

    if (!dd.view(iCartesianControl))
    {
        CD_ERROR("Problems acquiring interface.\n");
        return 1;
    }

    CD_SUCCESS("Acquired interface.\n");


    std::vector<double> pose[3];

    pose[0] = {15.0, 90.0};
    pose[1] = {30.0, 90.0};
    pose[2] = {30.0, 135.0};

    double timeout = 5.0;

    for(int i=0; i<3; i++)
    {
        CD_INFO_NO_HEADER("moving to pose [%d]\n", i);
        double initTime = yarp::os::Time::now();
        iCartesianControl->movj(pose[i]);

        if(file!=0)
        {
            while(yarp::os::Time::now() - initTime < timeout)
            {
                std::vector<double> imu;
                iCartesianControl->stat(imu);
                CD_INFO("%4f %4f\n", imu[0], imu[1]);
                fprintf(file,"%.4f","%.4f",pose[i][0], imu[0]);
                fprintf(file,"%.4f","%.4f",pose[i][1], imu[1]);
                yarp::os::Time::delay(0.020);
            }
        }

        yarp::os::Time::delay(5);
    }

    dd.close();

    return 0;
}
