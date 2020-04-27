// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp-devices-humasoft-examples
 * \defgroup stepInputDemo stepInputDemo
 *
 * <b>Legal</b>
 *
 * Copyright: (C) 2020 Universidad Carlos III de Madrid;
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

    else if(argv[1]==std::string("csv"))
    {
        CD_INFO_NO_HEADER("running demo with csv results..\n");
        file = fopen("../data.csv","w+");
        fprintf(file, "time, target_inclination, sensor_inclination, target_orientation, sensor_orientation\n");
    }
    else
    {
        CD_ERROR("incorrect parameter\n");
        return 0;
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

    // Optional step inputs
    pose[0] = {10.0, 90.0};
    pose[1] = {20.0, 90.0};
    pose[2] = {20.0, 135.0};

    // time/step (sec)
    double timeout = 20.0;

    while(true){
        for(int i=0; i<3; i++)
        {
            CD_INFO_NO_HEADER("moving to pose [%d]: [%f] [%f]\n", i, pose[i][0], pose[i][1]);
            double initTime = yarp::os::Time::now();
            iCartesianControl->movj(pose[i]);
            while(yarp::os::Time::now() - initTime < timeout)
            {
                std::vector<double> imu;
                iCartesianControl->stat(imu);

                // fix negative orientation
                if(imu[1]<0.0) imu[1]+= 360;
                CD_INFO_NO_HEADER("> Inclination: target(%.4f) sensor(%.4f)\n", pose[i][0], imu[0]);
                CD_INFO_NO_HEADER("> Orientation: target(%.4f) sensor(%.4f)\n", pose[i][1], imu[1]);
                if(file!=0)
                {
                    fprintf(file,"%.2f, ", yarp::os::Time::now() - initTime);
                    fprintf(file,"%.4f, %.4f, ",pose[i][0], imu[0]);
                    fprintf(file,"%.4f, %.4f \n",pose[i][1], imu[1]);
                }
                // sensor reading period
                yarp::os::Time::delay(0.020);
            }


        }
    } // while

    dd.close();

    return 0;
}
