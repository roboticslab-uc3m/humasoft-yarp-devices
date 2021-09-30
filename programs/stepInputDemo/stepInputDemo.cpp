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
#include <yarp/os/LogStream.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() <<"Please start a yarp name server first.";
        return 1;
    }

    FILE *file = 0;
    if(argc!=2)
    {
        yInfo() <<"running demo without csv results..";
    }

    else if(argv[1]==std::string("csv"))
    {
        yInfo() <<"running demo with csv results..";
        file = fopen("../data.csv","w+");
        fprintf(file, "time, target_inclination, sensor_inclination, target_orientation, sensor_orientation\n");
    }
    else
    {
        yError() <<"incorrect parameter";
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
        yError() <<"Device not available.";
        return 1;
    }

    roboticslab::ICartesianControl *iCartesianControl;

    if (!dd.view(iCartesianControl))
    {
        yError() <<"Problems acquiring interface.";
        return 1;
    }

    printf("Acquired interface.");


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
            yInfo("moving to pose [%d]: [%f] [%f]\n", i, pose[i][0], pose[i][1]);
            double initTime = yarp::os::Time::now();
            iCartesianControl->movj(pose[i]);
            while(yarp::os::Time::now() - initTime < timeout)
            {
                std::vector<double> imu;
                iCartesianControl->stat(imu);

                // fix negative orientation
                if(imu[1]<0.0) imu[1]+= 360;
                yInfo("> Inclination: target(%.4f) sensor(%.4f)\n", pose[i][0], imu[0]);
                yInfo("> Orientation: target(%.4f) sensor(%.4f)\n", pose[i][1], imu[1]);
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
