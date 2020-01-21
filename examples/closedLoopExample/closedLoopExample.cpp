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

    yarp::os::Property options;
    options.put("device", "CartesianControlClient"); // our device (a dynamically loaded library)
    options.put("cartesianRemote", "/SoftNeckControl"); // remote port through which we'll talk to the server
    options.put("cartesianLocal", "/ClosedLoopExample2");
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

    std::vector<double> imu;


    while(true)
    {
        iCartesianControl->stat(imu);
        CD_INFO("%d %d\n", imu[0], imu[1]);
        yarp::os::Time::delay(0.020);
    }

    /*
    if (iCartesianControl->movj(position))
    {
        CD_SUCCESS_NO_HEADER("[ok]\n");
        iCartesianControl->wait();
    }
    else
    {
        CD_ERROR_NO_HEADER("[error]\n");
        return 1;
    } 
    */

    dd.close();

    return 0;
}
