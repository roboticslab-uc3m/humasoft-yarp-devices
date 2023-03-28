// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp-devices-humasoft-programs
 * \defgroup tightenTendons tightenTendons
 *
 * <b>Legal</b>
 *
 * Copyright: (C) 2022 Universidad Carlos III de Madrid;
 *
 * Authors: Raul de Santos Rico
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * Usage: ./tightenThreads velM1 velM2 velM3 . Then, press a Key to stop the motors.
 *
 * Motor configuration of softNeck:
 *   M3 _ M2
 *    \   /
 *     M1
 *
 */

#include <iostream>
#include <string>
#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/ITorqueControl.h>

#include <yarp/os/LogStream.h>



int main(int argc, char *argv[])
{
    std::vector<double> vels(3);

    if(argc == 4){
        vels[0] = atof(argv[1]);
        vels[1] = atof(argv[2]);
        vels[2] = atof(argv[3]);
    }
    else
    {
        yError() <<"Usage: ./tightenThreads vel1 vel2 vel3\n";
        return 1;
    }

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() <<"Please start a yarp name server first.";
        return 1;
    }

    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/softneck");
    options.put("local", "/robot");

   yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        yError()<< "Device not available";
        return 1;
    }

    yarp::dev::IPositionControl *pos;
    yarp::dev::IVelocityControl *vel;
    yarp::dev::ITorqueControl *torq;
    yarp::dev::IEncodersTimed *enc;
    yarp::dev::IControlMode *mode;

    bool ok = true;
    ok &= dd.view(pos);
    ok &= dd.view(vel);
    ok &= dd.view(enc);
    ok &= dd.view(mode);
    
    if (!ok)
    {
        yError() << "ERROR: Problems acquiring soft-neck interface";
        return 1;
    }
    
    
    std::printf("SUCCESS: Acquired soft-neck interface\n");
    
    int axes;
    pos->getAxes(&axes);

    std::vector<int> velModes(axes, VOCAB_CM_VELOCITY);
    if(!mode->setControlModes(velModes.data()))
    {
        yError() <<"Unable to set velocity mode.";
        return false;
    }

    yInfo() << "SUCCESS: Configured in velocity mode "<<axes<<" motors";


    if(!vel->velocityMove(vels.data()))
    {
        yError() <<"Tighten threads failed.";
        dd.close();
    }

    yInfo() << "Tightening tendons... please press a key to skip";
    getchar();

    vel->stop();
    yInfo() << "stopped";


    return 0;
}
