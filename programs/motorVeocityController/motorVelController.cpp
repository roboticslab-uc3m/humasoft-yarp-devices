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
 * Usage: ./motorVelController .
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
#include "fcontrol.h"

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/ITorqueControl.h>

#include <yarp/os/PeriodicThread.h>

#include <yarp/os/LogStream.h>

#define RADIO 0.0075

using yarp::os::Network;
using yarp::os::PeriodicThread;




int main(int argc, char *argv[])
{

    yarp::os::Network yarp;

    double targetVel = 360;

    double vlong =  targetVel * M_PI/180 * RADIO;

    //control loop sampling time
    double freq=50; //sensor use values: 50,100,500...
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts); // 0.020

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

    if(!mode->setControlMode(0, VOCAB_CM_VELOCITY))
    {
        yError() <<"Unable to set velocity mode.";
        return false;
    }

    if(!vel->setRefAcceleration(0, 10))
    {
        yError() <<"setRefAcceleration failed.";
        dd.close();
    }

    // controller for motor
    PIDBlock controller(0.015,36,0,dts);



    while(1){ // 1seg

        double currentVel;
        if (!enc->getEncoderSpeed(0, &currentVel))
            yError() <<"getRefVelocity failed of motor 0.";

        double velError = vlong - currentVel;

        // Control process
        double cS = controller.OutputUpdate(velError);

        if (!std::isnormal(cS))
        {
            cS = 0.0;
        }

        if(!vel->velocityMove(0, cS))
        {
            yError() <<"velocityMove error";
            dd.close();
        }

        printf("-> currentVel: (%f) velError: (%f)  controlSignal: (%f)\n", currentVel, velError, cS);
        Ts.WaitSamplingTime();
    }



}



