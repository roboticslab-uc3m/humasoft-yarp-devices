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

using yarp::os::Network;
using yarp::os::PeriodicThread;

class ThreadVelocityControl : public PeriodicThread
{
public:
    ThreadVelocityControl(double targetVel) :
        PeriodicThread(0.02),
        targetVel(targetVel),
        cmcPeriod(0.02),

        // Motors controller
        cntrl(0.015,36,0,cmcPeriod)
    {
    }

    bool threadInit() override
    {
        printf("Starting ThreadVelocityControl\n");

        if (!yarp::os::Network::checkNetwork())
        {
            yError() <<"Please start a yarp name server first.";
            return 1;
        }


        options.put("device", "remote_controlboard");
        options.put("remote", "/softneck");
        options.put("local", "/robot");

        dd.configure(options);

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

        if(!mode->setControlMode(0, VOCAB_CM_VELOCITY))
        {
            yError() <<"Unable to set velocity mode the first motor.";
            return false;
        }

        yInfo() << "SUCCESS: Configured in velocity mode the first motor";



        return true;
    }

    //called by start after threadInit, s is true iff the thread started
    //successfully
    void afterStart(bool s) override
    {
        if (s) {
            printf("Thread started successfully\n");
        } else {
            printf("Thread did not start\n");
        }
    }

    void run() override
    {
        double motorVel;
        if (!enc->getEncoderSpeed(0, &motorVel))
            yError() <<"getRefVelocity failed of motor 0.";

        double velError = targetVel - motorVel;
        double cs = cntrl.OutputUpdate(velError);

        if (!std::isnormal(cs))
        {
            cs = 0.0;
        }


        if (!vel->velocityMove(0, cs))
            yError() <<"velocityMove failed of motor 0.";
    }

    void threadRelease() override
    {
        printf("Goodbye from thread\n");

        if (!vel->velocityMove(0, 0.0))
            yError() <<"velocityMove failed stopping the motor.";


        printf("Motor stopped\n");
    }

private:
    yarp::os::Network yarp;
    yarp::os::Property options;
    yarp::dev::PolyDriver dd;

    yarp::dev::IPositionControl *pos;
    yarp::dev::IVelocityControl *vel;
    yarp::dev::ITorqueControl *torq;
    yarp::dev::IEncodersTimed *enc;
    yarp::dev::IControlMode *mode;

    double cmcPeriod;
    PIDBlock cntrl;

    double targetVel;

};


int main(int argc, char *argv[])
{

    ThreadVelocityControl threadControl(0.01);
    printf("Thread Period: %f\n", threadControl.getPeriod() );
    threadControl.start();

    printf("Press a botom to stop the motor\n");
    threadControl.stop();

    return 0;

}
