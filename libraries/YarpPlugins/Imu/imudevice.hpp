#ifndef IMUDEVICE_H
#define IMUDEVICE_H

//Includes needed

#include <stdio.h>
#include <cstdio>

#include "imu3dmgx510.h"
#include "SerialComm.h"

#include <yarp/sig/all.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/PortReaderBuffer.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <ace/DEV_Connector.h>
#include <ace/TTY_IO.h>
#include <math.h>

#include <yarp/conf/version.h>
#if YARP_VERSION_MINOR >= 3
# include <yarp/dev/ISerialDevice.h>
#else
# include <yarp/dev/SerialInterfaces.h>
#endif

using yarp::os::Bottle;
using yarp::os::Network;


// ---------------   Defines of our device   ---------------
#define DEFAULT_PREFIX "/SoftNeckIMU"
#define DEFAULT_COMPORT "/dev/ttyUSB0"
#define DEFAULT_OUTPORT "/softimu"
#define DEFAULT_OUTPUT "rp"

 //Setting of GyroBias
#define DEFAULT_BX  -0.002786
#define DEFAULT_BY  -0.001833
#define DEFAULT_BZ  -0.001066

 //Defaults gains used
#define DEFAULT_KP  2.2
#define DEFAULT_TI  2.65
#define DEFAULT_KP_QUICK  10
#define DEFAULT_TI_QUICK  1.25

//Default period&freq time
#define DEFAULT_PERIOD 0.01 // seconds
#define DEFAULT_FREQUENCY 100 // Hz
#define DEFAULT_CMC_PERIOD 0.01 // seconds


// -------------------------------------------------------

class IMUdevice :    public yarp::dev::DeviceDriver,
                     public yarp::os::PeriodicThread
 {
public:
    IMUdevice() :   bx (DEFAULT_BX),
                    by (DEFAULT_BY),
                    bz (DEFAULT_BZ),
                    Ti(DEFAULT_TI),
                    Kp(DEFAULT_KP),
                    TiQuick(DEFAULT_TI_QUICK),
                    KpQuick(DEFAULT_KP_QUICK),
                    cmcPeriod(DEFAULT_CMC_PERIOD),
                    period(DEFAULT_PERIOD),
                    frequency(DEFAULT_FREQUENCY),
                    sensor(),
                    yarp::os::PeriodicThread(DEFAULT_CMC_PERIOD)
    {}

 // -------- DeviceDriver declarations. Implementation in DeviceDriverIMUImpl.cpp --------

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

 // -------- PeriodicThread declarations. Implementation in PeriodicThreadIMU.cpp --------

    virtual void run();

private:

    void setupIMU();

    //Setting of GyroBias
    double bx;
    double by;
    double bz;

    //Defaults gains used
    double Kp;
    double Ti;
    double KpQuick;
    double TiQuick;

    //IMU Comm parameters
    IMU3DMGX510 *sensor;
    double period;
    double frequency;
    std::string comport;
    std::string output;
    double pitch,roll,yaw;
    double *gyrodatos;

    //PeriodicThread parameters
    double cmcPeriod;
    double waitPeriod;

    //Outport to publish imu data. This kind of port allow us to start a server in the background
    yarp::os::Network Yarp;
    yarp::os::Port yarpPort;
    std::string nameyarpoutport;
    Bottle data;

};

#endif // IMUDEVICE_H
