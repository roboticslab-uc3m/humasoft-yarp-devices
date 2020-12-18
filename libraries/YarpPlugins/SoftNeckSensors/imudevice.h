#ifndef IMUDEVICE_H
#define IMUDEVICE_H

//Includes needed


#include <yarp/sig/all.h>

#include <yarp/dev/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/PortReaderBuffer.h>

#include <yarp/conf/version.h>
#if YARP_VERSION_MINOR >= 3
# include <yarp/dev/ISerialDevice.h>
#else
# include <yarp/dev/SerialInterfaces.h>
#endif





//Usings needed






//Defines of our device

#define DEFAULT_PREFIX "/IMUDevice"

//Setting of GyroBias
#define DEFAULT_BX  -0.002786
#define DEFAULT_BY  -0.001833
#define DEFAULT_BZ  -0.001066

//Defaults gains used
#define DEFAULT_KP  2.2
#define DEFAULT_TI  2.65
#define DEFAULT_KP_QUICK  10
#define DEFAULT_TI_QUICK  1.25



class IMUdevice : public yarp::dev::DeviceDriver
{
public:
    IMUdevice() :   bx (DEFAULT_BX),
                    by (DEFAULT_BY),
                    bz (DEFAULT_BZ),
                    Ti(DEFAULT_TI),
                    Kp(DEFAULT_KP),
                    TiQuick(DEFAULT_TI_QUICK),
                    KpQuick(DEFAULT_KP_QUICK)


    {}


//    // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();






private:

    yarp::os::BufferedPort<yarp::os::Bottle> serialPort;

    //Setting of GyroBias
    double bx;
    double by;
    double bz;

    //Defaults gains used
    double Kp;
    double Ti;
    double KpQuick;
    double TiQuick;

};




#endif // IMUDEVICE_H
