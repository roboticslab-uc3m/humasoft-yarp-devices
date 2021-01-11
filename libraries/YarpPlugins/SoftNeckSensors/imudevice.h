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
#include <ColorDebug.h>
#include <ace/DEV_Connector.h>
#include <ace/TTY_IO.h>

#include <yarp/conf/version.h>
#if YARP_VERSION_MINOR >= 3
# include <yarp/dev/ISerialDevice.h>
#else
# include <yarp/dev/SerialInterfaces.h>
#endif

using yarp::os::Bottle;
using yarp::os::BufferedPort;
using yarp::os::Network;


// ---------------   Defines of our device   ---------------
#define DEFAULT_PREFIX "/SoftNeckIMU"
#define DEFAULT_COMPORT "/dev/ttyUSB0"
#define DEFAULT_OUTPORT "/softimu"

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

// -------------------------------------------------------


//Había intentado hacerlo igual que en el de SoftNeckControl, pero no sé si será necesario o no
class SerialStreamResponder_IMU : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:

    SerialStreamResponder_IMU(double timeout);
    ~SerialStreamResponder_IMU  ();
    void onRead(yarp::os::Bottle & b);
    bool getLastData(std::vector<double> & x);
    bool getDataIMU();
private:

    const double timeout;
    double localArrivalTime;
    IMU3DMGX510 *sensor;
    double *estimator;
    int frequency; //1,100,1000
};





class IMUdevice :    public yarp::dev::DeviceDriver,
                     public yarp::dev::ISerialDevice, //Seguramente descartable
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
                    sensor(),
                    serialStreamResponderIMU(0),
                    yarp::os::PeriodicThread(DEFAULT_PERIOD)


    {}

 // -------- DeviceDriver declarations. Implementation in DeviceDriverIMUImpl.cpp --------

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

 // -------- PeriodicThread declarations. Implementation in PeriodicThreadIMU.cpp --------

    virtual void run();

 // -------- Trying to read data. Implementation in Readings.cpp --------

    //No se usa, es un device ya hecho de com serial
    bool send(const yarp::os::Bottle& msg) override;
    bool send(char *msg, size_t size) override;
    bool receive(yarp::os::Bottle& msg) override;
    int receiveChar(char& chr) override;
    int receiveBytes(unsigned char* bytes, const int size) override;
    int receiveLine(char* line, const int MaxLineLength) override;
    bool setDTR(bool enable) override;
    int flush() override;

private:

    //Setting of GyroBias
    double bx;
    double by;
    double bz;

    //Defaults gains used
    double Kp;
    double Ti;
    double KpQuick;
    double TiQuick;

    //Comm parameters
    double period;
    double frequency;

    std::string comport;
    std::string nameyarpoutport;

    Network yarp;
    BufferedPort<Bottle> yarpPort; //Outport to publish imu data. This kind of port allow us to start a server in the background
    SerialStreamResponder_IMU * serialStreamResponderIMU;

    IMU3DMGX510 *sensor;
    double *eulerdata;


    //No usado, de la implementación en Readings.cpp
    ACE_TTY_IO _serial_dev;
    ACE_DEV_Connector _serialConnector;
    bool verbose;



};

#endif // IMUDEVICE_H
