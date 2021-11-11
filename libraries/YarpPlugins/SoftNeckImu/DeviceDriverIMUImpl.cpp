#include "imudevice.hpp"
#include <yarp/os/LogStream.h>

// ------------------- DeviceDriver Related ------------------------------------

bool IMUdevice::open(yarp::os::Searchable & config)
{
     nameyarpoutport = config.check("sendport", yarp::os::Value(DEFAULT_OUTPORT), "local outport yarp").asString();
     comport = config.check("comport",yarp::os::Value(DEFAULT_COMPORT),"name of the serial port where imu is connected to").asString().c_str();
     output = config.check("output",yarp::os::Value(DEFAULT_OUTPUT),"output orientation: rp (softneck) or rpy (softarm)").asString().c_str();
     bx = config.check("bx", yarp::os::Value(DEFAULT_BX), "Gyro bias x").asFloat64();
     by = config.check("by", yarp::os::Value(DEFAULT_BY), "Gyro bias y").asFloat64();
     bz = config.check("bz", yarp::os::Value(DEFAULT_BZ), "Gyro bias z").asFloat64();
     Kp = config.check("Kp", yarp::os::Value(DEFAULT_KP), "Kp gain").asFloat64();
     Ti = config.check("Ti", yarp::os::Value(DEFAULT_TI), "Ti gain").asFloat64();
     KpQuick = config.check("KpQuick", yarp::os::Value(DEFAULT_KP_QUICK), "KpQuick gain").asFloat64();
     TiQuick = config.check("TiQuick", yarp::os::Value(DEFAULT_TI_QUICK), "TiQuick gain").asFloat64();
     period = config.check("period", yarp::os::Value(DEFAULT_PERIOD), "IMU Period").asDouble();
     frequency = config.check("freq",yarp::os::Value(DEFAULT_FREQUENCY), "Frequency").asInt();
     cmcPeriod = config.check("cmcPeriod", yarp::os::Value(DEFAULT_CMC_PERIOD), "Thread period (seconds)").asFloat64();

     //IMU period and run method period must be equals
     //In case in which period or frequency are introduced by the user ...
     if(config.check("period")){
         period = config.find("period").asDouble();
         cmcPeriod = period;
         frequency = 1 / period ;      
         cout << "Using period = " << period << "s, and frequency = " << frequency << "Hz." << endl;
     }else if(config.check("freq")){
         frequency = config.find("freq").asInt();
         period = 1 / frequency ;
         cmcPeriod = period;
         cout << "Using period = " << period << "s, and frequency = " << frequency << "Hz." << endl;
     }else if(config.check("cmcPeriod")){
         cmcPeriod = config.find("cmcPeriod").asDouble();
         period = cmcPeriod;
         frequency = 1 / period ;
         cout << "Using period = " << period << "s, and frequency = " << frequency << "Hz." << endl;
     }else {
         cout << "Using default period of " << DEFAULT_PERIOD << " s, and default frequency of " << DEFAULT_FREQUENCY << " Hz." << endl;
     }

     if (cmcPeriod != DEFAULT_CMC_PERIOD)
     {
         yarp::os::PeriodicThread::setPeriod(cmcPeriod);
     }

     //IMU initilization
     setupIMU();

    return yarp::os::PeriodicThread::start();
}

// -----------------------------------------------------------------------------

bool IMUdevice::close()
{
    delete sensor;
    yarpPort.close();
    PeriodicThread::stop();
    return true;
}

// -----------------------------------------------------------------------------








