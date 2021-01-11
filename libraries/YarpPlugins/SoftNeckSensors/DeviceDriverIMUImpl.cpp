#include "imudevice.h"
#include <ColorDebug.h>


// ------------------- DeviceDriver Related ------------------------------------

bool IMUdevice::open(yarp::os::Searchable & config)
{
     nameyarpoutport = config.check("sendport", yarp::os::Value(DEFAULT_OUTPORT), "local outport yarp").asString();
     comport = config.check("comport",yarp::os::Value(DEFAULT_COMPORT),"name of the serial port where imu is connected to").asString().c_str();
     period = config.check("period", yarp::os::Value(DEFAULT_PERIOD), "Period").asDouble();
     frequency = config.check("freq",yarp::os::Value(DEFAULT_FREQUENCY), "Frequency").asInt();
     bx = config.check("bx", yarp::os::Value(DEFAULT_BX), "Gyro bias x").asFloat64();
     by = config.check("by", yarp::os::Value(DEFAULT_BY), "Gyro bias y").asFloat64();
     bz = config.check("bz", yarp::os::Value(DEFAULT_BZ), "Gyro bias z").asFloat64();
     Kp = config.check("Kp", yarp::os::Value(DEFAULT_KP), "Kp gain").asFloat64();
     Ti = config.check("Ti", yarp::os::Value(DEFAULT_TI), "Ti gain").asFloat64();
     KpQuick = config.check("KpQuick", yarp::os::Value(DEFAULT_KP_QUICK), "KpQuick gain").asFloat64();
     TiQuick = config.check("TiQuick", yarp::os::Value(DEFAULT_TI_QUICK), "TiQuick gain").asFloat64();

     //In case in which period or frequency are introduced by the user ...
     if( config.check("period")){
         period = config.find("period").asDouble();
         frequency = 1 / period ;
         cout << "Using period = %d" << period << "s, and frequency = " << frequency << "Hz." << endl;
     } else if ( config.check("freq")){
         frequency = config.find("freq").asInt();
         period = 1 / frequency ;
         cout << "Using period = " << period << "s, and frequency = " << frequency << "Hz." << endl;
     } else  {
         cout << "Using default period of " << DEFAULT_PERIOD << " s, and default frequency of " << DEFAULT_FREQUENCY << " Hz" << endl;
     }

     //In case in which user decides the name of the Yarp outport
     if (config.check("sendport", "name of the yarp outport where imu data is published")){
         nameyarpoutport = config.find("sendport").asString();
     }

    //In case in which user sets the comport where he connect his imu
    if (config.check("comport"))
    {
        comport = config.find("comport").asString();

        if (!yarpPort.open(nameyarpoutport))
        {
            CD_ERROR("Unable to open Yarp outport.\n");
            return false;
        }

        //¿Lo usamos o no?
        serialStreamResponderIMU = new SerialStreamResponder_IMU(period);
        yarpPort.useCallback(*serialStreamResponderIMU);
    }

    //This method from Yarp::os::PeriodicThread will be run in the background
    run();

    return true;
}

// -----------------------------------------------------------------------------

bool IMUdevice::close()
{
    delete sensor;
    delete serialStreamResponderIMU;
    yarpPort.close();
    return true;
}

// -----------------------------------------------------------------------------








