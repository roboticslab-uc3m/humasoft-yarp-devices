#include "imudevice.h"

#include "string.h"
#include <iostream>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

#include <cstdio>

using yarp::os::Bottle;
using yarp::os::BufferedPort;
using yarp::os::Network;





// ------------------- DeviceDriver Related ------------------------------------

bool IMUdevice::open(yarp::os::Searchable & config)
{

    std::string prefix = config.check("prefix", yarp::os::Value(DEFAULT_PREFIX), "local port prefix").asString();

    int bx = config.check("bx", yarp::os::Value(DEFAULT_BX), "Gyro bias x").asFloat64();
    int by = config.check("by", yarp::os::Value(DEFAULT_BY), "Gyro bias y").asFloat64();
    int bz = config.check("bz", yarp::os::Value(DEFAULT_BZ), "Gyro bias z").asFloat64();

    int Kp = config.check("Kp", yarp::os::Value(DEFAULT_KP), "Kp gain").asFloat64();
    int Ti = config.check("Ti", yarp::os::Value(DEFAULT_TI), "Ti gain").asFloat64();
    int KpQuick = config.check("KpQuick", yarp::os::Value(DEFAULT_KP_QUICK), "KpQuick gain").asFloat64();
    int TiQuick = config.check("TiQuick", yarp::os::Value(DEFAULT_TI_QUICK), "TiQuick gain").asFloat64();



    return true;
}

//// -----------------------------------------------------------------------------

bool IMUdevice::close()
{

    return true;
}

// -----------------------------------------------------------------------------

