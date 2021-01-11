#include "imudevice.h"


#include "string.h"
#include <iostream>
#include <yarp/os/Property.h>
#include <cstdio>


namespace {
 YARP_LOG_COMPONENT(SERIALPORT, "yarp.device.serialport")
 }

bool IMUdevice::send(const yarp::os::Bottle& msg){

    CD_WARNING("Not implemented.\n");
    return false;
}

bool IMUdevice::send(char *msg, size_t size){

    CD_WARNING("Not implemented.\n");
    return false;
}

bool IMUdevice::receive(yarp::os::Bottle& msg){


    const int msgSize = 1001;
    char message[1001];

    //this function call blocks
    ssize_t bytes_read = _serial_dev.recv ((void *) message, msgSize - 1);

    if (bytes_read == -1)
    {
        yCError(SERIALPORT, "Error in SerialDeviceDriver::receive()");
        return false;
    }

    if (bytes_read == 0)  //nothing there
        return true;

    message[bytes_read] = 0;

    if (verbose)
    {
        yCDebug(SERIALPORT, "Data received from serial device: %s", message);
    }


    // Put message in the bottle
    msg.addString(message);

    return true;
}

int IMUdevice::receiveChar(char& chr){

    CD_WARNING("Not implemented.\n");
    return false;
}

int IMUdevice::receiveBytes(unsigned char* bytes, const int size){

    CD_WARNING("Not implemented.\n");
    return false;
}

int IMUdevice::receiveLine(char* line, const int MaxLineLength){

    CD_WARNING("Not implemented.\n");
    return false;
}

bool IMUdevice::setDTR(bool enable){

    CD_WARNING("Not implemented.\n");
    return false;
}

int IMUdevice::flush(){

    CD_WARNING("Not implemented.\n");
    return false;
}






