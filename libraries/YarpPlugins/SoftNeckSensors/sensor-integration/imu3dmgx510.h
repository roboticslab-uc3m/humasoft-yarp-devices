#ifndef IMU3DMGX510_HPP
#define IMU3DMGX510_HPP



#include <iostream>
#include <sstream>
#include <string.h>
#include <math.h>
#include "attitude_estimator.h"
#include <tuple>

#include <boost/algorithm/hex.hpp>
using namespace boost::algorithm;

#include "SerialComm.h"

using namespace std;
using namespace stateestimation;

class IMU3DMGX510
{
public:

    IMU3DMGX510(string portName = "/dev/ttyUSB0"); //Constructor


    long set_IDLEmode();  //This function sets our device into IDLE mode
    long set_streamon(); //This function enable stream
    long set_streamoff(); //This function unenable stream
    long set_reset(); //This function resets the device

    long set_devicetogetgyroacc(int); //This function configure our device to give us gyro(x,y,z) and acc(x,y,z)
    long set_devicetogetgyro(int); //This function configure our device to give us gyro(x,y,z)


    std::tuple <float, float, float> get_gyroPolling();
    double* get_euleranglesPolling();


    std::tuple <double*,double*,double*> get_gyroContinuousStream (int); //This funcion gives us gyro data
    std::tuple <double*,double*,double,double> get_euleranglesContinuousStream (int); //This funcion gives us pitch and roll, and both initial pitch offset and initial roll offset



private: //Attributes

    union ulf
    {
        unsigned long ul;
        float f;
    };

    SerialComm port; //DO NOT try to invoce the constructor here
    AttitudeEstimator estimador;

    //Setting of GyroBias
    double bx = -0.002786;
    double by = -0.001833;
    double bz = -0.001066;

    //Defaults gains used
    double Kp = 2.2;
    double Ti = 2.65;
    double KpQuick = 10;
    double TiQuick = 1.25;

    //Concrete data packets of this device
    std::string idle = "\x75\x65\x01\x02\x02\x02\xe1\xc7";
    string respuestacorrectaidle = ("\x75\x65\x01\x04\x04\xF1\x02\x00\xD6\x6C"s);
    std::string imudata1 = "\x75\x65\x0c\x07\x07\x08\x01\x01\x05\x03\xe8\xee\x04";
    std::string imudata100 = ("\x75\x65\x0c\x07\x07\x08\x01\x01\x05\x00\x0a\x0d\x20"s);
    std::string imudata1000 = ("\x75\x65\x0c\x07\x07\x08\x01\x01\x05\x00\x01\x04\x17"s);
    std::string reset = "\x75\x65\x01\x02\x02\x7e\x5d\x43";
    std::string respuestacorrectareset = ("\x75\x65\x01\x04\x04\xF1\x7e\x00\x52\x64"s);
    std::string baudratenew = ("\x75\x65\x0c\x07\x07\x40\x01\x00\x03\x84\x00\xbc\x64"s);
    std::string gyracc = ("\x75\x65\x0c\x0a\x0a\x08\x01\x02\x04\x03\xe8\x05\x03\xe8\xe4\x0b"s);
    std::string gyracc100 = ("\x75\x65\x0c\x0a\x0a\x08\x01\x02\x04\x00\x0a\x05\x00\x0a\x22\xa0"s);
    std::string respuestacorrectaajustes = ("\x75\x65\x0c\x04\x04\xF1\x08\x00\xE7\xBA"s);
    std::string streamon = "\x75\x65\x0c\x05\x05\x11\x01\x01\x01\x04\x1a";
    std::string streamoff = ("\x75\x65\x0c\x05\x05\x11\x01\x01\x00\x03\x19"s);
    std::string respuestacorrectastreamonoff = ("\x75\x65\x0c\x04\x04\xF1\x11\x00\xf0\xcc"s);
    std::string polling = ("\x75\x65\x0c\x04\x04\x01\x00\x00\xef\xda"s);
};



#endif // IMU3DMGX510_HPP
