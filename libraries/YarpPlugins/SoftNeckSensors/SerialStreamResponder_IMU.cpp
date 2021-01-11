// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "imudevice.h"

#include <cstdlib> // std::atof

#include <yarp/os/Time.h>


// -----------------------------------------------------------------------------

SerialStreamResponder_IMU::SerialStreamResponder_IMU(double _timeout)
    : timeout(_timeout),
      localArrivalTime(0.0),
      sensor()

{


}

SerialStreamResponder_IMU::~SerialStreamResponder_IMU()
    {


    }

// -----------------------------------------------------------------------------

void SerialStreamResponder_IMU::onRead(yarp::os::Bottle & b)
{

    if (!getDataIMU())
    {
        return;
    }

    if (getDataIMU())
    {
        localArrivalTime = yarp::os::Time::now();
    }
}

// -----------------------------------------------------------------------------

bool SerialStreamResponder_IMU::getDataIMU()
{
    bool parsed = true;

    sensor = new IMU3DMGX510(); //Main key changing constructor of IMU3DMGX510 atribute string --> string portname = "..."
    sensor->set_IDLEmode();
    sensor->set_devicetogetgyroacc(100);


    estimator = sensor->get_euleranglesPolling(); //Get the value in a vector
    cout << "(" << estimator[0] << "," << estimator[1] << ")" << endl;
    sensor->set_streamoff();

    return parsed;

}


// -----------------------------------------------------------------------------

bool SerialStreamResponder_IMU::getLastData(std::vector<double> & estimator)
{
    //Aqui entro con lo que leo en accumulateStuff
    return yarp::os::Time::now() - localArrivalTime <= timeout;
}

// -----------------------------------------------------------------------------
