// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <cstdlib> // std::atof

#include <yarp/os/Time.h>

using namespace humasoft;

// -----------------------------------------------------------------------------

IMU3DMGX510StreamResponder::IMU3DMGX510StreamResponder(double _timeout)
    : timeout(_timeout),
      localArrivalTime(0.0),
      x(2, 0.0)
{
}

IMU3DMGX510StreamResponder::~IMU3DMGX510StreamResponder()
    {
    }

// -----------------------------------------------------------------------------

void IMU3DMGX510StreamResponder::onRead(yarp::os::Bottle & b)
{
    std::lock_guard<std::mutex> lock(mutex);

    if (b.size() == 0)
    {
        return;
    }

    x[0] = b.get(0).asFloat64();
    x[1] = b.get(1).asFloat64();

    localArrivalTime = yarp::os::Time::now();

}


// -----------------------------------------------------------------------------

bool IMU3DMGX510StreamResponder::getLastData(std::vector<double> & v)
{
    std::lock_guard<std::mutex> lock(mutex);
    v=x ;
    return yarp::os::Time::now() - localArrivalTime <= timeout;
}

// -----------------------------------------------------------------------------
