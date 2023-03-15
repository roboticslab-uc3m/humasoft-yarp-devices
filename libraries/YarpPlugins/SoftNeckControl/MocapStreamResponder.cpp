// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <cstdlib> // std::atof

#include <yarp/os/Time.h>

using namespace humasoft;

// -----------------------------------------------------------------------------

MocapStreamResponder::MocapStreamResponder(double _timeout)
    : timeout(_timeout),
      localArrivalTime(0.0),
      x(2, 0.0)
{
}

MocapStreamResponder::~MocapStreamResponder()
    {
    }

// -----------------------------------------------------------------------------

void MocapStreamResponder::onRead(yarp::os::Bottle & b)
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

bool MocapStreamResponder::getLastData(std::vector<double> & v)
{
    std::lock_guard<std::mutex> lock(mutex);
    v=x ;
    return yarp::os::Time::now() - localArrivalTime <= timeout;
}

// -----------------------------------------------------------------------------
