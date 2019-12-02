// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <yarp/os/Time.h>

using namespace humasoft;

// -----------------------------------------------------------------------------

SerialStreamResponder::SerialStreamResponder(double _timeout)
    : timeout(_timeout),
      localArrivalTime(0.0)
{}

// -----------------------------------------------------------------------------

void SerialStreamResponder::onRead(yarp::os::Bottle & b)
{
    std::lock_guard<std::mutex> lock(mutex);

    localArrivalTime = yarp::os::Time::now();
    data.resize(b.size());

    for (int i = 0; i < data.size(); i++)
    {
        data[i] = b.get(i).asFloat64();
    }
}

// -----------------------------------------------------------------------------

bool SerialStreamResponder::getLastData(std::vector<double> & data)
{
    std::lock_guard<std::mutex> lock(mutex);
    data = this->data;
    return yarp::os::Time::now() - localArrivalTime <= timeout;
}

// -----------------------------------------------------------------------------

