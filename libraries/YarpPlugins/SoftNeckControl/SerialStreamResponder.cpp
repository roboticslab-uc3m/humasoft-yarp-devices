// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <cmath>
#include <cstdlib> // std::atof

#include <yarp/os/Time.h>

using namespace humasoft;

// -----------------------------------------------------------------------------

SerialStreamResponder::SerialStreamResponder(double _timeout)
    : timeout(_timeout),
      localArrivalTime(0.0),
      x(4, 0.0)
{}

// -----------------------------------------------------------------------------

void SerialStreamResponder::onRead(yarp::os::Bottle & b)
{
    std::lock_guard<std::mutex> lock(mutex);

    if (b.size() == 0)
    {
        return;
    }

    if (accumulateStuff(b.get(0).asString()))
    {
        localArrivalTime = yarp::os::Time::now();
    }
}

// -----------------------------------------------------------------------------

bool SerialStreamResponder::accumulateStuff(const std::string & s)
{
    bool parsed = false;
    std::string::size_type newline = s.find('\n');

    if (newline == std::string::npos)
    {
        accumulator += s;
        return false;
    }

    accumulator += s.substr(0, newline);

    std::string::size_type i = accumulator.find('i');
    std::string::size_type o = accumulator.find('o');

    if (i == 0 && o != std::string::npos && o > i)
    {
        double orient = std::atof(accumulator.substr(o + 1, accumulator.size()).c_str()) * M_PI / 180.0;
        x[0] = -std::sin(orient);
        x[1] = std::cos(orient);
        x[3] = std::atof(accumulator.substr(i + 1, o).c_str());
        parsed = true;
    }

    accumulator = s.substr(newline + 1);
    return parsed;
}

// -----------------------------------------------------------------------------

bool SerialStreamResponder::getLastData(std::vector<double> & x)
{
    std::lock_guard<std::mutex> lock(mutex);
    x = this->x;
    return yarp::os::Time::now() - localArrivalTime <= timeout;
}

// -----------------------------------------------------------------------------
