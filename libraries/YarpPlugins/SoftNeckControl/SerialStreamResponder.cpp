// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <cstdlib> // std::atof

#include <yarp/os/Time.h>

using namespace humasoft;

// -----------------------------------------------------------------------------

SerialStreamResponder::SerialStreamResponder(double _timeout)
    : timeout(_timeout),
      localArrivalTime(0.0),
      x(2, 0.0)
{
    // Para una frecuencua de corte de 8 rad/s y tiempo de muestreo 0,02s.
    polarFilterSensor = new SystemBlock(0.1479, 0, -0.8521, 1);
    azimuthFilterSensor = new SystemBlock(0.1479, 0, -0.8521, 1);
}

SerialStreamResponder::~SerialStreamResponder()
    {
        delete polarFilterSensor;
        delete azimuthFilterSensor;
    }

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
        x[0] = std::atof(accumulator.substr(i + 1, o).c_str());
        x[1] = std::atof(accumulator.substr(o + 1, accumulator.size()).c_str());
        parsed = true;
    }

    accumulator = s.substr(newline + 1);
    return parsed;
}

// -----------------------------------------------------------------------------

bool SerialStreamResponder::getLastData(std::vector<double> & x)
{
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<double> v = this->x;
    v[0] = polarFilterSensor->OutputUpdate(this->x[0]);
    v[1] = azimuthFilterSensor->OutputUpdate(this->x[1]);
    if (!std::isnormal(this->x[1])) azimuthFilterSensor->Reset();
    if(this->x[1] < 3 || this->x[1] > 357 ) v[1] = 0.0;
    if(v[0]<5)   v[1] = 0.0;
    x = v;
    return yarp::os::Time::now() - localArrivalTime <= timeout;
}

// -----------------------------------------------------------------------------
