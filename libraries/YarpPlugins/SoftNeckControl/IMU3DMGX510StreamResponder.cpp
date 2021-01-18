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
    // Configuración de los filtros: para una frecuencua de corte de 8 rad/s y tiempo de muestreo 0,02s.
    polarFilterSensor = new SystemBlock(0.1479, 0, -0.8521, 1);
    azimuthFilterSensor = new SystemBlock(0.1479, 0, -0.8521, 1);
}

IMU3DMGX510StreamResponder::~IMU3DMGX510StreamResponder()
    {
        delete polarFilterSensor;
        delete azimuthFilterSensor;
    }

// -----------------------------------------------------------------------------

void IMU3DMGX510StreamResponder::onRead(yarp::os::Bottle & b)
{
    std::lock_guard<std::mutex> lock(mutex);

    if (b.size() == 0)
    {
        return;
    }

    x[0] = b.get(0).asDouble();
    x[1] = b.get(1).asDouble();

    localArrivalTime = yarp::os::Time::now();

}


// -----------------------------------------------------------------------------

bool IMU3DMGX510StreamResponder::getLastData(std::vector<double> & v)
{
    std::lock_guard<std::mutex> lock(mutex);

    // si fuera necesario filtrar
    v[0] = polarFilterSensor->OutputUpdate(this->x[0]);
    v[1] = azimuthFilterSensor->OutputUpdate(this->x[1]);

    return yarp::os::Time::now() - localArrivalTime <= timeout;
}

// -----------------------------------------------------------------------------
