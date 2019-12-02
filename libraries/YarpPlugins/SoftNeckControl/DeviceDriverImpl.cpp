// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <ColorDebug.h>

using namespace humasoft;

// ------------------- DeviceDriver Related ------------------------------------

bool SoftNeckControl::open(yarp::os::Searchable & config)
{
    CD_DEBUG("%s.\n", config.toString().c_str());
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::close()
{
    return true;
}

// -----------------------------------------------------------------------------

