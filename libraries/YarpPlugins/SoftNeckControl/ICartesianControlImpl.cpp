// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <ColorDebug.h>

using namespace humasoft;

// ------------------- ICartesianControl Related ------------------------------------

bool SoftNeckControl::stat(std::vector<double> & x, int * state, double * timestamp)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::inv(const std::vector<double> & xd, std::vector<double> & q)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::movj(const std::vector<double> & xd)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::relj(const std::vector<double> & xd)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::movl(const std::vector<double> & xd)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::movv(const std::vector<double> & xdotd)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::gcmp()
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::forc(const std::vector<double> & td)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::stopControl()
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::wait(double timeout)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::tool(const std::vector<double> & x)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::act(int command)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

void SoftNeckControl::twist(const std::vector<double> & xdot)
{
    CD_WARNING("Not implemented.\n");
}

// -----------------------------------------------------------------------------

void SoftNeckControl::pose(const std::vector<double> & x, double interval)
{
    CD_WARNING("Not implemented.\n");
}

// -----------------------------------------------------------------------------

void SoftNeckControl::movi(const std::vector<double> & x)
{
    CD_WARNING("Not implemented.\n");
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::setParameter(int vocab, double value)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::getParameter(int vocab, double * value)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::setParameters(const std::map<int, double> & params)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::getParameters(std::map<int, double> & params)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

