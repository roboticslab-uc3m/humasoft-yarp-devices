// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

using namespace humasoft;

// ------------------- ICartesianControl Related ------------------------------------

bool SoftNeckControl::stat(std::vector<double> & x, int * state, double * timestamp)
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::inv(const std::vector<double> & xd, std::vector<double> & q)
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::movj(const std::vector<double> & xd)
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::relj(const std::vector<double> & xd)
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::movl(const std::vector<double> & xd)
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::movv(const std::vector<double> & xdotd)
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::gcmp()
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::forc(const std::vector<double> & td)
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::stopControl()
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::wait(double timeout)
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::tool(const std::vector<double> & x)
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::act(int command)
{
    return true;
}

// -----------------------------------------------------------------------------

void SoftNeckControl::twist(const std::vector<double> & xdot)
{
}

// -----------------------------------------------------------------------------

void SoftNeckControl::pose(const std::vector<double> & x, double interval)
{
}

// -----------------------------------------------------------------------------

void SoftNeckControl::movi(const std::vector<double> & x)
{
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::setParameter(int vocab, double value)
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::getParameter(int vocab, double * value)
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::setParameters(const std::map<int, double> & params)
{
    return true;
}

// -----------------------------------------------------------------------------

bool SoftNeckControl::getParameters(std::map<int, double> & params)
{
    return true;
}

// -----------------------------------------------------------------------------

