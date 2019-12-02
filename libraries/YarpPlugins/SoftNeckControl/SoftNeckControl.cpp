// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

using namespace humasoft;

// -----------------------------------------------------------------------------

int SoftNeckControl::getCurrentState() const
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return currentState;
}

// -----------------------------------------------------------------------------

void SoftNeckControl::setCurrentState(int value)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    currentState = value;
}

// -----------------------------------------------------------------------------

