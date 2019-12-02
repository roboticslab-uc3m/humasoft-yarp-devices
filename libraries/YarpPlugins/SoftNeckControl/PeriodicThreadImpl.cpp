// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

using namespace humasoft;

// ------------------- PeriodicThread Related ------------------------------------

void SoftNeckControl::run()
{
    const int currentState = getCurrentState();

    if (currentState == VOCAB_CC_NOT_CONTROLLING)
    {
        return;
    }
}

// -----------------------------------------------------------------------------

