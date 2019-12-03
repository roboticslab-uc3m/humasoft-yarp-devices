// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <ColorDebug.h>

using namespace humasoft;

// ------------------- PeriodicThread Related ------------------------------------

void SoftNeckControl::run()
{
    const int currentState = getCurrentState();

    switch (currentState)
    {
    case VOCAB_CC_MOVJ_CONTROLLING:
        handleMovj();
        break;
    default:
        break;
    }
}

// -----------------------------------------------------------------------------

void SoftNeckControl::handleMovj()
{
    bool done;

    if (!iPositionControl->checkMotionDone(&done))
    {
        CD_ERROR("Unable to query current robot state.\n");
        cmcSuccess = false;
        stopControl();
        return;
    }

    if (done)
    {
        setCurrentState(VOCAB_CC_NOT_CONTROLLING);

        if (!iPositionControl->setRefSpeeds(qRefSpeeds.data()))
        {
             CD_WARNING("setRefSpeeds (to restore) failed.\n");
        }
    }
}

// -----------------------------------------------------------------------------
