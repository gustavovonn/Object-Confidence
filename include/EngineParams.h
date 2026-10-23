#ifndef ENGINE_PARAMS_H_
#define ENGINE_PARAMS_H_

#include "MechanismConstants.h"

// Runtime configuration; defaults are the values of MechanismConstants.h
struct EngineParams {

    // association and fusion
    double gateDistance = ConstantsAssociation::DEFAULT_GATE_DISTANCE;  // [m]
    double retireAfter = ConstantsAssociation::DEFAULT_RETIRE_AFTER;    // [s]
    double sourceActiveWindow = ConstantsAssociation::DEFAULT_SOURCE_ACTIVE_WINDOW;  // [s]

    // consistency check between consecutive reports of one source
    double speedToleranceMps = ConstantsConsistency::DEFAULT_SPEED_TOLERANCE_MPS;
    bool enableConsistency = true;

    // confidence
    double forgetting = ConstantsScore::DEFAULT_FORGETTING;
};

#endif  // ENGINE_PARAMS_H_
