#ifndef ENGINE_PARAMS_H_
#define ENGINE_PARAMS_H_

#include "MechanismConstants.h"

// Runtime configuration; defaults are the values of MechanismConstants.h
struct EngineParams {

    // association and fusion
    double gateAlong = ConstantsAssociation::DEFAULT_GATE_ALONG;      // [m]
    double gateAcross = ConstantsAssociation::DEFAULT_GATE_ACROSS;    // [m]
    double retireAfter = ConstantsAssociation::DEFAULT_RETIRE_AFTER;  // [s]

    // predictive check against the fused track
    double maxPosDev = ConstantsPredictive::DEFAULT_MAX_POS_DEV;  // d_max_0 [m]

    // consistency check between consecutive reports of one source
    double speedToleranceMps = ConstantsConsistency::DEFAULT_SPEED_TOLERANCE_MPS;
    bool enableConsistency = true;

    // confidence
    double forgetting = ConstantsScore::DEFAULT_FORGETTING;
};

#endif  // ENGINE_PARAMS_H_
