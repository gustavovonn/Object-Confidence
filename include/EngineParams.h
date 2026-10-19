#ifndef ENGINE_PARAMS_H_
#define ENGINE_PARAMS_H_

#include "MechanismConstants.h"

// Runtime configuration; defaults are the values of MechanismConstants.h
struct EngineParams {

    // association and fusion
    double gateDistance = ConstantsAssociation::DEFAULT_GATE_DISTANCE;  // [m]
    double retireAfter = ConstantsAssociation::DEFAULT_RETIRE_AFTER;    // [s]

    // consistency check between consecutive reports of one source
    double speedToleranceMps = ConstantsConsistency::DEFAULT_SPEED_TOLERANCE_MPS;
    bool enableConsistency = true;

    // UKF process noise
    double jerkStd = ConstantsNoise::DEFAULT_JERK_STD;
    double yawAccelStd = ConstantsNoise::DEFAULT_YAW_ACCEL_STD;

    // UKF measurement noise (same for CAM and CPM until radar noise is modelled)
    double positionStd = ConstantsNoise::DEFAULT_POSITION_STD;
    double speedStd = ConstantsNoise::DEFAULT_SPEED_STD;
    double headingStd = ConstantsNoise::DEFAULT_HEADING_STD;
    double unmeasuredStd = ConstantsNoise::DEFAULT_UNMEASURED_STD;

    // confidence
    double forgetting = ConstantsScore::DEFAULT_FORGETTING;
};

#endif  // ENGINE_PARAMS_H_
