#ifndef ENGINE_PARAMS_H_
#define ENGINE_PARAMS_H_

#include "MechanismConstants.h"

#include <array>

inline std::array<double, 6> toArray(const double (&a)[6])
{
    return {a[0], a[1], a[2], a[3], a[4], a[5]};
}

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
    // frozen-position bound scaled to K * v_avg * dt; <= 0: the fixed FROZEN_DISPLACEMENT_MIN
    double frozenScale = 0.0;

    // track UKF
    double stdJerk = ConstantsUkf::DEFAULT_STD_JERK;           // [m/s^3]
    double stdYawAccel = ConstantsUkf::DEFAULT_STD_YAW_ACCEL;  // [rad/s^2]
    std::array<double, 6> p0 = toArray(ConstantsUkf::DEFAULT_P0);
    std::array<double, 6> p0NoVelocity = toArray(ConstantsUkf::DEFAULT_P0_NO_VELOCITY);

    // confidence
    double forgetting = ConstantsScore::DEFAULT_FORGETTING;
};

#endif  // ENGINE_PARAMS_H_
