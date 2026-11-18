#ifndef ENGINE_PARAMS_H_
#define ENGINE_PARAMS_H_

#include "MechanismConstants.h"

#include <array>
#include <cstdint>

inline std::array<double, 6> toArray(const double (&a)[6])
{
    return {a[0], a[1], a[2], a[3], a[4], a[5]};
}

// Confidence of a vector: its track's (Object), or the lower of its track's and its author's (MinAuthor), or of its
// track's and its author's for that container kind, CAM or CPM objects (MinAuthorKind)
enum class ConfidenceMode : uint8_t { Object, MinAuthor, MinAuthorKind };

// Runtime configuration; defaults are the values of MechanismConstants.h
// enable<component>: replay --no-<component> turns one off for ablation (all on by default)
struct EngineParams {

    // association and fusion
    double gateAlong = ConstantsAssociation::DEFAULT_GATE_ALONG;      // [m]
    double gateAcross = ConstantsAssociation::DEFAULT_GATE_ACROSS;    // [m]
    double retireAfter = ConstantsAssociation::DEFAULT_RETIRE_AFTER;  // [s]
    // off: no filter, the track is its latest accepted report, projected at constant velocity
    bool enableKalman = true;
    // white part of the position error in the reports, sigma per axis [m], whatever its source (injected by
    // scripts/add_noise.py, or the jitter of the VeReMi sensor errors). The UKF's
    // position R is then sqrt(sigma^2 + POS_NOISE_FLOOR^2), the same for CAM and CPM objects.
    double measNoise = -1.0;
    // fill VectorRecord's track-state fields (diagnosis)
    bool recordState = false;

    // predictive check against the fused track
    double maxPosDev = ConstantsPredictive::DEFAULT_MAX_POS_DEV;  // d_max_0 [m]
    bool enablePredictive = true;

    // consistency check against the track: |reported speed - track speed| above speedToleranceMps
    double speedToleranceMps = ConstantsConsistency::DEFAULT_SPEED_TOLERANCE_MPS;
    bool enableConsistency = true;
    // frozen position: a moving object reported closer than frozenScale * v_avg * dt to the track's position at
    // its latest update, a share of the displacement the speed implies
    double frozenScale = ConstantsConsistency::DEFAULT_FROZEN_SCALE;
    bool enableFrozen = true;

    // track UKF
    double stdJerk = ConstantsUkf::DEFAULT_STD_JERK;           // [m/s^3]
    double stdYawAccel = ConstantsUkf::DEFAULT_STD_YAW_ACCEL;  // [rad/s^2]
    std::array<double, 6> p0 = toArray(ConstantsUkf::DEFAULT_P0);
    std::array<double, 6> p0NoVelocity = toArray(ConstantsUkf::DEFAULT_P0_NO_VELOCITY);

    // confidence
    ConfidenceMode confidence = ConfidenceMode::Object;
    double forgetting = ConstantsScore::DEFAULT_FORGETTING;
};

#endif  // ENGINE_PARAMS_H_
