#ifndef VECTOR_RECORD_H_
#define VECTOR_RECORD_H_

#include "MotionVector.h"

#include <cmath>
#include <cstdint>

enum class Outcome : uint8_t {
    Dropped,       // failed plausibility
    StartedTrack,  // matched no track
    Fused,         // passed the checks: hit
    Rejected       // failed a check, not fused: miss
};

// Engine output, one per received motion vector
struct VectorRecord {
    double t = 0.0;
    uint64_t receiver = 0;
    uint64_t author = 0;
    int64_t objectId = -1;
    Origin origin = Origin::Cam;
    int64_t trueStation = -1;  // oracle

    Outcome outcome = Outcome::Dropped;
    uint64_t trackId = 0;          // 0 when none
    double residual = -1.0;        // position residual to the projected track [m]

    bool consistencyFailed = false;
    bool predictiveFailed = false;  // residual above maxPosDev

    double confidence = -1.0;      // confidence after this vector (EngineParams::confidence): the lower of the two below
    double objectConfidence = NAN;  // its track's
    double authorConfidence = NAN;  // its author's (or author and kind); NaN with ConfidenceMode::Object

    // track state, filled only with EngineParams::recordState (diagnosis and calibration); NaN when none
    double predX = NAN, predY = NAN;  // track projected to t, before this vector [m]
    double vhat = NAN;                // speed of that projection [m/s]
    double stateT = NAN, stateX = NAN, stateY = NAN, stateV = NAN, stateYaw = NAN;  // after it; ENU yaw [rad]
    double nis = NAN;                 // UKF normalized innovation squared of its update (fused, with velocity)
};

#endif  // VECTOR_RECORD_H_
