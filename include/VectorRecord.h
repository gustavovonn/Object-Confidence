#ifndef VECTOR_RECORD_H_
#define VECTOR_RECORD_H_

#include "MotionVector.h"

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
    double confidence = -1.0;      // track confidence after this vector
};

#endif  // VECTOR_RECORD_H_
