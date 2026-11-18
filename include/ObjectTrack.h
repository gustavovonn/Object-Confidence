#ifndef OBJECT_TRACK_H_
#define OBJECT_TRACK_H_

#include "MotionTypes.h"
#include "ukf.h"

#include <cstdint>
#include <deque>

using TrackId = uint64_t;

// Fused reference state of one object, held by the UKF.
// filter.x_ = [px, py, v, yaw, yaw rate]; yaw counter-clockwise from east
struct ObjectTrack {
    TrackId id = 0;
    UKF filter;
    double t = 0.0;  // time of the filter state [s]

    // without the filter (EngineParams::enableKalman off): the latest accepted reports, the OLS window of the
    // history-aware CTRA
    std::deque<KinematicState> history;
    bool lastHasVelocity = false;  // a fused report carried a velocity: the track has a speed to check against
};

#endif  // OBJECT_TRACK_H_
