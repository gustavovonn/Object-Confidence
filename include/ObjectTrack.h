#ifndef OBJECT_TRACK_H_
#define OBJECT_TRACK_H_

#include "ukf.h"

#include <cstdint>

// Fused reference state of one object, held by the UKF.
// filter.x_ = [px, py, v, yaw, yaw rate]; yaw counter-clockwise from east
struct ObjectTrack {
    uint64_t id = 0;
    UKF filter;
    double t = 0.0;  // time of the filter state [s]
};

#endif  // OBJECT_TRACK_H_
