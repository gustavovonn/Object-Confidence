#ifndef MOTION_VECTOR_H_
#define MOTION_VECTOR_H_

#include <cstdint>

// Container that carried the vector; used only to pick the measurement noise
enum class Origin : uint8_t {
    Cam,  // the author reports itself
    Cpm   // the author reports a perceived object
};

// One received report about one object, independent of the message format.
// Local tangent plane: x east, y north [m]; heading clockwise from north.
struct MotionVector {
    uint64_t author = 0;     // station that sent the message
    int64_t objectId = -1;   // CPM objectId; -1 when the author is the object (CAM)
    Origin origin = Origin::Cam;

    double t = 0.0;          // measurement time [s]
    double x = 0.0;          // [m]
    double y = 0.0;          // [m]

    bool hasVelocity = false;
    double speed = 0.0;      // [m/s]
    double heading = 0.0;    // [rad]

    int64_t trueStation = -1;  // oracle, evaluation only
};

#endif  // MOTION_VECTOR_H_
