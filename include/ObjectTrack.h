#ifndef OBJECT_TRACK_H_
#define OBJECT_TRACK_H_

#include <Eigen/Dense>

#include <cstdint>

// Fused reference state of one object; CtraState layout
struct ObjectTrack {
    uint64_t id = 0;
    Eigen::VectorXd x;  // mean
    Eigen::MatrixXd P;  // covariance
    double t = 0.0;     // time of x [s]
};

#endif  // OBJECT_TRACK_H_
