#ifndef MECHANISM_CONSTANTS_H_
#define MECHANISM_CONSTANTS_H_

// Defaults of the object-trajectory confidence mechanism. Every value names its source.
// "Paper 1": A Lightweight Mechanism to Attribute Confidence to V2X Messages based on Physical
// Plausibility (CP-Artery, include/MechanismConstants.h).

namespace ConstantsAssociation {

// Association gate along the projected track heading, semi-axis [m]
constexpr double DEFAULT_GATE_ALONG = 4.3506;

// Association gate across the projected track heading, semi-axis [m]: half the 3.2 m SUMO default
// lane width, so vehicles side by side in adjacent lanes are not associated
constexpr double DEFAULT_GATE_ACROSS = 1.6;

// Track retired after this long without a fused report [s]
constexpr double DEFAULT_RETIRE_AFTER = 1.5;

// A source unreported for longer than this no longer holds its author's place on a track [s]:
// ETSI TS 103 324 V2.1.1 T_GenCpmMax, the longest a perceived vehicle may go without inclusion
// in a CPM, so the sender has dropped its object 
constexpr double DEFAULT_SOURCE_ACTIVE_WINDOW = 1.0;

}  // namespace ConstantsAssociation

namespace ConstantsPredictive {

// Predictive check: largest position residual to the track projected to the report time [m],
// static d_max_0 of paper 1
constexpr double DEFAULT_MAX_POS_DEV = 4.3506;

}  // namespace ConstantsPredictive

namespace ConstantsConsistency {

// Paper 1 consistency check (Sec. IV-B-1)
constexpr double MIN_INTER_FRAME_TIME = 0.1;         // [s] ETSI EN 302 637-2 minimum CAM interval
constexpr double MAX_INTER_FRAME_TIME = 3.1;         // [s] F2MD
constexpr double DEFAULT_SPEED_TOLERANCE_MPS = 4.0;  // [m/s] paper 1, fault-free VeReMi Extension
constexpr double FROZEN_SPEED_THRESHOLD = 2.0;       // [m/s] paper 1, reference pending there
constexpr double FROZEN_DISPLACEMENT_MIN = 0.3;      // [m] paper 1

}  // namespace ConstantsConsistency

namespace ConstantsUkf {

// Calibrated on simulator ground truth

// CTRA process noise: standard deviations of the noise drawn at every UKF Prediction() call, which
// UKF::Predict makes at most 0.1 s long (0.05 s pieces, then the remainder); valid for that scheme only.
// Maximum likelihood on the error of v and yaw predicted from exact truth states, over the intervals
// between reports of a station (0.1-1 s)
constexpr double DEFAULT_STD_JERK = 10.2;      // longitudinal jerk [m/s^3]
constexpr double DEFAULT_STD_YAW_ACCEL = 2.7;  // yaw acceleration [rad/s^2]

// Initial state covariance, diagonal over [px, py, v, yaw, yaw rate, a]
// [m^2, m^2, (m/s)^2, rad^2, (rad/s)^2, (m/s^2)^2]: the measured states start from the first vector, so
// the squares of the UKF laser noise (0.15 m, 0.15 m, 0.29 m/s, 0.12 rad); yaw rate and a start at 0, so
// their truth second moments at report times
constexpr double DEFAULT_P0[6] = {0.0225, 0.0225, 0.0841, 0.0144, 0.0526, 3.42};

// same, for a first vector without velocity: v and yaw start at 0, so the truth second moment of v and
// the variance of a uniform yaw, pi^2 / 3
constexpr double DEFAULT_P0_NO_VELOCITY[6] = {0.0225, 0.0225, 329.0, 3.29, 0.0526, 3.42};

}  // namespace ConstantsUkf

namespace ConstantsScore {

// Beta reputation with forgetting (Josang & Ismail, Bled 2002, Sec. 2.6); lambda = 0.9 as paper 1
constexpr double DEFAULT_FORGETTING = 0.9;

}  // namespace ConstantsScore

#endif  // MECHANISM_CONSTANTS_H_
