#ifndef MECHANISM_CONSTANTS_H_
#define MECHANISM_CONSTANTS_H_

#include <cstddef>

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

// Measurement-noise floor of the UKF's position R [m]: the spread of the generator's own reports about the
// ground truth with no noise injected (CAM/CPM reference points, lat/lon rounding; p95 0.06 m CAM, 0.11 m CPM)
constexpr double POS_NOISE_FLOOR = 0.1;

}  // namespace ConstantsAssociation

namespace ConstantsPredictive {

// Predictive check: largest position residual to the track projected to the report time [m],
// static d_max_0 of paper 1
constexpr double DEFAULT_MAX_POS_DEV = 4.3506;

// Without the Kalman filter: OLS window of the history-aware CTRA, the latest accepted reports of a track
constexpr std::size_t HISTORY_WINDOW = 8;

}  // namespace ConstantsPredictive

namespace ConstantsConsistency {

// Consistency check against the track
constexpr double DEFAULT_SPEED_TOLERANCE_MPS = 2.0;
constexpr double FROZEN_SPEED_THRESHOLD = 2.0;
// @See scripts/sensitivity.py
constexpr double DEFAULT_FROZEN_SCALE = 0.25;
// reports closer in time to the track's latest update than this are not judged frozen (two sources of one instant)
constexpr double FROZEN_MIN_DT = 0.05;  // [s]

}  // namespace ConstantsConsistency

namespace ConstantsUkf {

// Calibrated on simulator ground truth of the validation window; the test window is never used to calibrate

// CTRA process noise: standard deviations of the noise drawn at every UKF Prediction() call, which
// UKF::Predict makes at most 0.1 s long (0.05 s pieces, then the remainder); valid for that scheme only.
// Maximum likelihood on the error of v and yaw predicted from exact truth states, over the intervals
// between reports of a station (0.1-1 s)
constexpr double DEFAULT_STD_JERK = 11.1;      // longitudinal jerk [m/s^3]
constexpr double DEFAULT_STD_YAW_ACCEL = 2.1;  // yaw acceleration [rad/s^2]

// Initial state covariance, diagonal over [px, py, v, yaw, yaw rate, a]
// [m^2, m^2, (m/s)^2, rad^2, (rad/s)^2, (m/s^2)^2]: the measured states start from the first vector, so
// the squares of the UKF laser noise (0.15 m, 0.15 m, 0.29 m/s, 0.12 rad); yaw rate and a start at 0, so
// their truth second moments at report times. Tracker::initiateTrack replaces px, py by the effective position R
// (EngineParams::measNoise)
constexpr double DEFAULT_P0[6] = {0.0225, 0.0225, 0.0841, 0.0144, 0.0753, 3.50};

// same, for a first vector without velocity: v and yaw start at 0, so the truth second moment of v and
// the variance of a uniform yaw, pi^2 / 3
constexpr double DEFAULT_P0_NO_VELOCITY[6] = {0.0225, 0.0225, 114.3, 3.29, 0.0753, 3.50};

}  // namespace ConstantsUkf

namespace ConstantsScore {

// Beta reputation with forgetting (Josang & Ismail, Bled 2002, Sec. 2.6); lambda = 0.9 as paper 1
constexpr double DEFAULT_FORGETTING = 0.9;

}  // namespace ConstantsScore

#endif  // MECHANISM_CONSTANTS_H_
