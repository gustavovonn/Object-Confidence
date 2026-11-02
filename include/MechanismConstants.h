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

namespace ConstantsScore {

// Beta reputation with forgetting (Josang & Ismail, Bled 2002, Sec. 2.6); lambda = 0.9 as paper 1
constexpr double DEFAULT_FORGETTING = 0.9;

// Gaussian NIS and Windowed NIS: position innovation s.d. [m].
// Calibrate on fault-free data so that the median NIS ≈ 1 (chi-squared(2) median ≈ 1.39).
// A conservative starting point for vehicular datasets is 3 m.
constexpr double DEFAULT_INNOVATION_SIGMA = 3.0;

// Windowed NIS: number of most-recent samples kept per track.
constexpr int DEFAULT_NIS_WINDOW_SIZE = 10;

// Windowed NIS: warm-up guard — return 0.5 until this many samples are in the window.
constexpr int DEFAULT_NIS_MIN_SAMPLES = 3;

// NIS chi-squared(2) threshold at alpha = 0.05 (95th percentile).
// Used by both GaussianNIS (as the neutral point of the logistic) and WindowedNIS.
constexpr double DEFAULT_NIS_THRESHOLD = 5.991;

}  // namespace ConstantsScore

#endif  // MECHANISM_CONSTANTS_H_
