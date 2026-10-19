#ifndef MECHANISM_CONSTANTS_H_
#define MECHANISM_CONSTANTS_H_

// Defaults of the object-trajectory confidence mechanism. Every value names its source.
// "Paper 1": A Lightweight Mechanism to Attribute Confidence to V2X Messages based on Physical
// Plausibility (CP-Artery, include/MechanismConstants.h).

#include <cmath>

namespace ConstantsAssociation {

// Association and fusion gate on the position residual [m]: static d_max_0 of paper 1,
// calibrated on the fault-free VeReMi Extension subset (Sec. V-A)
constexpr double DEFAULT_GATE_DISTANCE = 4.3506;

// Track retired after this long without a fused report [s]: paper 1 MAX_INTER_FRAME_TIME (F2MD)
constexpr double DEFAULT_RETIRE_AFTER = 3.1;

}  // namespace ConstantsAssociation

namespace ConstantsConsistency {

// Paper 1 consistency check (Sec. IV-B-1)
constexpr double MIN_INTER_FRAME_TIME = 0.1;         // [s] ETSI EN 302 637-2 minimum CAM interval
constexpr double MAX_INTER_FRAME_TIME = 3.1;         // [s] F2MD
constexpr double DEFAULT_SPEED_TOLERANCE_MPS = 4.0;  // [m/s] paper 1, fault-free VeReMi Extension
constexpr double FROZEN_SPEED_THRESHOLD = 2.0;       // [m/s] paper 1, reference pending there
constexpr double FROZEN_DISPLACEMENT_MIN = 0.3;      // [m] paper 1

}  // namespace ConstantsConsistency

namespace ConstantsNoise {

// Process noise: magnitudes of e_a and e_w, Schubert et al., FUSION 2008, Table I
constexpr double DEFAULT_JERK_STD = 0.5;       // [m/s^3]
constexpr double DEFAULT_YAW_ACCEL_STD = 0.25; // [rad/s^2]

// Measurement noise from the VeReMi Extension sensor errors (Kamel et al., ICC 2020, Sec. IV-B)
// position: E0 ~ U[-5, 5] m per axis -> std 10/sqrt(12)
constexpr double DEFAULT_POSITION_STD = 10.0 / 3.4641016151377544;  // [m]
// heading: E0 ~ U[-20, 20] deg (at standstill) -> std 40/sqrt(12)
constexpr double DEFAULT_HEADING_STD = (40.0 / 3.4641016151377544) * M_PI / 180.0;  // [rad]
// speed: VeReMi error negligible; CDD SpeedValue resolution 0.01 m/s -> std 0.01/sqrt(12)
constexpr double DEFAULT_SPEED_STD = 0.01 / 3.4641016151377544;  // [m/s]

// Prior std of unmeasured state components (acceleration, yaw rate, missing velocity):
// lisha-artery monitorAV/ukf initial covariance P = I
constexpr double DEFAULT_UNMEASURED_STD = 1.0;

}  // namespace ConstantsNoise

namespace ConstantsScore {

// Beta reputation with forgetting (Josang & Ismail, Bled 2002, Sec. 2.6); lambda = 0.9 as paper 1
constexpr double DEFAULT_FORGETTING = 0.9;

}  // namespace ConstantsScore

#endif  // MECHANISM_CONSTANTS_H_
