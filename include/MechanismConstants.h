#ifndef MECHANISM_CONSTANTS_H_
#define MECHANISM_CONSTANTS_H_

// Defaults of the object-trajectory confidence mechanism. Every value names its source.
// "Paper 1": A Lightweight Mechanism to Attribute Confidence to V2X Messages based on Physical
// Plausibility (CP-Artery, include/MechanismConstants.h).

namespace ConstantsAssociation {

// Association and fusion gate on the position residual [m]: static d_max_0 of paper 1,
// calibrated on the fault-free VeReMi Extension subset (Sec. V-A)
constexpr double DEFAULT_GATE_DISTANCE = 4.3506;

// Track retired after this long without a fused report [s]: paper 1 MAX_INTER_FRAME_TIME (F2MD)
constexpr double DEFAULT_RETIRE_AFTER = 3.1;

// A source unreported for longer than this no longer holds its author's place on a track [s]:
// ETSI TS 103 324 V2.1.1 T_GenCpmMax, the longest a perceived vehicle may go without inclusion
// in a CPM, so the sender has dropped its object 
constexpr double DEFAULT_SOURCE_ACTIVE_WINDOW = 1.0;

}  // namespace ConstantsAssociation

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

}  // namespace ConstantsScore

#endif  // MECHANISM_CONSTANTS_H_
