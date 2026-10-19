#ifndef OBJECT_CONFIDENCE_ENGINE_H_
#define OBJECT_CONFIDENCE_ENGINE_H_

#include "ConsistencyCheck.h"
#include "EngineParams.h"
#include "MotionVector.h"
#include "Scorer.h"
#include "StageTimer.h"
#include "Tracker.h"
#include "VectorRecord.h"

#include <cstdint>
#include <vector>

// Receiver-local confidence of fused object trajectories.
// Stage order as Allig et al., VNC 2019, Sec. II-A: align, check, associate, fuse.
//   0 retireStaleTracks   tracks without a fused report for retireAfter, before association
//   1 plausibilityCheck   finite values
//   2 associate           per message: bound source keeps its track; unbound sources jointly,
//                         one per track, inside the gate (Hungarian)
//   3 checks              per vector: track projected to the vector time; consistency against the
//                         source's last report; position gate
//   4 kalmanUpdate        only vectors that passed the checks
//   5 score               hit or miss on the track
class ObjectConfidenceEngine
{
public:
    explicit ObjectConfidenceEngine(EngineParams params = {});

    // Vectors of one received message, in any container
    std::vector<VectorRecord> onMotionVectors(const std::vector<MotionVector>& vectors, uint64_t receiver, double now);

    bool plausibilityCheck(const MotionVector& v) const;
    ConsistencyResult consistencyCheck(const MotionVector& v) const;
    bool gateCheck(double residual) const;

    const Tracker& tracker() const { return m_tracker; }
    const Scorer& scorer() const { return m_scorer; }
    const EngineParams& params() const { return m_params; }

    // Per-stage timing; null disables it
    void setStageTimer(StageTimer* timer) { m_timer = timer; }

private:
    void process(const MotionVector& v, uint64_t trackId, VectorRecord& r);

    EngineParams m_params;
    Tracker m_tracker;
    Scorer m_scorer;
    ConsistencyThresholds m_consistency;
    StageTimer* m_timer = nullptr;
};

#endif  // OBJECT_CONFIDENCE_ENGINE_H_
