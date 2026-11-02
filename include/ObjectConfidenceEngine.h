#ifndef OBJECT_CONFIDENCE_ENGINE_H_
#define OBJECT_CONFIDENCE_ENGINE_H_

#include "ConsistencyCheck.h"
#include "EngineParams.h"
#include "IScorer.h"
#include "MotionVector.h"
#include "StageTimer.h"
#include "Tracker.h"
#include "VectorRecord.h"

#include <cstdint>
#include <memory>
#include <vector>

// Receiver-local confidence of fused object trajectories.
//   0 retireStaleTracks   tracks without a fused report for retireAfter, before association
//   1 plausibilityCheck   finite values
//   2 associate           per message: bound source keeps its track; unbound sources jointly,
//                         one per track, inside the gate (Hungarian)
//   3 checks              per vector with a track: consistency against the source's last report; predictive
//                         check, the position residual to the track projected to the vector time
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
    bool predictiveCheck(double residual) const;

    const Tracker& tracker() const { return m_tracker; }
    const IScorer& scorer() const { return *m_scorer; }
    const EngineParams& params() const { return m_params; }

    // Per-stage timing; null disables it
    void setStageTimer(StageTimer* timer) { m_timer = timer; }

private:
    void process(const MotionVector& v, uint64_t trackId, VectorRecord& r);

    EngineParams m_params;
    Tracker m_tracker;
    std::unique_ptr<IScorer> m_scorer;
    ConsistencyThresholds m_consistency;
    StageTimer* m_timer = nullptr;
};

#endif  // OBJECT_CONFIDENCE_ENGINE_H_
