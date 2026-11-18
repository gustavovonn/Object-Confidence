#ifndef OBJECT_CONFIDENCE_ENGINE_H_
#define OBJECT_CONFIDENCE_ENGINE_H_

#include "EngineParams.h"
#include "MotionVector.h"
#include "Scorer.h"
#include "StageTimer.h"
#include "Tracker.h"
#include "VectorRecord.h"

#include <cstdint>
#include <vector>

// Receiver-local confidence of fused object trajectories.
//   0 retireStaleTracks   tracks without a fused report for retireAfter, before association
//   1 plausibilityCheck   finite values
//   2 associate           per message: bound source keeps its track; unbound sources jointly,
//                         one per track, inside the gate (Hungarian)
//   3 checks              per vector with a track, all against the track projected to the vector time:
//                         consistency (speed, frozen position) and predictive (position residual)
//   4 kalmanUpdate        only vectors that passed the checks
//   5 score               hit or miss on the track, and on the author (EngineParams::confidence)
class ObjectConfidenceEngine
{
public:
    explicit ObjectConfidenceEngine(EngineParams params = {});

    // Vectors of one received message, in any container
    std::vector<VectorRecord> onMotionVectors(const std::vector<MotionVector>& vectors, uint64_t receiver, double now);

    bool plausibilityCheck(const MotionVector& v) const;
    bool consistencyCheck(const ObjectTrack& track, const MotionVector& v, const UKF& predicted) const;
    bool predictiveCheck(double residual) const;

    const Tracker& tracker() const { return m_tracker; }
    const Scorer& scorer() const { return m_scorer; }
    const EngineParams& params() const { return m_params; }

    // Per-stage timing; null disables it
    void setStageTimer(StageTimer* timer) { m_timer = timer; }

    // tracks the latest message was associated against (after retirement): the receiver's neighbourhood
    std::size_t liveTracks() const { return m_liveTracks; }

private:
    void recordState(const ObjectTrack& track, VectorRecord& r) const;
    void process(const MotionVector& v, TrackId trackId, VectorRecord& r);
    void score(const MotionVector& v, TrackId trackId, bool passed, VectorRecord& r);
    static uint64_t getAuthorKindKey(const MotionVector& v);
    double combine(const VectorRecord& r) const;

    EngineParams m_params;
    StageTimer* m_timer = nullptr;
    std::size_t m_liveTracks = 0;

    Tracker m_tracker;

    Scorer m_scorer;        // per track
    Scorer m_authorScorer;      // per author
    Scorer m_authorKindScorer;  // per author and container kind
};

#endif  // OBJECT_CONFIDENCE_ENGINE_H_
