#include "ObjectConfidenceEngine.h"

#include "CoordinatePolicies.h"
#include "MechanismConstants.h"

#include <cmath>

ObjectConfidenceEngine::ObjectConfidenceEngine(EngineParams params)
    : m_params(params), m_tracker(params), m_scorer(params.forgetting)
{
    m_consistency.speedTolerance = params.speedToleranceMps;
    m_consistency.minInterFrame = ConstantsConsistency::MIN_INTER_FRAME_TIME;
    m_consistency.maxInterFrame = ConstantsConsistency::MAX_INTER_FRAME_TIME;
    m_consistency.frozenSpeed = ConstantsConsistency::FROZEN_SPEED_THRESHOLD;
    m_consistency.frozenDisplacement = ConstantsConsistency::FROZEN_DISPLACEMENT_MIN;
}

std::vector<VectorRecord> ObjectConfidenceEngine::onMotionVectors(const std::vector<MotionVector>& vectors,
                                                                  uint64_t receiver, double now)
{
    MaybeStage whole(m_timer, "onMotionVectors");

    // stale tracks leave before association, so no vector is compared with a dead track
    {
        MaybeStage s(m_timer, "0 retire");
        for (uint64_t id : m_tracker.retireStaleTracks(now)) {
            m_scorer.forget(id);
        }
    }

    std::vector<VectorRecord> out(vectors.size());
    std::vector<const MotionVector*> kept;
    std::vector<std::size_t> keptIndex;
    {
        MaybeStage s(m_timer, "1 plausibility");
        for (std::size_t i = 0; i < vectors.size(); ++i) {
            const MotionVector& v = vectors[i];
            VectorRecord& r = out[i];
            r.t = v.t;
            r.receiver = receiver;
            r.author = v.author;
            r.objectId = v.objectId;
            r.origin = v.origin;
            r.trueStation = v.trueStation;
            if (plausibilityCheck(v)) {
                kept.push_back(&v);
                keptIndex.push_back(i);
            } else {
                r.outcome = Outcome::Dropped;
            }
        }
    }

    std::vector<uint64_t> trackOf;
    {
        MaybeStage s(m_timer, "2 associate");
        trackOf = m_tracker.associate(kept);
    }

    for (std::size_t k = 0; k < kept.size(); ++k) {
        process(*kept[k], trackOf[k], out[keptIndex[k]]);
    }

    return out;
}

bool ObjectConfidenceEngine::plausibilityCheck(const MotionVector& v) const
{
    const bool position = std::isfinite(v.t) && std::isfinite(v.x) && std::isfinite(v.y);
    const bool velocity = !v.hasVelocity || (std::isfinite(v.speed) && std::isfinite(v.heading) && v.speed >= 0.0);
    return position && velocity;
}

/**
 * @brief Simple displacement check consistency between consecutive states
 * @return check result, normalized velocity and excess ratio if failed
 */
ConsistencyResult ObjectConfidenceEngine::consistencyCheck(const MotionVector& v) const
{
    const SourceBinding* b = m_tracker.binding(v);
    if (!m_params.enableConsistency || b == nullptr || !v.hasVelocity) {
        return {};
    }
    KinematicState current = b->last;
    current.lon = v.x;
    current.lat = v.y;
    current.speed = v.speed;
    current.time = v.t;
    return ::consistencyCheck<CartesianCoordinatePolicy>(b->last, current, m_consistency);
}

bool ObjectConfidenceEngine::gateCheck(double residual) const
{
    return residual <= m_params.gateDistance;
}

/**
 * @brief Pipeline for each single state motion vector: check, associate,
 * update, score
 */
void ObjectConfidenceEngine::process(const MotionVector& v, uint64_t trackId, VectorRecord& r)
{
    ObjectTrack* track = trackId != 0 ? m_tracker.find(trackId) : nullptr;
    double residual = -1.0;
    bool projected = false;
    bool passed = false;
    if (track != nullptr) {
        MaybeStage s(m_timer, "3 checks");
        projected = m_tracker.residual(*track, v, residual);
        if (projected) {
            const ConsistencyResult consistency = consistencyCheck(v);
            r.consistencyFailed = consistency.failed;
            passed = gateCheck(residual) && !consistency.failed;
        }
    }

    // New track
    if (!projected) {
        // Creates the new track and starts the UKF
        const ObjectTrack& started = m_tracker.initiateTrack(v);

        m_tracker.bind(v, started.id);
        r.outcome = Outcome::StartedTrack;
        r.trackId = started.id;
        r.confidence = m_scorer.confidenceOf(started.id);
        return;
    }

    const bool bound = m_tracker.binding(v) != nullptr;
    r.trackId = track->id;
    r.residual = residual;

    // If the object passed the checks, update the known track with the measurement
    if (passed) {
        MaybeStage s(m_timer, "4 kalmanUpdate");
        m_tracker.kalmanUpdate(*track, v);
        if (!bound) {
            m_tracker.bind(v, track->id);
        }
    }

    {
        MaybeStage s(m_timer, "5 score");
        r.outcome = passed ? Outcome::Fused : Outcome::Rejected;
        r.confidence = passed ? m_scorer.hit(r.trackId) : m_scorer.miss(r.trackId);
    }

    m_tracker.recordReport(v);
}
