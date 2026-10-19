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

    std::vector<VectorRecord> out;
    out.reserve(vectors.size());
    for (const MotionVector& v : vectors) {
        out.push_back(process(v, receiver));
    }

    {
        MaybeStage s(m_timer, "6 retire");
        for (uint64_t id : m_tracker.retireStaleTracks(now)) {
            m_scorer.forget(id);
        }
    }
    return out;
}

bool ObjectConfidenceEngine::plausibilityCheck(const MotionVector& v) const
{
    const bool position = std::isfinite(v.t) && std::isfinite(v.x) && std::isfinite(v.y);
    const bool velocity = !v.hasVelocity || (std::isfinite(v.speed) && std::isfinite(v.heading) && v.speed >= 0.0);
    return position && velocity;
}

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

VectorRecord ObjectConfidenceEngine::process(const MotionVector& v, uint64_t receiver)
{
    VectorRecord r;
    r.t = v.t;
    r.receiver = receiver;
    r.author = v.author;
    r.objectId = v.objectId;
    r.origin = v.origin;
    r.trueStation = v.trueStation;

    {
        MaybeStage s(m_timer, "1 plausibility");
        if (!plausibilityCheck(v)) {
            r.outcome = Outcome::Dropped;
            return r;
        }
    }

    Tracker::Candidate c;
    {
        MaybeStage s(m_timer, "2 project + associate");
        c = m_tracker.associate(v);
    }

    if (c.track == nullptr) {
        const ObjectTrack& track = m_tracker.initiateTrack(v);
        m_tracker.bind(v, track.id);
        r.outcome = Outcome::StartedTrack;
        r.trackId = track.id;
        r.confidence = m_scorer.confidenceOf(track.id);
        return r;
    }

    r.trackId = c.track->id;
    r.residual = c.residual;

    bool passed = false;
    {
        MaybeStage s(m_timer, "3 checks");
        const ConsistencyResult consistency = consistencyCheck(v);
        r.consistencyFailed = consistency.failed;
        passed = gateCheck(c.residual) && !consistency.failed;
    }

    if (passed) {
        MaybeStage s(m_timer, "4 kalmanUpdate");
        m_tracker.kalmanUpdate(*c.track, c.prediction, v);
        if (!c.bound) {
            m_tracker.bind(v, c.track->id);
        }
    }

    {
        MaybeStage s(m_timer, "5 score");
        r.outcome = passed ? Outcome::Fused : Outcome::Rejected;
        r.confidence = passed ? m_scorer.hit(r.trackId) : m_scorer.miss(r.trackId);
    }

    m_tracker.recordReport(v);
    return r;
}
