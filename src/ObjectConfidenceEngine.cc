#include "ObjectConfidenceEngine.h"

#include "Angles.h"
#include "Constants.h"
#include "MechanismConstants.h"

#include <algorithm>
#include <cmath>

ObjectConfidenceEngine::ObjectConfidenceEngine(EngineParams params)
    : m_params(params), m_tracker(params), m_scorer(params.forgetting), m_authorScorer(params.forgetting),
      m_authorKindScorer(params.forgetting)
{
}

std::vector<VectorRecord> ObjectConfidenceEngine::onMotionVectors(const std::vector<MotionVector>& vectors,
                                                                  uint64_t receiver, double now)
{
    MaybeStage whole(m_timer, "onMotionVectors");

    // stale tracks leave before association, so no vector is compared with a dead track
    {
        MaybeStage s(m_timer, "0 retire");
        // Paradigm choice: a retired track's evidence is erased, an author's never is (the author scorer grows
        // with the authors this receiver has heard; forgetting decays it only when new evidence arrives)
        for (TrackId id : m_tracker.retireStaleTracks(now)) {
            m_scorer.forget(id);
        }
    }

    // projections of the previous message are stale: tracks were updated and time moved on
    m_tracker.clearProjections();
    m_liveTracks = m_tracker.tracks().size();

    std::vector<VectorRecord> out(vectors.size());
    std::vector<const MotionVector*> plausibleVectors;
    std::vector<std::size_t> plausibleIndex;
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
                plausibleVectors.push_back(&v);
                plausibleIndex.push_back(i);
            } else {
                r.outcome = Outcome::Dropped;
            }
        }
    }

    std::vector<TrackId> trackIds;
    {
        MaybeStage s(m_timer, "2 associate");
        trackIds = m_tracker.associate(plausibleVectors);
    }

    for (std::size_t k = 0; k < plausibleVectors.size(); ++k) {
        process(*plausibleVectors[k], trackIds[k], out[plausibleIndex[k]]);
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
 * @brief Consistency against the track: the reported speed against the track's speed projected to the report
 * time (speed check), and a moving object reported too close to the track's position at its latest update
 * (frozen rule). Needs a velocity in the report and in the track; passes without one
 * @return true when passed
 */
bool ObjectConfidenceEngine::consistencyCheck(const ObjectTrack& track, const MotionVector& v,
                                              const UKF& predicted) const
{
    if (!m_params.enableConsistency || !v.hasVelocity || !track.lastHasVelocity) {
        return true;
    }
    if (std::abs(v.speed - predicted.x_(2)) > m_params.speedToleranceMps) {
        return false;
    }
    const double dt = std::abs(v.t - track.t);
    if (!m_params.enableFrozen || dt < ConstantsConsistency::FROZEN_MIN_DT) {
        return true;
    }
    const bool kalman = m_params.enableKalman;
    const double x = kalman ? track.filter.x_(0) : track.history.back().lon;
    const double y = kalman ? track.filter.x_(1) : track.history.back().lat;
    const double speed = 0.5 * (v.speed + (kalman ? track.filter.x_(2) : track.history.back().speed));
    const bool frozen = speed > ConstantsConsistency::FROZEN_SPEED_THRESHOLD
                        && std::hypot(v.x - x, v.y - y) < m_params.frozenScale * speed * dt;
    return !frozen;
}

/**
 * @brief The vector's verdict as evidence on its track, its author and its author and kind, whatever the paradigm:
 * the paradigms differ only in the confidence they report (combine), so one run records all three
 */
void ObjectConfidenceEngine::score(const MotionVector& v, TrackId trackId, bool passed, VectorRecord& r)
{
    r.objectConfidence = passed ? m_scorer.hit(trackId) : m_scorer.miss(trackId);
    r.authorConfidence = passed ? m_authorScorer.hit(v.author) : m_authorScorer.miss(v.author);
    const uint64_t kindKey = getAuthorKindKey(v);
    r.authorKindConfidence = passed ? m_authorKindScorer.hit(kindKey) : m_authorKindScorer.miss(kindKey);
    r.confidence = combine(r);
}

// the author and the container kind (CAM 0, CPM 1) in the lowest bit
uint64_t ObjectConfidenceEngine::getAuthorKindKey(const MotionVector& v)
{
    return (v.author << 1) | (v.origin == Origin::Cpm ? 1u : 0u);
}

// the confidence of EngineParams::confidence: the track's, or the lower of the track's and the author's (or author
// and kind)
double ObjectConfidenceEngine::combine(const VectorRecord& r) const
{
    switch (m_params.confidence) {
        case ConfidenceMode::MinAuthor: return std::min(r.objectConfidence, r.authorConfidence);
        case ConfidenceMode::MinAuthorKind: return std::min(r.objectConfidence, r.authorKindConfidence);
        default: return r.objectConfidence;
    }
}

bool ObjectConfidenceEngine::predictiveCheck(double residual) const
{
    return !m_params.enablePredictive || residual <= m_params.maxPosDev;
}

/**
 * @brief Pipeline for each single state motion vector: check, associate,
 * update, score
 */
void ObjectConfidenceEngine::process(const MotionVector& v, TrackId trackId, VectorRecord& r)
{
    ObjectTrack* track = trackId != 0 ? m_tracker.getObjectTrack(trackId) : nullptr;
    double residual = -1.0;
    bool projected = false;
    bool passed = false;
    if (track != nullptr) {
        MaybeStage s(m_timer, "3 checks");
        const UKF& predicted = m_tracker.projected(*track, v.t);
        residual = std::hypot(v.x - predicted.x_(0), v.y - predicted.x_(1));
        projected = std::isfinite(residual);
        if (projected) {
            r.consistencyFailed = !consistencyCheck(*track, v, predicted);
            r.predictiveFailed = !predictiveCheck(residual);
            if (m_params.recordState) {
                r.predX = predicted.x_(0);
                r.predY = predicted.x_(1);
                r.vhat = predicted.x_(2);
            }

            passed = !r.predictiveFailed && !r.consistencyFailed;
        }
    }

    // New track
    if (!projected) {
        // Creates the new track and starts the UKF
        const ObjectTrack& started = m_tracker.initiateTrack(v);

        m_tracker.setBind(v, started.id);
        r.outcome = Outcome::StartedTrack;
        r.trackId = started.id;
        // a new track starts trusted and adds no evidence: its confidence is its author's
        r.objectConfidence = 1.0;
        r.authorConfidence = m_authorScorer.getConfidence(v.author);
        r.authorKindConfidence = m_authorKindScorer.getConfidence(getAuthorKindKey(v));
        r.confidence = combine(r);
        recordState(started, r);
        return;
    }

    // a source that lost its track to another object of its message is bound again where it is fused
    const bool bound = m_tracker.getTrackId(v) == track->id;
    r.trackId = track->id;
    r.residual = residual;

    // If the object passed the checks, update the known track with the measurement
    if (passed) {
        MaybeStage s(m_timer, "4 kalmanUpdate");
        m_tracker.kalmanUpdate(*track, v);
        if (m_params.recordState && m_params.enableKalman && v.hasVelocity && v.t >= track->t) {  // fused
            r.nis = track->filter.NIS_laser_;
        }
        if (!bound) {
            m_tracker.setBind(v, track->id);
        }
    }

    {
        MaybeStage s(m_timer, "5 score");
        r.outcome = passed ? Outcome::Fused : Outcome::Rejected;
        score(v, r.trackId, passed, r);
    }

    recordState(*track, r);
}

/**
 * @brief The track's reference state after a vector (EngineParams::recordState): the filter's, or the latest
 * accepted report's without the Kalman filter
 */
void ObjectConfidenceEngine::recordState(const ObjectTrack& track, VectorRecord& r) const
{
    if (!m_params.recordState) {
        return;
    }
    r.stateT = track.t;
    if (m_params.enableKalman) {
        r.stateX = track.filter.x_(0);
        r.stateY = track.filter.x_(1);
        r.stateV = track.filter.x_(2);
        r.stateYaw = track.filter.x_(3);
    } else {
        const KinematicState& last = track.history.back();
        r.stateX = last.lon;
        r.stateY = last.lat;
        r.stateV = last.speed;
        r.stateYaw = Angles::wrapPi(Angles::compassToCartesianDeg(last.heading) * ConstantsWGS84::DEG_TO_RAD);
    }
}
