#include "Tracker.h"

#include "Constants.h"
#include "CtraProcessModel.h"

#include <algorithm>
#include <cmath>

namespace {

SourceKey keyOf(const MotionVector& v)
{
    return SourceKey{v.author, v.objectId};
}

KinematicState reportOf(const MotionVector& v)
{
    KinematicState s;
    s.lon = v.x;
    s.lat = v.y;
    s.speed = v.speed;
    s.heading = v.heading * ConstantsWGS84::RAD_TO_DEG;
    s.time = v.t;
    return s;
}

Eigen::VectorXd measurePositionVelocity(const Eigen::VectorXd& x)
{
    using namespace CtraState;
    Eigen::VectorXd z(4);
    z << x(X), x(Y), x(SPEED), x(HEADING);
    return z;
}

Eigen::VectorXd measurePosition(const Eigen::VectorXd& x)
{
    using namespace CtraState;
    Eigen::VectorXd z(2);
    z << x(X), x(Y);
    return z;
}

}  // namespace

Tracker::Tracker(const EngineParams& params)
    : m_params(params)
    , m_ukf(CtraState::SIZE, CtraState::NOISE_SIZE, ctraProcess, CtraState::HEADING)
{
    m_Q = Eigen::MatrixXd::Zero(CtraState::NOISE_SIZE, CtraState::NOISE_SIZE);
    m_Q(CtraState::JERK, CtraState::JERK) = params.jerkStd * params.jerkStd;
    m_Q(CtraState::YAW_ACCEL, CtraState::YAW_ACCEL) = params.yawAccelStd * params.yawAccelStd;
}

bool Tracker::project(const ObjectTrack& track, double t, ukf::Prediction& out) const
{
    // out-of-sequence reports are compared with the latest state
    const double dt = std::max(0.0, t - track.t);
    return m_ukf.predict(track.x, track.P, m_Q, dt, out);
}

Tracker::Candidate Tracker::associate(const MotionVector& v)
{
    Candidate best;

    auto bound = m_bindings.find(keyOf(v));
    if (bound != m_bindings.end()) {
        if (ObjectTrack* track = find(bound->second.trackId)) {
            if (project(*track, v.t, best.prediction)) {
                best.track = track;
                best.bound = true;
                best.residual = std::hypot(v.x - best.prediction.x(CtraState::X),
                                           v.y - best.prediction.x(CtraState::Y));
                return best;
            }
        }
    }

    Candidate c;
    for (ObjectTrack& track : m_tracks) {
        if (!project(track, v.t, c.prediction)) {
            continue;
        }
        const double residual = std::hypot(v.x - c.prediction.x(CtraState::X), v.y - c.prediction.x(CtraState::Y));
        if (residual <= m_params.gateDistance && (best.track == nullptr || residual < best.residual)) {
            best.track = &track;
            best.prediction = c.prediction;
            best.residual = residual;
        }
    }
    return best;
}

void Tracker::kalmanUpdate(ObjectTrack& track, const ukf::Prediction& prediction, const MotionVector& v)
{
    const double sp = m_params.positionStd * m_params.positionStd;
    ukf::Innovation in;
    if (v.hasVelocity) {
        Eigen::VectorXd z(4);
        z << v.x, v.y, v.speed, v.heading;
        const Eigen::Vector4d r(sp, sp, m_params.speedStd * m_params.speedStd, m_params.headingStd * m_params.headingStd);
        in = m_ukf.innovate(prediction, measurePositionVelocity, z, r.asDiagonal().toDenseMatrix(), 3);
    } else {
        Eigen::VectorXd z(2);
        z << v.x, v.y;
        in = m_ukf.innovate(prediction, measurePosition, z, Eigen::Vector2d(sp, sp).asDiagonal().toDenseMatrix());
    }
    m_ukf.update(prediction, in, track.x, track.P);
    track.t = std::max(track.t, v.t);
}

ObjectTrack& Tracker::initiateTrack(const MotionVector& v)
{
    using namespace CtraState;

    const double sp = m_params.positionStd * m_params.positionStd;
    const double su = m_params.unmeasuredStd * m_params.unmeasuredStd;

    ObjectTrack track;
    track.id = m_nextId++;
    track.t = v.t;
    track.x = Eigen::VectorXd::Zero(SIZE);
    track.x(X) = v.x;
    track.x(Y) = v.y;

    Eigen::VectorXd var = Eigen::VectorXd::Constant(SIZE, su);
    var(X) = sp;
    var(Y) = sp;
    if (v.hasVelocity) {
        track.x(HEADING) = ukf::wrapPi(v.heading);
        track.x(SPEED) = v.speed;
        var(HEADING) = m_params.headingStd * m_params.headingStd;
        var(SPEED) = m_params.speedStd * m_params.speedStd;
    }
    track.P = var.asDiagonal();

    m_tracks.push_back(std::move(track));
    return m_tracks.back();
}

std::vector<uint64_t> Tracker::retireStaleTracks(double now)
{
    std::vector<uint64_t> retired;
    for (const ObjectTrack& t : m_tracks) {
        if (now - t.t > m_params.retireAfter) {
            retired.push_back(t.id);
        }
    }
    if (retired.empty()) {
        return retired;
    }

    auto isRetired = [&retired](uint64_t id) {
        return std::find(retired.begin(), retired.end(), id) != retired.end();
    };
    m_tracks.erase(std::remove_if(m_tracks.begin(), m_tracks.end(),
                                  [&](const ObjectTrack& t) { return isRetired(t.id); }),
                   m_tracks.end());
    for (auto it = m_bindings.begin(); it != m_bindings.end();) {
        it = isRetired(it->second.trackId) ? m_bindings.erase(it) : std::next(it);
    }
    return retired;
}

const SourceBinding* Tracker::binding(const MotionVector& v) const
{
    auto it = m_bindings.find(keyOf(v));
    return it != m_bindings.end() ? &it->second : nullptr;
}

void Tracker::bind(const MotionVector& v, uint64_t trackId)
{
    SourceBinding& b = m_bindings[keyOf(v)];
    b.trackId = trackId;
    b.last = reportOf(v);
}

void Tracker::recordReport(const MotionVector& v)
{
    auto it = m_bindings.find(keyOf(v));
    if (it != m_bindings.end()) {
        it->second.last = reportOf(v);
    }
}

ObjectTrack* Tracker::find(uint64_t id)
{
    for (ObjectTrack& t : m_tracks) {
        if (t.id == id) {
            return &t;
        }
    }
    return nullptr;
}
