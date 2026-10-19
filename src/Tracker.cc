#include "Tracker.h"

#include "Constants.h"
#include "CtraProcessModel.h"
#include "Hungarian.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

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

bool Tracker::residual(const ObjectTrack& track, const MotionVector& v, ukf::Prediction& out, double& residual) const
{
    if (!project(track, v.t, out)) {
        return false;
    }
    residual = std::hypot(v.x - out.x(CtraState::X), v.y - out.x(CtraState::Y));
    return true;
}

std::vector<uint64_t> Tracker::associate(const std::vector<const MotionVector*>& vectors)
{
    std::vector<uint64_t> trackOf(vectors.size(), 0);

    std::vector<std::size_t> unbound;
    for (std::size_t i = 0; i < vectors.size(); ++i) {
        auto b = m_bindings.find(keyOf(*vectors[i]));
        if (b != m_bindings.end() && find(b->second.trackId) != nullptr) {
            trackOf[i] = b->second.trackId;
        } else {
            unbound.push_back(i);
        }
    }
    if (unbound.empty() || m_tracks.empty()) {
        return trackOf;
    }

    // a CAM is the station's own report: one CAM source per track
    std::set<uint64_t> withCam;
    for (const auto& b : m_bindings) {
        if (b.first.objectId < 0) {
            withCam.insert(b.second.trackId);
        }
    }

    // outside the gate: finite but above any sum of gated costs, so feasible matches come first
    const double infeasible = 1e6;
    std::vector<std::vector<double>> cost(unbound.size(), std::vector<double>(m_tracks.size(), infeasible));
    ukf::Prediction p;
    for (std::size_t r = 0; r < unbound.size(); ++r) {
        const MotionVector& v = *vectors[unbound[r]];
        for (std::size_t c = 0; c < m_tracks.size(); ++c) {
            // an author reports each object once
            if (authorBound(v.author, m_tracks[c].id) || (v.objectId < 0 && withCam.count(m_tracks[c].id))) {
                continue;
            }
            double d = 0.0;
            if (residual(m_tracks[c], v, p, d) && d <= m_params.gateDistance) {
                cost[r][c] = d;
            }
        }
    }

    std::vector<int> assignment;
    HungarianAlgorithm().Solve(cost, assignment);
    for (std::size_t r = 0; r < unbound.size(); ++r) {
        const int c = assignment[r];
        if (c >= 0 && cost[r][c] <= m_params.gateDistance) {
            trackOf[unbound[r]] = m_tracks[c].id;
        }
    }
    return trackOf;
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

bool Tracker::authorBound(uint64_t author, uint64_t trackId) const
{
    for (auto it = m_bindings.lower_bound(SourceKey{author, std::numeric_limits<int64_t>::min()}); it != m_bindings.end() && it->first.author == author;
         ++it) {
        if (it->second.trackId == trackId) {
            return true;
        }
    }
    return false;
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
