#include "Tracker.h"

#include "Constants.h"
#include "Hungarian.h"
#include "measurement_package.h"

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

long long microseconds(double t)
{
    return std::llround(t * 1e6);
}

// UKF yaw is counter-clockwise from east; reports carry a heading clockwise from north
double yawOf(double heading)
{
    return M_PI / 2.0 - heading;
}

// UKF's LASER measurement: [px, py, v, yaw]
MeasurementPackage laserOf(const MotionVector& v, long long timestamp)
{
    MeasurementPackage m;
    m.sensor_type_ = MeasurementPackage::LASER;
    m.raw_measurements_ = Eigen::VectorXd(4);
    m.raw_measurements_ << v.x, v.y, v.speed, yawOf(v.heading);
    m.timestamp_ = timestamp;
    return m;
}

}  // namespace

Tracker::Tracker(const EngineParams& params)
    : m_params(params)
{
}

void Tracker::project(const ObjectTrack& track, double t, UKF& out) const
{
    // out-of-sequence reports are compared with the latest state
    const double dt = std::max(0.0, t - track.t);
    // plain copy: UKF_copy() leaves std_laspx_ uninitialised
    out = track.filter;
    out.Prediction(dt);
}

bool Tracker::residual(const ObjectTrack& track, const MotionVector& v, double& residual) const
{
    UKF predicted;
    project(track, v.t, predicted);
    residual = std::hypot(v.x - predicted.x_(0), v.y - predicted.x_(1));
    return std::isfinite(residual);
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
    for (std::size_t r = 0; r < unbound.size(); ++r) {
        const MotionVector& v = *vectors[unbound[r]];
        for (std::size_t c = 0; c < m_tracks.size(); ++c) {
            // an author reports each object once
            if (authorBound(v.author, m_tracks[c].id) || (v.objectId < 0 && withCam.count(m_tracks[c].id))) {
                continue;
            }
            double d = 0.0;
            if (residual(m_tracks[c], v, d) && d <= m_params.gateDistance) {
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

void Tracker::kalmanUpdate(ObjectTrack& track, const MotionVector& v)
{
    // the UKF has no position-only measurement, so a report without velocity is not fused
    if (!v.hasVelocity) {
        return;
    }
    // out-of-sequence reports are fused at the latest state time, as in project()
    const long long timestamp = std::max(track.filter.time_us_, microseconds(v.t));
    track.filter.ProcessMeasurement(laserOf(v, timestamp));
    track.t = std::max(track.t, v.t);
}

ObjectTrack& Tracker::initiateTrack(const MotionVector& v)
{
    ObjectTrack track;
    track.id = m_nextId++;
    track.t = v.t;

    // the UKF initialises from its first measurement
    MeasurementPackage first = laserOf(v, microseconds(v.t));
    if (!v.hasVelocity) {
        // at rest, as the UKF's own RADAR initialisation
        first.raw_measurements_(2) = 0.0;
        first.raw_measurements_(3) = 0.0;
    }
    track.filter.ProcessMeasurement(first);

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
