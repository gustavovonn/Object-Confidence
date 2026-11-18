#include "Tracker.h"

#include "Angles.h"
#include "Constants.h"
#include "Hungarian.h"
#include "MechanismConstants.h"
#include "MotionModels.h"
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

long long microseconds(double t)
{
    return std::llround(t * 1e6);
}

// compass angle (clockwise from north) -> ENU yaw (counter-clockwise from east), the UKF's
// convention, wrapped to [-pi, pi]
double compassToEnuYaw(double compassRad)
{
    return Angles::wrapPi(M_PI / 2.0 - compassRad);
}

// keeps the report as the track state without the filter: appended to the OLS window, trimmed to its W latest
// reports; track.t is still the previous report's time
void updateTrackWindow(ObjectTrack& track, const MotionVector& v)
{
    if (v.hasVelocity && !track.lastHasVelocity) {
        // positions only so far: the window starts at the first report with a velocity
        track.history.clear();
    }
    KinematicState s;
    s.lon = v.x;
    s.lat = v.y;
    s.speed = v.hasVelocity ? v.speed : 0.0;
    s.heading = v.hasVelocity ? v.heading * ConstantsWGS84::RAD_TO_DEG : 0.0;
    s.time = v.t;
    track.history.push_back(s);
    while (track.history.size() > ConstantsPredictive::HISTORY_WINDOW) {
        track.history.pop_front();
    }
}

// the UKF's position R from the white measurement noise (EngineParams::measNoise); unset: the UKF's own value
void setPositionNoiseR(UKF& filter, double measNoise)
{
    if (measNoise < 0.0) {
        return;
    }
    const double sigma = std::hypot(measNoise, ConstantsAssociation::POS_NOISE_FLOOR);
    filter.std_laspx_ = sigma;
    filter.std_laspy_ = sigma;
}

MeasurementPackage laserOf(const MotionVector& v, long long timestamp)
{
    MeasurementPackage m;
    m.sensor_type_ = MeasurementPackage::LASER;
    m.raw_measurements_ = Eigen::VectorXd(4);
    m.raw_measurements_ << v.x, v.y, v.speed, compassToEnuYaw(v.heading);
    m.timestamp_ = timestamp;
    return m;
}

}  // namespace

Tracker::Tracker(const EngineParams& params)
    : m_params(params)
{
}

/**
 * @brief Projects the track to delta time t with a UKF copy
 * i. e. the track itself is not modified
 */
void Tracker::project(const ObjectTrack& track, double t, UKF& out) const
{
    const double dt = t - track.t;
    // plain copy: UKF_copy() leaves std_laspx_ uninitialised
    out = track.filter;
    if (!m_params.enableKalman) {
        // history-aware CTRA from the latest accepted report (OLS-smoothed acceleration and yaw rate over the window)
        const KinematicState& last = track.history.back();
        PredictedState next{last.lat, last.lon, last.speed, last.heading};  // dt 0: the report itself
        CartesianHistoryAwareCtraModel().predict(track.history, last, next, std::max(0.0, dt));
        out.x_.setZero();
        out.x_(0) = next.lon;
        out.x_(1) = next.lat;
        out.x_(2) = next.speed;
        out.x_(3) = compassToEnuYaw(next.heading * ConstantsWGS84::DEG_TO_RAD);
    } else {
        // same stepping as the fusion in ProcessMeasurement
        out.Predict(std::max(0.0, dt));
    }
    // FIX: a report older than the track state (a CPM object measured before a newer report of the same object was
    // fused) was compared with the newer state: an honest report off by v * |dt|, rejected at speed. The filter
    // does not run backwards, so the state is moved back along its velocity to the report time
    if (dt < 0.0) {
        out.x_(0) += out.x_(2) * std::cos(out.x_(3)) * dt;
        out.x_(1) += out.x_(2) * std::sin(out.x_(3)) * dt;
    }
}

const UKF& Tracker::projected(const ObjectTrack& track, double t) const
{
    const auto key = std::make_pair(track.id, t);
    auto it = m_projections.find(key);
    if (it == m_projections.end()) {
        it = m_projections.emplace(key, UKF()).first;
        project(track, t, it->second);
    }
    return it->second;
}

bool Tracker::residual(const ObjectTrack& track, const MotionVector& v, double& residual, double* along,
                       double* across) const
{
    const UKF& predicted = projected(track, v.t);
    const double dx = v.x - predicted.x_(0);
    const double dy = v.y - predicted.x_(1);
    residual = std::hypot(dx, dy);
    if (along != nullptr && across != nullptr) {
        // components along and across the projected track yaw (ENU)
        const double yaw = predicted.x_(3);
        *along = dx * std::cos(yaw) + dy * std::sin(yaw);
        *across = -dx * std::sin(yaw) + dy * std::cos(yaw);
    }
    return std::isfinite(residual);
}

bool Tracker::insideGate(const ObjectTrack& track, const MotionVector& v, double along, double across) const
{
    if (track.length > 0.0 && track.width > 0.0 && v.length > 0.0 && v.width > 0.0) {
        return std::abs(along) <= 0.5 * (track.length + v.length) && std::abs(across) <= 0.5 * (track.width + v.width);
    }
    return std::hypot(along / m_params.gateAlong, across / m_params.gateAcross) <= 1.0;
}

std::vector<TrackId> Tracker::associate(const std::vector<const MotionVector*>& vectors)
{
    std::vector<TrackId> trackOf(vectors.size(), 0);

    // the objects of one message are distinct (ETSI TS 103 324): they never share a
    // track. heldBy: track -> vector of this message holding it
    std::map<TrackId, std::size_t> heldBy;
    std::vector<std::size_t> unbound;
    for (std::size_t i = 0; i < vectors.size(); ++i) {
        auto b = m_bindings.find(keyOf(*vectors[i]));
        ObjectTrack* track = b != m_bindings.end() ? getObjectTrack(b->second) : nullptr;
        if (track == nullptr) {
            // do not have any associated track for this object.
            // the object id may be retired, switched in the font or
            // it is a new vehicle
            unbound.push_back(i);
            continue;
        }
        // the engine still has the tracking for this author object id
        auto held = heldBy.find(track->id);
        if (held == heldBy.end()) {
            heldBy[track->id] = i;
            trackOf[i] = track->id;
            continue;
        }
        // two bound objects of this message on one track: the closer keeps it, the other is associated again
        double di = 0.0;
        double dh = 0.0;
        residual(*track, *vectors[i], di);
        residual(*track, *vectors[held->second], dh);
        if (di < dh) {
            trackOf[held->second] = 0;
            unbound.push_back(held->second);
            held->second = i;
            trackOf[i] = track->id;
        } else {
            unbound.push_back(i);
        }
    }

    // all objects ids are bounded to a track or there are no tracks yet
    if (unbound.empty() || m_tracks.empty()) {
        return trackOf;
    }

    // a CAM object is the station itself, where the objectId was setted -1
    std::set<TrackId> withCam;
    for (const auto& b : m_bindings) {
        if (b.first.objectId < 0) {
            withCam.insert(b.second);
        }
    }

    // outside the gate: finite but above any sum of gated costs, so feasible matches come first
    const double infeasible = 1e6;

    // Cost matrix, i. e. the residuals between each unbound object and the existing tracks.
    // Paradigm choice: feasible when the two bounding boxes overlap, centre to centre along and across the projected
    // track yaw within half the sum of their lengths and widths: one object, solid bodies do not interpenetrate. No
    // margin for the position error. Without both sizes, the ellipse of gateAlong (maxPosDev) along and gateAcross
    // (half a SUMO lane) across
    std::vector<std::vector<double>> cost(unbound.size(), std::vector<double>(m_tracks.size(), infeasible));
    for (std::size_t r = 0; r < unbound.size(); ++r) {
        const MotionVector& v = *vectors[unbound[r]];
        for (std::size_t c = 0; c < m_tracks.size(); ++c) {
            // a track already held by another object of this message, or a second CAM source
            if (heldBy.count(m_tracks[c].id) || (v.objectId < 0 && withCam.count(m_tracks[c].id))) {
                continue;
            }
            double d = 0.0;
            double along = 0.0;
            double across = 0.0;
            // euclidean distance between tracks projected in delta time and
            // the incoming object
            if (residual(m_tracks[c], v, d, &along, &across) && insideGate(m_tracks[c], v, along, across)) {
                cost[r][c] = d;
            }
        }
    }

    std::vector<int> assignment;

    // Hungarian attribution solve O(n³). It looks for the minimum sum of the costs,
    // i. e. the best association between unbound objects and existing tracks
    HungarianAlgorithm().Solve(cost, assignment);
    for (std::size_t r = 0; r < unbound.size(); ++r) {
        const int c = assignment[r];
        if (c >= 0 && cost[r][c] < infeasible) {
            trackOf[unbound[r]] = m_tracks[c].id;
        }
    }
    return trackOf;
}

// the size of the latest report that carried one: the track's box in the association gate
static void keepSize(ObjectTrack& track, const MotionVector& v)
{
    if (v.length > 0.0 && v.width > 0.0) {
        track.length = v.length;
        track.width = v.width;
    }
}

void Tracker::kalmanUpdate(ObjectTrack& track, const MotionVector& v)
{
    keepSize(track, v);  // every report that passed the checks, also one not fused below (no velocity, out of sequence)
    // the UKF has no position-only measurement, so a report without velocity is not fused
    if (!v.hasVelocity) {
        return;
    }
    // Paradigm choice: a report older than the track state (out of sequence: a CPM object measured before a newer
    // report of the same object was fused) is checked and scored, but not fused. The filter does not run
    // backwards, so fusing it at the state time would pull the track back by v * |dt|
    if (v.t < track.t) {
        return;
    }

    if (!m_params.enableKalman) {
        // the report replaces the state, without the filter
        updateTrackWindow(track, v);
        track.t = v.t;
        track.lastHasVelocity = true;
        return;
    }

    // projects the track to the reported time and fuses the measurement
    track.filter.ProcessMeasurement(laserOf(v, microseconds(v.t)));
    track.t = v.t;
    track.lastHasVelocity = true;
}

/**
 * @brief Creates a new tracks starting the UKF with the first measurement
 */
ObjectTrack& Tracker::initiateTrack(const MotionVector& v)
{
    ObjectTrack track;
    track.id = m_nextId++;  // local ids!
    keepSize(track, v);
    track.t = v.t;

    track.filter.std_jerk_ = m_params.stdJerk;
    track.filter.std_yawdd_ = m_params.stdYawAccel;

    // the UKF initialises from its first measurement
    MeasurementPackage first = laserOf(v, microseconds(v.t));
    const std::array<double, 6>& p0 = v.hasVelocity ? m_params.p0 : m_params.p0NoVelocity;
    track.filter.P0_diag_ = Eigen::Map<const Eigen::VectorXd>(p0.data(), 6);

    if (!v.hasVelocity) {
        // at rest, as the UKF's own RADAR initialisation
        first.raw_measurements_(2) = 0.0;
        first.raw_measurements_(3) = 0.0;
    }

    setPositionNoiseR(track.filter, m_params.measNoise);
    // the position starts from the first vector, so its P0 is the effective position R (measNoise), as the
    // constant P0 is the default R for the other measured states
    track.filter.P0_diag_(0) = track.filter.std_laspx_ * track.filter.std_laspx_;
    track.filter.P0_diag_(1) = track.filter.std_laspy_ * track.filter.std_laspy_;
    track.filter.ProcessMeasurement(first);

    if (!m_params.enableKalman) {
        updateTrackWindow(track, v);  // the report as the track state without the filter
    }

    track.lastHasVelocity = v.hasVelocity;

    m_tracks.push_back(std::move(track));
    return m_tracks.back();
}

std::vector<TrackId> Tracker::retireStaleTracks(double now)
{
    std::vector<TrackId> retired;
    for (const ObjectTrack& t : m_tracks) {
        if (now - t.t > m_params.retireAfter) {
            retired.push_back(t.id);
        }
    }
    if (retired.empty()) {
        return retired;
    }

    auto isRetired = [&retired](TrackId id) {
        return std::find(retired.begin(), retired.end(), id) != retired.end();
    };
    m_tracks.erase(std::remove_if(m_tracks.begin(), m_tracks.end(),
                                  [&](const ObjectTrack& t) { return isRetired(t.id); }),
                   m_tracks.end());
    for (auto it = m_bindings.begin(); it != m_bindings.end();) {
        it = isRetired(it->second) ? m_bindings.erase(it) : std::next(it);
    }
    return retired;
}

TrackId Tracker::getTrackId(const MotionVector& v) const
{
    auto it = m_bindings.find(keyOf(v));
    return it != m_bindings.end() ? it->second : 0;
}

void Tracker::setBind(const MotionVector& v, TrackId trackId)
{
    m_bindings[keyOf(v)] = trackId;
}

ObjectTrack* Tracker::getObjectTrack(TrackId id)
{
    for (ObjectTrack& t : m_tracks) {
        if (t.id == id) {
            return &t;
        }
    }
    return nullptr;
}
