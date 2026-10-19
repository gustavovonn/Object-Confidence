#ifndef TRACKER_H_
#define TRACKER_H_

#include "EngineParams.h"
#include "MotionTypes.h"
#include "MotionVector.h"
#include "ObjectTrack.h"
#include "UnscentedKalmanFilter.h"

#include <cstdint>
#include <map>
#include <vector>

// A source is one (author, objectId) stream; a CAM author is its own object
struct SourceKey {
    uint64_t author = 0;
    int64_t objectId = -1;

    bool operator<(const SourceKey& o) const
    {
        return author != o.author ? author < o.author : objectId < o.objectId;
    }
};

// Track a source is associated with, and its last report (consistency check input)
struct SourceBinding {
    uint64_t trackId = 0;
    KinematicState last;  // Cartesian policy: lon = x, lat = y
};

// Owns the tracks and the source -> track association.
// Association: a bound source keeps its track (unique PDO object ID, Allig et al., VNC 2019);
// an unbound source takes the nearest projected track inside the gate (GNN, Ambrosin et al., ITSC 2019).
class Tracker
{
public:
    struct Candidate {
        ObjectTrack* track = nullptr;
        ukf::Prediction prediction;  // track projected to the vector time
        double residual = -1.0;      // position residual [m]
        bool bound = false;          // reached through an existing binding
    };

    explicit Tracker(const EngineParams& params);

    // Track moved to time t; the track itself is not modified
    bool project(const ObjectTrack& track, double t, ukf::Prediction& out) const;

    Candidate associate(const MotionVector& v);
    void kalmanUpdate(ObjectTrack& track, const ukf::Prediction& prediction, const MotionVector& v);
    ObjectTrack& initiateTrack(const MotionVector& v);
    std::vector<uint64_t> retireStaleTracks(double now);

    const SourceBinding* binding(const MotionVector& v) const;
    void bind(const MotionVector& v, uint64_t trackId);
    void recordReport(const MotionVector& v);

    const std::vector<ObjectTrack>& tracks() const { return m_tracks; }

private:
    ObjectTrack* find(uint64_t id);

    EngineParams m_params;
    ukf::UnscentedKalmanFilter m_ukf;
    Eigen::MatrixXd m_Q;
    std::vector<ObjectTrack> m_tracks;
    std::map<SourceKey, SourceBinding> m_bindings;
    uint64_t m_nextId = 1;
};

#endif  // TRACKER_H_
