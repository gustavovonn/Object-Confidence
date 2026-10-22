#ifndef TRACKER_H_
#define TRACKER_H_

#include "EngineParams.h"
#include "MotionTypes.h"
#include "MotionVector.h"
#include "ObjectTrack.h"

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
class Tracker
{
public:
    explicit Tracker(const EngineParams& params);

    // Track moved to time t; the track itself is not modified
    void project(const ObjectTrack& track, double t, UKF& out) const;

    // Track projected to v's time and v's position residual to it [m]; false when not finite
    bool residual(const ObjectTrack& track, const MotionVector& v, double& residual) const;

    // Track id per vector of one message, 0 when none
    std::vector<uint64_t> associate(const std::vector<const MotionVector*>& vectors);
    ObjectTrack* find(uint64_t id);
    void kalmanUpdate(ObjectTrack& track, const MotionVector& v);
    ObjectTrack& initiateTrack(const MotionVector& v);
    std::vector<uint64_t> retireStaleTracks(double now);

    const SourceBinding* binding(const MotionVector& v) const;
    void bind(const MotionVector& v, uint64_t trackId);
    void recordReport(const MotionVector& v);

    const std::vector<ObjectTrack>& tracks() const { return m_tracks; }

private:
    // a source of this author already feeds the track
    bool authorBound(uint64_t author, uint64_t trackId) const;

    EngineParams m_params;
    std::vector<ObjectTrack> m_tracks;
    std::map<SourceKey, SourceBinding> m_bindings;
    uint64_t m_nextId = 1;
};

#endif  // TRACKER_H_
