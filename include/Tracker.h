#ifndef TRACKER_H_
#define TRACKER_H_

#include "EngineParams.h"
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

// Owns the tracks and the source -> track association.
class Tracker
{
public:
    explicit Tracker(const EngineParams& params);

    // Track moved to time t, also back to a time before its state; the track itself is not modified
    void project(const ObjectTrack& track, double t, UKF& out) const;

    // Track projected to v's time and v's position residual to it [m], optionally split along and
    // across the projected yaw; false when not finite
    bool residual(const ObjectTrack& track, const MotionVector& v, double& residual, double* along = nullptr,
                  double* across = nullptr) const;

    // Track id per vector of one message, 0 when none
    std::vector<TrackId> associate(const std::vector<const MotionVector*>& vectors);
    ObjectTrack* getObjectTrack(TrackId id);
    void kalmanUpdate(ObjectTrack& track, const MotionVector& v);
    ObjectTrack& initiateTrack(const MotionVector& v);
    std::vector<TrackId> retireStaleTracks(double now);

    TrackId getTrackId(const MotionVector& v) const;
    void setBind(const MotionVector& v, TrackId trackId);

    const std::vector<ObjectTrack>& tracks() const { return m_tracks; }

private:
    std::vector<ObjectTrack> m_tracks;
    std::map<SourceKey, TrackId> m_bindings;  // source -> the track it is associated with

    EngineParams m_params;
    TrackId m_nextId = 1;
};

#endif  // TRACKER_H_
