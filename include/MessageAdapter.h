#ifndef MESSAGE_ADAPTER_H_
#define MESSAGE_ADAPTER_H_

// Received CAM / CPM content -> MotionVector in a shared local East-North plane.
// Every vector describes the object's bounding-box centre with its ground velocity there.

#include "Constants.h"
#include "LocalTangentPlane.h"
#include "MotionVector.h"
#include "TransportTheorem.h"

#include <cmath>
#include <cstdint>

// CAM Basic Container + High Frequency Container (ETSI EN 302 637-2)
struct CamReport {
    uint64_t sender = 0;
    double t = 0.0;            // generation time [s]
    double lat = 0.0;          // reference position [deg]
    double lon = 0.0;
    bool hasVelocity = false;
    double speed = 0.0;        // [m/s]
    double headingDeg = 0.0;   // clockwise from true north
    bool hasYawRate = false;
    double yawRateDeg = 0.0;   // [deg/s] positive to the left (CDD YawRate)
    double length = 0.0;       // [m], <= 0 when unavailable
};

// One PerceivedObject of a CPM (ETSI TS 103 324 V2.1.1, clause 7.1.8)
struct CpmObjectReport {
    uint64_t sender = 0;
    double senderLat = 0.0;    // Management Container referencePosition [deg]
    double senderLon = 0.0;
    int64_t objectId = -1;
    double t = 0.0;            // referenceTime + measurementDeltaTime [s]
    double x = 0.0;            // position, East [m] from the sender reference point
    double y = 0.0;            // position, North [m]
    bool hasVelocity = false;
    double speed = 0.0;        // velocityMagnitude [m/s]
    double directionDeg = 0.0; // velocityDirection, counter-clockwise from East
    int64_t objectStation = -1; // oracle
};

namespace MessageAdapter {

// unit vector of a compass heading (clockwise from north), East-North
inline PlanarVector headingVector(double headingRad)
{
    return {std::sin(headingRad), std::cos(headingRad)};
}

// compass heading of an East-North velocity
inline double headingOf(const PlanarVector& v)
{
    return std::atan2(v.east, v.north);
}

// CAM: reference point is the centre of the vehicle's front side (EN 302 637-2);
// moved to the bounding-box centre so CAM and CPM describe the same point
inline MotionVector fromCam(const CamReport& cam, const LocalTangentPlane& plane)
{
    MotionVector v;
    v.author = cam.sender;
    v.objectId = -1;
    v.origin = Origin::Cam;
    v.t = cam.t;
    v.trueStation = static_cast<int64_t>(cam.sender);

    PlanarVector front;
    plane.toLocal(cam.lat, cam.lon, front.east, front.north);

    const double heading = cam.headingDeg * ConstantsWGS84::DEG_TO_RAD;
    const PlanarVector u = headingVector(heading);

    // body-fixed lever arm front -> centre; zero when the length is unknown
    const double halfLength = cam.length > 0.0 ? 0.5 * cam.length : 0.0;
    const PlanarVector r{-halfLength * u.east, -halfLength * u.north};

    // transport theorem, rigid body: p_c = p_front + r
    const PlanarVector centre = transportPosition(front, r);
    v.x = centre.east;
    v.y = centre.north;

    if (cam.hasVelocity) {
        // CDD yaw rate is positive to the left, i.e. counter-clockwise about Up
        const double omega = cam.hasYawRate ? cam.yawRateDeg * ConstantsWGS84::DEG_TO_RAD : 0.0;
        const PlanarVector vFront{cam.speed * u.east, cam.speed * u.north};

        // transport theorem, rigid body: v_c = v_front + omega x r (no motion relative to the body)
        const PlanarVector vCentre = transportVelocity(vFront, PlanarVector{}, omega, r);

        v.hasVelocity = true;
        v.speed = std::hypot(vCentre.east, vCentre.north);
        v.heading = v.speed > 0.0 ? headingOf(vCentre) : heading;
    }
    return v;
}

// CPM perceived object -> local plane
inline MotionVector fromCpmObject(const CpmObjectReport& obj, const LocalTangentPlane& plane)
{
    MotionVector v;
    v.author = obj.sender;
    v.objectId = obj.objectId;
    v.origin = Origin::Cpm;
    v.t = obj.t;
    v.trueStation = obj.objectStation;

    PlanarVector sender;
    plane.toLocal(obj.senderLat, obj.senderLon, sender.east, sender.north);
    const PlanarVector r{obj.x, obj.y};

    // transport theorem, frame of the sender: p = p_O + r
    // TS 103 324 7.1.8.1: East-North-Up frame centred at the sender reference point
    const PlanarVector p = transportPosition(sender, r);
    v.x = p.east;
    v.y = p.north;

    if (obj.hasVelocity) {
        const double direction = obj.directionDeg * ConstantsWGS84::DEG_TO_RAD;
        const PlanarVector vReported{obj.speed * std::cos(direction), obj.speed * std::sin(direction)};

        // transport theorem, frame of the sender: v = v_O + v_rel + omega x r
        // TS 103 324 7.1.8.3: velocity is relative to ground -> v_O = 0
        // TS 103 324 7.1.8.1: East-North-Up axes do not rotate -> omega = 0
        const PlanarVector vFrame{};
        const double omegaFrame = 0.0;
        const PlanarVector vGround = transportVelocity(vFrame, vReported, omegaFrame, r);

        v.hasVelocity = true;
        v.speed = std::hypot(vGround.east, vGround.north);
        v.heading = headingOf(vGround);
    }
    return v;
}

}  // namespace MessageAdapter

#endif  // MESSAGE_ADAPTER_H_
