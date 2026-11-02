#ifndef ISCORER_H_
#define ISCORER_H_

#include <cstdint>

// Abstract scorer interface.
// A scorer maintains a per-track confidence estimate in [0, 1].
// hit()  — called when a vector passes all checks and is fused into the track.
// miss() — called when a vector is rejected (failed gate or consistency).
// Both receive the Euclidean position residual [m] so implementations that
// are NIS-based can use it; residual is negative when unknown (e.g. no track
// existed before this call — currently unused path, but keeps the interface
// future-proof).
class IScorer
{
public:
    virtual ~IScorer() = default;

    // Returns the updated confidence estimate for trackId.
    virtual double hit(uint64_t trackId, double residual) = 0;
    virtual double miss(uint64_t trackId, double residual) = 0;

    // Confidence without changing state; returns 0.5 when no evidence exists.
    virtual double confidenceOf(uint64_t trackId) const = 0;

    // Called when a track is retired; frees per-track state.
    virtual void forget(uint64_t trackId) = 0;
};

#endif  // ISCORER_H_