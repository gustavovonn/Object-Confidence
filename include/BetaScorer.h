#ifndef BETA_SCORER_H_
#define BETA_SCORER_H_

#include "IScorer.h"

#include <cstdint>
#include <unordered_map>

// Per-track confidence via Beta reputation with exponential forgetting.
// (Josang & Ismail, Bled 2002, Eq. 5 and 12; Hallyburton & Pajic, CDC 2024, Eq. 5).
//
// A fused report is a hit (+1 positive evidence), a rejected one a miss
// (+1 negative evidence). Older evidence is discounted by lambda each step.
//
// Confidence = E[Beta(r+1, s+1)] = (r+1) / (r+s+2)
//   → 0.5 with no evidence (uniform prior)
//
// The position residual is accepted but ignored; this scorer is purely
// event-based (hit / miss counts).
class BetaScorer : public IScorer
{
public:
    explicit BetaScorer(double forgetting) : m_lambda(forgetting) {}

    double hit(uint64_t trackId, double /*residual*/) override { return add(trackId, 1.0, 0.0); }
    double miss(uint64_t trackId, double /*residual*/) override { return add(trackId, 0.0, 1.0); }

    double confidenceOf(uint64_t trackId) const override
    {
        auto it = m_evidence.find(trackId);
        return it != m_evidence.end() ? expectation(it->second) : 0.5;
    }

    void forget(uint64_t trackId) override { m_evidence.erase(trackId); }

private:
    struct Evidence {
        double r = 0.0;  // positive (fused)
        double s = 0.0;  // negative (rejected)
    };

    static double expectation(const Evidence& e) { return (e.r + 1.0) / (e.r + e.s + 2.0); }

    double add(uint64_t trackId, double r, double s)
    {
        Evidence& e = m_evidence[trackId];
        e.r = m_lambda * e.r + r;
        e.s = m_lambda * e.s + s;
        return expectation(e);
    }

    double m_lambda;
    std::unordered_map<uint64_t, Evidence> m_evidence;
};

#endif  // BETA_SCORER_H_