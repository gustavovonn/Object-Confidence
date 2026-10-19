#ifndef SCORER_H_
#define SCORER_H_

#include <cstdint>
#include <unordered_map>

// Per-track confidence: Beta reputation with forgetting (Josang & Ismail, Bled 2002, Eq. 5 and 12).
// A fused report is a hit, a rejected one a miss (Hallyburton & Pajic, CDC 2024, Eq. 5).
class Scorer
{
public:
    explicit Scorer(double forgetting) : m_lambda(forgetting) {}

    double hit(uint64_t trackId) { return add(trackId, 1.0, 0.0); }
    double miss(uint64_t trackId) { return add(trackId, 0.0, 1.0); }

    // Expectation of Beta(r + 1, s + 1); 0.5 without evidence
    double confidenceOf(uint64_t trackId) const
    {
        auto it = m_evidence.find(trackId);
        return it != m_evidence.end() ? expectation(it->second) : 0.5;
    }

    void forget(uint64_t trackId) { m_evidence.erase(trackId); }

private:
    struct Evidence {
        double r = 0.0;  // positive
        double s = 0.0;  // negative
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

#endif  // SCORER_H_
