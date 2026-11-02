#ifndef WINDOWED_NIS_SCORER_H_
#define WINDOWED_NIS_SCORER_H_

#include "IScorer.h"

#include <cmath>
#include <cstdint>
#include <deque>
#include <unordered_map>

// Per-track confidence via Windowed Sample-Mean NIS.
//
// Theory
// ------
// Under a consistent UKF with isotropic position innovation covariance σ²·I₂,
// the NIS is chi-squared distributed with 2 degrees of freedom:
//
//   NIS_k = r_k² / σ²  ~  χ²(2)
//
// The chi-squared(2) CDF at threshold τ is 1 − exp(−τ/2), giving:
//   τ = 5.99  →  95 % of inlier observations fall below it   (α = 0.05)
//
// Confidence is estimated as the fraction of the last W NIS samples that
// fall below τ:
//
//   confidence = (# samples < τ) / W       once the window is full
//   confidence = 0.5                        while warming up (< minSamples)
//
// A sample is added on every hit() or miss() call, using the actual residual
// supplied by the engine.  miss() samples typically come from rejected vectors
// whose residuals are large (> gate) or from consistency failures where the
// engine passes the measured residual anyway; both naturally push the NIS
// above τ and lower confidence.
//
// If residual < 0 (unknown), we push +∞ (always above threshold), which is
// conservative: an unknown-residual rejection counts as an inlier failure.
//
// Parameters
// ----------
// innovationSigma  σ [m]  — same calibration as GaussianNISScorer
// windowSize       W      — number of most-recent samples kept (default 10)
// nisThreshold     τ      — chi-squared quantile (default 5.99 for χ²(2), α=0.05)
// minSamples            — window fill before returning a real estimate (default 3)
//
// This scorer fits the architecture well:
//  - non-parametric beyond the chi-squared assumption for τ
//  - forget() clears the deque, so track retirement costs O(W)
//  - windowSize and innovationSigma are independently tunable
class WindowedNISScorer : public IScorer
{
public:
    struct Params {
        double innovationSigma = 3.0;  // σ [m]
        int    windowSize = 10;         // W
        double nisThreshold = 5.991;    // τ: χ²(2) at α = 0.05
        int    minSamples = 3;          // warm-up guard
    };

    explicit WindowedNISScorer(Params p) : m_p(p), m_sigma2(p.innovationSigma * p.innovationSigma) {}

    double hit(uint64_t trackId, double residual) override  { return push(trackId, residual); }
    double miss(uint64_t trackId, double residual) override { return 0; }

    double confidenceOf(uint64_t trackId) const override
    {
        auto it = m_windows.find(trackId);
        if (it == m_windows.end()) {
            return 0.5;
        }
        return estimate(it->second);
    }

    void forget(uint64_t trackId) override { m_windows.erase(trackId); }

private:
    double nisOf(double residual) const
    {
        if (residual < 0.0 || !std::isfinite(residual)) {
            return std::numeric_limits<double>::infinity();
        }
        return (residual * residual) / m_sigma2;
    }

    double estimate(const std::deque<double>& w) const
    {
        if (static_cast<int>(w.size()) < m_p.minSamples) {
            return 0.5;
        }
        int below = 0;
        for (double nis : w) {
            if (nis < m_p.nisThreshold) {
                ++below;
            }
        }
        return static_cast<double>(below) / static_cast<double>(w.size());
    }

    double push(uint64_t trackId, double residual)
    {
        std::deque<double>& w = m_windows[trackId];
        w.push_back(nisOf(residual));
        if (static_cast<int>(w.size()) > m_p.windowSize) {
            w.pop_front();
        }
        return estimate(w);
    }

    Params m_p;
    double m_sigma2;
    std::unordered_map<uint64_t, std::deque<double>> m_windows;
};

#endif  // WINDOWED_NIS_SCORER_H_