#ifndef GAUSSIAN_NIS_SCORER_H_
#define GAUSSIAN_NIS_SCORER_H_

#include "IScorer.h"

#include <cmath>
#include <cstdint>
#include <unordered_map>

// Per-track confidence via a Gaussian Observation Likelihood Factor.
//
// Theory
// ------
// The UKF's position innovation ν = [Δx, Δy] is assumed to be zero-mean
// Gaussian with innovation covariance S ≈ σ²·I₂ (isotropic, calibrated).
// The Normalised Innovation Squared (NIS) is:
//
//   NIS_k = ‖ν_k‖² / σ²  ~  χ²(2)  when the filter is consistent
//
// We receive the scalar position residual r = ‖ν_k‖ (Euclidean [m]) from the
// engine, so NIS_k = r² / σ².
//
// The log-likelihood of one observation under this model is:
//
//   ℓ_k = −½ · (NIS_k + ln(2π·σ²))
//        = −½ · (r²/σ² + ln(2π·σ²))
//
// We accumulate with exponential forgetting:
//
//   L_k = λ·L_{k-1} + ℓ_k
//
// and convert to a [0, 1] confidence via the logistic function:
//
//   confidence = σ(scale · L_k + shift)
//              = 1 / (1 + exp(−(scale·L_k + shift)))
//
// where `scale` and `shift` are tuned so that L = 0 (prior) maps to 0.5
// and the function saturates at reasonable evidence counts.
//
// Parameters
// ----------
// innovationSigma  σ  [m] — assumed s.d. of position innovation; should be
//                           calibrated to match the UKF's actual NIS distribution
//                           on fault-free data (~2–5 m for vehicular datasets)
// forgetting       λ       — same role as BetaScorer's lambda
// logitScale            — steepness of the logistic; default 1.0
// logitShift            — offset; default 0.0 (L=0 → confidence=0.5)
//
// miss() is called for rejected vectors (residual > gate or consistency fail).
// Their residual is typically large (often the gate boundary distance or the
// actual outlier distance). We still accumulate the likelihood — a large
// residual naturally produces a very negative ℓ_k and drives confidence down.
// If residual < 0 (unknown), we treat it as a miss with NIS = +∞, clamped to
// a minimum log-likelihood floor.
class GaussianNISScorer : public IScorer
{
public:
    struct Params {
        double innovationSigma = 3.0;   // σ [m], position innovation s.d.
        double forgetting = 0.9;        // λ ∈ (0, 1]
        double logitScale = 0.5;        // steepness of sigmoid
        double logitShift = 0.0;        // L = 0 → confidence = 0.5
    };

    explicit GaussianNISScorer(Params p) : m_p(p)
    {
        // Pre-compute the constant part of the log-likelihood:
        //   c = −½ · ln(2π·σ²)
        m_logConst = -0.5 * std::log(2.0 * M_PI * p.innovationSigma * p.innovationSigma);
        m_sigma2 = p.innovationSigma * p.innovationSigma;
    }

    double hit(uint64_t trackId, double residual) override  { return accumulate(trackId, residual); }
    double miss(uint64_t trackId, double residual) override { return 0; }

    double confidenceOf(uint64_t trackId) const override
    {
        auto it = m_accum.find(trackId);
        return it != m_accum.end() ? logistic(it->second) : 0.5;
    }

    void forget(uint64_t trackId) override { m_accum.erase(trackId); }

private:
    // Minimum per-step log-likelihood (floor for unknown / very large residuals).
    // Equivalent to NIS = 100 with σ = 1 m.
    static constexpr double LOG_FLOOR = -50.0;

    double logLikelihood(double residual) const
    {
        if (residual < 0.0 || !std::isfinite(residual)) {
            return LOG_FLOOR;
        }
        const double nis = (residual * residual) / m_sigma2;
        const double ll = m_logConst - 0.5 * nis;
        return ll < LOG_FLOOR ? LOG_FLOOR : ll;
    }

    double logistic(double L) const
    {
        return 1.0 / (1.0 + std::exp(-(m_p.logitScale * L + m_p.logitShift)));
    }

    double accumulate(uint64_t trackId, double residual)
    {
        double& L = m_accum[trackId];
        L = m_p.forgetting * L + logLikelihood(residual);
        return logistic(L);
    }

    Params m_p;
    double m_logConst = 0.0;
    double m_sigma2 = 1.0;
    std::unordered_map<uint64_t, double> m_accum;  // accumulated log-likelihood per track
};

#endif  // GAUSSIAN_NIS_SCORER_H_