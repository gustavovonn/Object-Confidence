#ifndef SCORER_FACTORY_H_
#define SCORER_FACTORY_H_

#include "BetaScorer.h"
#include "EngineParams.h"
#include "GaussianNISScorer.h"
#include "IScorer.h"
#include "WindowedNISScorer.h"

#include <memory>
#include <stdexcept>
#include <string>

// Creates the scorer named by params.scorer.
// This is the only file that includes all concrete scorer headers,
// keeping ObjectConfidenceEngine free of that dependency.
inline std::unique_ptr<IScorer> makeScorer(const EngineParams& params)
{
    switch (params.scorer) {
        case ScorerKind::Beta:
            return std::make_unique<BetaScorer>(params.forgetting);

        case ScorerKind::GaussianNIS: {
            GaussianNISScorer::Params p;
            p.innovationSigma = params.innovationSigma;
            p.forgetting       = params.forgetting;
            p.logitScale       = params.gaussianLogitScale;
            p.logitShift       = params.gaussianLogitShift;
            return std::make_unique<GaussianNISScorer>(p);
        }

        case ScorerKind::WindowedNIS: {
            WindowedNISScorer::Params p;
            p.innovationSigma = params.innovationSigma;
            p.windowSize       = params.nisWindowSize;
            p.nisThreshold     = ConstantsScore::DEFAULT_NIS_THRESHOLD;
            p.minSamples       = params.nisMinSamples;
            return std::make_unique<WindowedNISScorer>(p);
        }
    }
    // Unreachable on well-formed ScorerKind, but satisfies -Wreturn-type
    throw std::invalid_argument("makeScorer: unknown ScorerKind " +
                                std::to_string(static_cast<int>(params.scorer)));
}

// Inverse of the above: the canonical CLI string for each scorer kind.
inline const char* scorerKindName(ScorerKind k)
{
    switch (k) {
        case ScorerKind::Beta:        return "beta";
        case ScorerKind::GaussianNIS: return "gaussian-nis";
        case ScorerKind::WindowedNIS: return "windowed-nis";
    }
    return "unknown";
}

// Parses the CLI string produced by scorerKindName(); throws on unknown input.
inline ScorerKind parseScorerKind(const std::string& s)
{
    if (s == "beta")         return ScorerKind::Beta;
    if (s == "gaussian-nis") return ScorerKind::GaussianNIS;
    if (s == "windowed-nis") return ScorerKind::WindowedNIS;
    throw std::invalid_argument("unknown scorer '" + s +
                                "'; valid: beta | gaussian-nis | windowed-nis");
}

#endif  // SCORER_FACTORY_H_