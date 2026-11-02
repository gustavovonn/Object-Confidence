#ifndef ENGINE_PARAMS_H_
#define ENGINE_PARAMS_H_

#include "MechanismConstants.h"

// Which confidence scorer the engine instantiates.
enum class ScorerKind {
    Beta,         // Beta reputation with forgetting (Josang & Ismail, Bled 2002)
    GaussianNIS,  // Gaussian observation log-likelihood accumulator
    WindowedNIS,  // Windowed sample-mean NIS fraction (default for new experiments)
};

// Runtime configuration; defaults are the values of MechanismConstants.h
struct EngineParams {

    // association and fusion
    double gateAlong = ConstantsAssociation::DEFAULT_GATE_ALONG;      // [m]
    double gateAcross = ConstantsAssociation::DEFAULT_GATE_ACROSS;    // [m]
    double retireAfter = ConstantsAssociation::DEFAULT_RETIRE_AFTER;  // [s]

    // predictive check against the fused track
    double maxPosDev = ConstantsPredictive::DEFAULT_MAX_POS_DEV;  // d_max_0 [m]

    // consistency check between consecutive reports of one source
    double speedToleranceMps = ConstantsConsistency::DEFAULT_SPEED_TOLERANCE_MPS;
    bool   enableConsistency = true;

    // confidence scorer selection
    ScorerKind scorer = ScorerKind::Beta;

    // Beta scorer
    double forgetting = ConstantsScore::DEFAULT_FORGETTING;

    // Gaussian NIS and Windowed NIS: position innovation s.d. [m]
    double innovationSigma = ConstantsScore::DEFAULT_INNOVATION_SIGMA;

    // Windowed NIS
    int  nisWindowSize  = ConstantsScore::DEFAULT_NIS_WINDOW_SIZE;
    int  nisMinSamples  = ConstantsScore::DEFAULT_NIS_MIN_SAMPLES;

    // Gaussian NIS logistic parameters
    double gaussianLogitScale = 0.5;
    double gaussianLogitShift = 0.0;
};

#endif  // ENGINE_PARAMS_H_
