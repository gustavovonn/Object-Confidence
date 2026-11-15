// Calibrates the track UKF (libs/ukf, CTRA) on simulator ground truth, not on receptions: with
// receptions only the innovation covariance H P Hᵀ + R is observable, so process and measurement noise
// cannot be told apart.
//   Q   from the prediction error of exact truth states: x = truth(t), P ≈ 0, UKF::Predict(Δ), compared
//       with truth(t + Δ). The CTRA chains are linear and decoupled, (v, a) driven by the jerk noise
//       only and (yaw, yaw rate) by the yaw-acceleration noise only, so the variance Predict adds to v
//       is exactly σ_jerk² c_v(Δ) and to yaw σ_yawdd² c_yaw(Δ), sub-stepping included; the maximum
//       likelihood estimate is σ² = mean(r² / c) over Δ weighted as the intervals between reports.
//   P0  second moments of the truth yaw rate and acceleration at report times (the UKF starts both at
//       0); the measured states start from the first vector, so their P0 is its measurement noise R.
//   NEES of the UKF run over the report times of every station with measurements = truth + N(0, R),
//       R known exactly, before and after calibration; Q and P0 are fitted on every other station
//       (sorted by id) and validated on the rest.
// Input: scripts/flatten_truth.py <ground truth> <dir> -> <dir>/truth.csv, <dir>/report_times.csv
// Usage: calibrate_ukf <dir> <report.md> [--runs N] [--seed S]

#include "Angles.h"
#include "LocalTangentPlane.h"
#include "MessageAdapter.h"
#include "ukf.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double POSE_INTERVAL = 0.1;     // [s] vehicle_pose poseInterval (= SUMO step)
constexpr int MAX_STEPS = 10;             // Δ up to 1.0 s: CPM objects are included at least every 1 s
constexpr int MARGIN = 5;                 // poses skipped at each end of a vehicle (0.5 s): insertion, removal
constexpr double MAX_YAW_RATE = 1.5;      // [rad/s] above: heading jump of a SUMO junction internal lane
constexpr double MAX_SPEED_MISMATCH = 2.0;  // [m/s] displacement / dt against the speed: teleport
constexpr double REF_STD = 0.1;           // reference σ for c(Δ); the chains are linear, so any value
constexpr double P_FLOOR = 1e-9;          // "exact" start state; P = 0 would break the Cholesky
constexpr double CHI2_95[7] = {0, 3.841, 5.991, 7.815, 9.488, 11.070, 12.592};

// ================================================================================================
// Types
// ================================================================================================

// Simulator truth at one pose, at the bounding-box centre in the local ENU plane, UKF conventions
struct Pose {
    double t = 0.0;
    double x = 0.0;
    double y = 0.0;
    double v = 0.0;
    double yaw = 0.0;   // counter-clockwise from east
    double yawd = 0.0;  // [rad/s], counter-clockwise positive
    double a = 0.0;
};

struct Station {
    int64_t id = 0;
    std::vector<Pose> poses;
    std::vector<bool> contiguous;  // step k -> k+1: one pose interval, no teleport
    std::vector<bool> smooth;      // ... and no junction heading jump
    std::vector<double> reports;   // report times, any source, unique
};

using Stations = std::map<int64_t, Station>;
using StationSet = std::vector<const Station*>;
using IntervalShares = std::array<double, MAX_STEPS + 1>;  // index: Δ in pose steps

struct Options {
    std::string dir;
    std::string reportPath;
    int runs = 20;
    unsigned seed = 1;
};

// Pose steps dropped by the cleaning, for the report
struct StepCounts {
    std::size_t steps = 0;
    std::size_t notContiguous = 0;
    std::size_t notSmooth = 0;  // contiguous, but with a junction heading jump
};

// One prediction of exact truth over m pose steps: residual r and unit-σ variance c per channel
struct QSample {
    int m = 0;
    double w = 0.0;
    bool smooth = false;
    double rv = 0, cv = 0, ryaw = 0, cyaw = 0, ra = 0, ca = 0, ryawd = 0, cyawd = 0;
    std::size_t station = 0;  // index into the fit set
    std::size_t k = 0;        // start pose
};

struct Fit {
    double stdJerk = NAN;
    double stdYawdd = NAN;
    double robustJerk = NAN;
    double robustYawdd = NAN;
    double impliedJerkFromA = NAN;
    double impliedYawddFromYawd = NAN;
    std::size_t n = 0;
};

// Normalised variance and coverage of one channel at the fitted σ
struct Channel {
    const char* name;
    double sw = 0, nv = 0, c68 = 0, c95 = 0, c99 = 0;

    void add(double r, double var, double w)
    {
        const double z2 = r * r / var;
        sw += w;
        nv += w * z2;
        c68 += w * (z2 <= 1.0);
        c95 += w * (z2 <= 1.96 * 1.96);
        c99 += w * (z2 <= 2.576 * 2.576);
    }

    void normalise()
    {
        nv /= sw;
        c68 /= sw;
        c95 /= sw;
        c99 /= sw;
    }
};

struct QResult {
    std::vector<QSample> samples;
    Fit adopted;           // smooth steps only
    Fit withHeadingJumps;  // every contiguous step
    std::vector<Channel> channels;
};

struct P0Result {
    std::array<double, 6> p0{};
    std::array<double, 6> p0NoVelocity{};
    std::size_t n = 0;
    std::size_t dropped = 0;  // report times with a junction heading jump
    std::size_t nFirst = 0;   // first report of each station
    double firstYawd2 = NAN;
    double firstA2 = NAN;
};

struct UkfConfig {
    const char* name;
    double stdJerk;
    double stdYawdd;
    std::array<double, 6> p0;
};

// NEES statistics per time since the track start
struct NeesBin {
    const char* name;
    double from, to;
    std::vector<double> post6, post2, prior2;
};

using NeesBins = std::vector<NeesBin>;

struct NeesResult {
    UkfConfig previous;
    UkfConfig calibrated;
    UkfConfig derivative;
    NeesBins previousBins;
    NeesBins calibratedBins;
    NeesBins derivativeBins;
    NeesBins calibratedExactBins;
};

// ================================================================================================
// Small helpers
// ================================================================================================

std::vector<std::string> split(const std::string& line)
{
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        cells.push_back(cell);
    }
    return cells;
}

double mean(const std::vector<double>& v)
{
    double s = 0.0;
    for (double x : v) {
        s += x;
    }
    return v.empty() ? NAN : s / static_cast<double>(v.size());
}

double median(std::vector<double> v)
{
    if (v.empty()) {
        return NAN;
    }
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
}

double weightedMedian(std::vector<std::pair<double, double>> vw)
{
    std::sort(vw.begin(), vw.end());
    double total = 0.0;
    for (const auto& p : vw) {
        total += p.second;
    }
    double acc = 0.0;
    for (const auto& p : vw) {
        acc += p.second;
        if (acc >= 0.5 * total) {
            return p.first;
        }
    }
    return NAN;
}

// share of v at or below bound
double within(const std::vector<double>& v, double bound)
{
    std::size_t n = 0;
    for (double x : v) {
        n += x <= bound;
    }
    return v.empty() ? NAN : static_cast<double>(n) / static_cast<double>(v.size());
}

double nees(const VectorXd& e, const MatrixXd& P)
{
    return e.dot(P.ldlt().solve(e));
}

// measurement noise the UKF assumes (laser): px, py, v, yaw
std::array<double, 4> measurementStd()
{
    const UKF defaults;
    return {defaults.std_laspx_, defaults.std_laspy_, defaults.std_spd_, defaults.std_yaw_};
}

// ================================================================================================
// Input
// ================================================================================================

// truth.csv row: station,time,lat,lon,heading,speed,accel,yawRate,length. The pose is converted as a
// perfect CAM, so with the same front -> centre conversion and conventions as the receptions
Pose poseOf(const std::vector<std::string>& c, const LocalTangentPlane& plane)
{
    CamReport cam;
    cam.sender = static_cast<uint64_t>(std::stoll(c[0]));
    cam.t = std::stod(c[1]);
    cam.lat = std::stod(c[2]);
    cam.lon = std::stod(c[3]);
    cam.hasVelocity = true;
    cam.headingDeg = std::stod(c[4]);
    cam.speed = std::stod(c[5]);
    cam.length = std::stod(c[8]);
    const MotionVector mv = MessageAdapter::fromCam(cam, plane);

    Pose p;
    p.t = mv.t;
    p.x = mv.x;
    p.y = mv.y;
    p.v = mv.speed;
    p.yaw = Angles::wrapPi(M_PI / 2.0 - mv.heading);
    p.yawd = std::stod(c[7]) * M_PI / 180.0;  // positive to the left = counter-clockwise
    p.a = std::stod(c[6]);
    return p;
}

void readPoses(std::ifstream& in, Stations& stations)
{
    std::string line;
    std::getline(in, line);  // header
    std::unique_ptr<LocalTangentPlane> plane;  // origin: first pose
    while (std::getline(in, line)) {
        const auto c = split(line);
        if (c.size() < 9) {
            continue;
        }
        if (!plane) {
            plane = std::make_unique<LocalTangentPlane>(std::stod(c[2]), std::stod(c[3]));
        }
        Station& s = stations[std::stoll(c[0])];
        s.id = std::stoll(c[0]);
        s.poses.push_back(poseOf(c, *plane));
    }
}

void readReports(std::ifstream& in, Stations& stations)
{
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        const auto c = split(line);
        if (c.size() < 2) {
            continue;
        }
        auto it = stations.find(std::stoll(c[0]));
        if (it != stations.end()) {
            it->second.reports.push_back(std::stod(c[1]));
        }
    }
}

// Sorts poses and reports, merges reports less than 1 µs apart (the UKF timestamp resolution)
void sortStation(Station& s)
{
    std::sort(s.poses.begin(), s.poses.end(), [](const Pose& a, const Pose& b) { return a.t < b.t; });
    std::sort(s.reports.begin(), s.reports.end());
    s.reports.erase(std::unique(s.reports.begin(), s.reports.end(), [](double a, double b) { return b - a < 1e-6; }),
                    s.reports.end());
}

// Marks each pose step contiguous (one pose interval, no teleport) and smooth (no junction heading jump)
void flagSteps(Station& s)
{
    const std::size_t n = s.poses.size();
    s.contiguous.assign(n, false);
    s.smooth.assign(n, false);
    for (std::size_t k = 0; k + 1 < n; ++k) {
        const Pose& p = s.poses[k];
        const Pose& q = s.poses[k + 1];
        const double dt = q.t - p.t;
        const double moved = std::hypot(q.x - p.x, q.y - p.y) / dt;
        s.contiguous[k] = std::fabs(dt - POSE_INTERVAL) < 1e-3
                          && std::fabs(moved - 0.5 * (p.v + q.v)) <= MAX_SPEED_MISMATCH;
        s.smooth[k] = s.contiguous[k] && std::fabs(Angles::wrapPi(q.yaw - p.yaw)) / dt <= MAX_YAW_RATE;
    }
}

bool load(const std::string& dir, Stations& stations)
{
    std::ifstream truth(dir + "/truth.csv");
    std::ifstream reports(dir + "/report_times.csv");
    if (!truth || !reports) {
        return false;
    }
    readPoses(truth, stations);
    readReports(reports, stations);
    for (auto& [id, s] : stations) {
        sortStation(s);
        flagSteps(s);
    }
    return true;
}

StepCounts countSteps(const Stations& stations)
{
    StepCounts c;
    for (const auto& [id, s] : stations) {
        for (std::size_t k = 0; k + 1 < s.poses.size(); ++k) {
            ++c.steps;
            c.notContiguous += !s.contiguous[k];
            c.notSmooth += s.contiguous[k] && !s.smooth[k];
        }
    }
    return c;
}

// Hold-out: every other station, sorted by id (the ids themselves are all even)
void splitStations(const Stations& stations, StationSet& fitSet, StationSet& evalSet)
{
    for (const auto& [id, s] : stations) {
        (fitSet.size() == evalSet.size() ? fitSet : evalSet).push_back(&s);
    }
}

// ================================================================================================
// Truth and prediction
// ================================================================================================

VectorXd stateOf(const Pose& p)
{
    VectorXd x(6);
    x << p.x, p.y, p.v, p.yaw, p.yawd, p.a;
    return x;
}

// truth at time t, linear between poses; false outside the station's poses
bool truthAt(const Station& s, double t, Pose& out)
{
    const auto& p = s.poses;
    if (p.empty() || t < p.front().t || t > p.back().t) {
        return false;
    }
    auto it = std::lower_bound(p.begin(), p.end(), t, [](const Pose& a, double v) { return a.t < v; });
    if (it == p.begin()) {
        out = *it;
        return true;
    }
    const Pose& b = *it;
    const Pose& a = *(it - 1);
    const double u = (t - a.t) / (b.t - a.t);
    out.t = t;
    out.x = a.x + u * (b.x - a.x);
    out.y = a.y + u * (b.y - a.y);
    out.v = a.v + u * (b.v - a.v);
    out.yaw = Angles::wrapPi(a.yaw + u * Angles::wrapPi(b.yaw - a.yaw));
    out.yawd = a.yawd + u * (b.yawd - a.yawd);
    out.a = a.a + u * (b.a - a.a);
    return true;
}

// UKF started at the exact state p (P ≈ 0) and predicted over dt with the engine's Predict
UKF predictFrom(const Pose& p, double dt, double stdJerk, double stdYawdd)
{
    UKF f;
    f.std_jerk_ = stdJerk;
    f.std_yawdd_ = stdYawdd;
    f.x_ = stateOf(p);
    f.P_ = MatrixXd::Identity(6, 6) * P_FLOOR;
    f.is_initialized_ = true;
    f.Predict(dt);
    return f;
}

// ================================================================================================
// Process noise Q
// ================================================================================================

// Operational Δ: share of the intervals between consecutive reports of a station, in pose steps;
// beyond: intervals longer than MAX_STEPS
IntervalShares intervalShares(const Stations& stations, std::size_t& beyond)
{
    IntervalShares share{};
    std::size_t total = 0;
    beyond = 0;
    for (const auto& [id, s] : stations) {
        for (std::size_t i = 1; i < s.reports.size(); ++i) {
            const long m = std::lround((s.reports[i] - s.reports[i - 1]) / POSE_INTERVAL);
            if (m < 1) {
                continue;
            }
            ++total;
            if (m > MAX_STEPS) {
                ++beyond;
                continue;
            }
            share[m] += 1.0;
        }
    }
    for (double& x : share) {
        x /= static_cast<double>(total);
    }
    return share;
}

// Prediction from pose k over m steps at the reference σ: residuals against the truth m steps later
// and their variances per unit σ²; false when a variance is not positive
bool makeSample(const Station& s, std::size_t station, std::size_t k, std::size_t m, bool smooth, QSample& q)
{
    const Pose& p = s.poses[k];
    const Pose& e = s.poses[k + m];
    const UKF f = predictFrom(p, e.t - p.t, REF_STD, REF_STD);
    const double unit = REF_STD * REF_STD;
    q.m = static_cast<int>(m);
    q.smooth = smooth;
    q.station = station;
    q.k = k;
    q.rv = e.v - f.x_(2);
    q.cv = f.P_(2, 2) / unit;
    q.ryaw = Angles::wrapPi(e.yaw - f.x_(3));
    q.cyaw = f.P_(3, 3) / unit;
    q.ra = e.a - f.x_(5);
    q.ca = f.P_(5, 5) / unit;
    q.ryawd = e.yawd - f.x_(4);
    q.cyawd = f.P_(4, 4) / unit;
    return q.cv > 0 && q.cyaw > 0 && q.ca > 0 && q.cyawd > 0;
}

// Every prediction over 1..MAX_STEPS contiguous pose steps, away from the ends of a vehicle;
// smoothCount[m]: smooth samples per m, for the weights
std::vector<QSample> collectSamples(const StationSet& fitSet, std::array<std::size_t, MAX_STEPS + 1>& smoothCount)
{
    std::vector<QSample> samples;
    smoothCount.fill(0);
    for (std::size_t si = 0; si < fitSet.size(); ++si) {
        const Station& s = *fitSet[si];
        const std::size_t n = s.poses.size();
        for (std::size_t k = MARGIN; k + MARGIN < n; ++k) {
            bool smooth = true;
            for (std::size_t m = 1; m <= MAX_STEPS && k + m + MARGIN < n; ++m) {
                if (!s.contiguous[k + m - 1]) {
                    break;
                }
                smooth = smooth && s.smooth[k + m - 1];
                QSample q;
                if (makeSample(s, si, k, m, smooth, q)) {
                    samples.push_back(q);
                    smoothCount[m] += smooth;
                }
            }
        }
    }
    return samples;
}

// Each Δ counts as its share of the report intervals, spread over its smooth samples
void weightSamples(std::vector<QSample>& samples, const IntervalShares& share,
                   const std::array<std::size_t, MAX_STEPS + 1>& smoothCount)
{
    for (QSample& q : samples) {
        q.w = smoothCount[q.m] ? share[q.m] / static_cast<double>(smoothCount[q.m]) : 0.0;
    }
}

// closed-form ML, σ² = Σ w r² / c / Σ w; robust: weighted median |r| / √c / 0.6745;
// onlyM > 0: samples of that Δ only
Fit fitQ(const std::vector<QSample>& samples, bool smoothOnly, int onlyM = 0)
{
    double sw = 0, sj = 0, sy = 0, sa = 0, syd = 0;
    std::vector<std::pair<double, double>> zj, zy;
    Fit f;
    for (const QSample& q : samples) {
        if ((smoothOnly && !q.smooth) || (onlyM && q.m != onlyM)) {
            continue;
        }
        sw += q.w;
        sj += q.w * q.rv * q.rv / q.cv;
        sy += q.w * q.ryaw * q.ryaw / q.cyaw;
        sa += q.w * q.ra * q.ra / q.ca;
        syd += q.w * q.ryawd * q.ryawd / q.cyawd;
        zj.emplace_back(std::fabs(q.rv) / std::sqrt(q.cv), q.w);
        zy.emplace_back(std::fabs(q.ryaw) / std::sqrt(q.cyaw), q.w);
        ++f.n;
    }
    if (f.n == 0) {
        return f;
    }
    f.stdJerk = std::sqrt(sj / sw);
    f.stdYawdd = std::sqrt(sy / sw);
    f.impliedJerkFromA = std::sqrt(sa / sw);
    f.impliedYawddFromYawd = std::sqrt(syd / sw);
    f.robustJerk = weightedMedian(zj) / 0.6745;
    f.robustYawdd = weightedMedian(zy) / 0.6745;
    return f;
}

// Position residual and covariance in the frame of the predicted yaw: (along-track, cross-track)
void positionInTrackFrame(const UKF& f, const Pose& truth, Eigen::Vector2d& r, Eigen::Matrix2d& P)
{
    const double c = std::cos(f.x_(3));
    const double s = std::sin(f.x_(3));
    Eigen::Matrix2d rot;
    rot << c, s, -s, c;
    r = rot * Eigen::Vector2d(truth.x - f.x_(0), truth.y - f.x_(1));
    P = rot * f.P_.topLeftCorner(2, 2) * rot.transpose();
}

// Every channel of the smooth samples against the exact Predict covariance at (stdJerk, stdYawdd)
std::vector<Channel> diagnose(const std::vector<QSample>& samples, const StationSet& stations, double stdJerk,
                              double stdYawdd)
{
    std::vector<Channel> ch = {{"v"}, {"yaw"}, {"a"}, {"yaw rate"}, {"along-track (Δ ≥ 0.2 s)"},
                               {"cross-track (Δ ≥ 0.2 s)"}};
    for (const QSample& q : samples) {
        if (!q.smooth) {
            continue;
        }
        const Station& s = *stations[q.station];
        const Pose& p = s.poses[q.k];
        const Pose& e = s.poses[q.k + q.m];
        const UKF f = predictFrom(p, e.t - p.t, stdJerk, stdYawdd);
        const VectorXd& x = f.x_;
        const MatrixXd& P = f.P_;
        ch[0].add(e.v - x(2), P(2, 2), q.w);
        ch[1].add(Angles::wrapPi(e.yaw - x(3)), P(3, 3), q.w);
        ch[2].add(e.a - x(5), P(5, 5), q.w);
        ch[3].add(e.yawd - x(4), P(4, 4), q.w);
        // one step gives the cross-track position no variance (the yaw noise reaches it through later
        // sub-steps only), so position is judged from two steps on
        if (q.m >= 2) {
            Eigen::Vector2d r;
            Eigen::Matrix2d Pp;
            positionInTrackFrame(f, e, r, Pp);
            ch[4].add(r(0), Pp(0, 0), q.w);
            ch[5].add(r(1), Pp(1, 1), q.w);
        }
    }
    for (Channel& c : ch) {
        c.normalise();
    }
    return ch;
}

QResult calibrateQ(const StationSet& fitSet, const IntervalShares& share)
{
    QResult q;
    std::array<std::size_t, MAX_STEPS + 1> smoothCount{};
    q.samples = collectSamples(fitSet, smoothCount);
    weightSamples(q.samples, share, smoothCount);
    q.adopted = fitQ(q.samples, true);
    q.withHeadingJumps = fitQ(q.samples, false);
    q.channels = diagnose(q.samples, fitSet, q.adopted.stdJerk, q.adopted.stdYawdd);
    return q;
}

// ================================================================================================
// Initial covariance P0
// ================================================================================================

// Truth second moments at the report times of the fit stations; the measured states start from the
// first vector, so their P0 is R²; without velocity v and yaw start at 0: E[v²] and a uniform yaw
P0Result calibrateP0(const StationSet& fitSet)
{
    double sYawd2 = 0, sA2 = 0, sV2 = 0, fYawd2 = 0, fA2 = 0;
    P0Result r;
    for (const Station* s : fitSet) {
        bool first = true;
        for (double t : s->reports) {
            Pose p;
            if (!truthAt(*s, t, p)) {
                continue;
            }
            if (std::fabs(p.yawd) > MAX_YAW_RATE) {
                ++r.dropped;
                continue;
            }
            sYawd2 += p.yawd * p.yawd;
            sA2 += p.a * p.a;
            sV2 += p.v * p.v;
            ++r.n;
            if (first) {
                fYawd2 += p.yawd * p.yawd;
                fA2 += p.a * p.a;
                ++r.nFirst;
                first = false;
            }
        }
    }
    const auto R = measurementStd();
    const double yawd2 = sYawd2 / r.n;
    const double a2 = sA2 / r.n;
    r.p0 = {R[0] * R[0], R[1] * R[1], R[2] * R[2], R[3] * R[3], yawd2, a2};
    r.p0NoVelocity = {R[0] * R[0], R[1] * R[1], sV2 / r.n, M_PI * M_PI / 3.0, yawd2, a2};
    r.firstYawd2 = fYawd2 / r.nFirst;
    r.firstA2 = fA2 / r.nFirst;
    return r;
}

// ================================================================================================
// NEES validation
// ================================================================================================

NeesBins emptyBins()
{
    return {{"0-0.5 s", 0.0, 0.5, {}, {}, {}},
            {"0.5-1 s", 0.5, 1.0, {}, {}, {}},
            {"1-2 s", 1.0, 2.0, {}, {}, {}},
            {"2-5 s", 2.0, 5.0, {}, {}, {}},
            {">5 s", 5.0, 1e9, {}, {}, {}}};
}

NeesBin* binOf(NeesBins& bins, double age)
{
    NeesBin* bin = nullptr;
    for (NeesBin& b : bins) {
        if (age >= b.from && age < b.to) {
            bin = &b;
        }
    }
    return bin;
}

// Source of the synthetic measurement noise. One distribution for a whole NEES run: the standard
// normal distribution draws in pairs and keeps the second value for the next call
struct Noise {
    std::mt19937_64 rng;
    std::normal_distribution<double> gauss{0.0, 1.0};

    explicit Noise(unsigned seed) : rng(seed) {}
    double operator()() { return gauss(rng); }
};

// Laser measurement of the truth at time t with N(0, (scale R)²) noise
MeasurementPackage noisyMeasurement(const Pose& truth, double t, double scale, Noise& noise)
{
    static const auto R = measurementStd();
    MeasurementPackage m;
    m.sensor_type_ = MeasurementPackage::LASER;
    m.timestamp_ = std::llround(t * 1e6);
    m.raw_measurements_ = VectorXd(4);
    const double px = truth.x + scale * R[0] * noise();
    const double py = truth.y + scale * R[1] * noise();
    const double v = truth.v + scale * R[2] * noise();
    const double yaw = Angles::wrapPi(truth.yaw + scale * R[3] * noise());
    m.raw_measurements_ << px, py, v, yaw;
    return m;
}

// Position NEES of the projection to the measurement time: what the checks and the association gate use
double priorPositionNees(const UKF& f, const MeasurementPackage& m, const Pose& truth)
{
    UKF prior = f;
    prior.Predict((m.timestamp_ - f.time_us_) / 1e6);
    const Eigen::Vector2d e(truth.x - prior.x_(0), truth.y - prior.x_(1));
    return nees(e, prior.P_.topLeftCorner(2, 2));
}

// One station: the UKF from its first report time over the next ones, NEES per update into bins
void neesOfStation(const Station& s, const UkfConfig& cfg, double noiseScale, Noise& noise, NeesBins& bins)
{
    UKF f;
    f.std_jerk_ = cfg.stdJerk;
    f.std_yawdd_ = cfg.stdYawdd;
    f.P0_diag_ = Eigen::Map<const VectorXd>(cfg.p0.data(), 6);
    double start = NAN;
    for (double t : s.reports) {
        Pose truth;
        if (!truthAt(s, t, truth)) {
            continue;
        }
        const MeasurementPackage m = noisyMeasurement(truth, t, noiseScale, noise);
        if (!f.is_initialized_) {
            f.ProcessMeasurement(m);
            start = t;
            continue;
        }
        const double n2prior = priorPositionNees(f, m, truth);

        f.ProcessMeasurement(m);
        VectorXd e = stateOf(truth) - f.x_;
        e(3) = Angles::wrapPi(e(3));
        const double n6 = nees(e, f.P_);
        const double n2 = nees(e.head(2), f.P_.topLeftCorner(2, 2));
        if (std::isfinite(n6) && std::isfinite(n2) && std::isfinite(n2prior)) {
            NeesBin* bin = binOf(bins, t - start);
            bin->post6.push_back(n6);
            bin->post2.push_back(n2);
            bin->prior2.push_back(n2prior);
        }
    }
}

NeesBins runNees(const StationSet& stations, const UkfConfig& cfg, double noiseScale, int runs, unsigned seed)
{
    NeesBins bins = emptyBins();
    Noise noise(seed);
    for (int run = 0; run < runs; ++run) {
        for (const Station* s : stations) {
            neesOfStation(*s, cfg, noiseScale, noise, bins);
        }
    }
    return bins;
}

// The previous values, the calibrated ones, a sensitivity variant and the noise-free case
NeesResult validate(const StationSet& evalSet, const QResult& q, const P0Result& p0, const Options& opt)
{
    NeesResult r{
        // the values before this calibration (Udacity CTRV defaults, P0 = I)
        {"previous", 0.6, 0.401, {1, 1, 1, 1, 1, 1}},
        {"calibrated", q.adopted.stdJerk, q.adopted.stdYawdd, p0.p0},
        // sensitivity: σ matched on a and yaw rate instead of the measured v and yaw
        {"derivative", q.adopted.impliedJerkFromA, q.adopted.impliedYawddFromYawd, p0.p0},
        {}, {}, {}, {}};
    r.previousBins = runNees(evalSet, r.previous, 1.0, opt.runs, opt.seed);
    r.calibratedBins = runNees(evalSet, r.calibrated, 1.0, opt.runs, opt.seed);
    r.derivativeBins = runNees(evalSet, r.derivative, 1.0, opt.runs, opt.seed);
    r.calibratedExactBins = runNees(evalSet, r.calibrated, 0.0, 1, opt.seed);
    return r;
}

// ================================================================================================
// Report
// ================================================================================================

void writeIntervalTable(std::FILE* md, const IntervalShares& share)
{
    std::fprintf(md, "| Δ [s] |");
    for (int m = 1; m <= MAX_STEPS; ++m) {
        std::fprintf(md, " %.1f |", m * POSE_INTERVAL);
    }
    std::fprintf(md, "\n|---|");
    for (int m = 1; m <= MAX_STEPS; ++m) {
        std::fprintf(md, "---|");
    }
    std::fprintf(md, "\n| share |");
    for (int m = 1; m <= MAX_STEPS; ++m) {
        std::fprintf(md, " %.3f |", share[m]);
    }
}

void writeData(std::FILE* md, const StepCounts& steps, const IntervalShares& share, std::size_t beyond)
{
    std::fprintf(md, "\n## Data\n\n%zu pose steps; %zu not contiguous (gap or teleport > %.1f m/s), %zu more with a "
                     "junction heading jump (> %.1f rad/s); %d poses skipped at each end of a vehicle.\n",
                 steps.steps, steps.notContiguous, MAX_SPEED_MISMATCH, steps.notSmooth, MAX_YAW_RATE, MARGIN);
    std::fprintf(md, "\nOperational Δ (intervals between reports of a station, any source; %zu beyond %.1f s "
                     "left out):\n\n",
                 beyond, MAX_STEPS * POSE_INTERVAL);
    writeIntervalTable(md, share);
}

void writeFitRow(std::FILE* md, const char* name, const Fit& f)
{
    std::fprintf(md, "| %s | %zu | **%.3f** | %.3f | **%.3f** | %.3f | %.3f | %.3f |\n", name, f.n, f.stdJerk,
                 f.robustJerk, f.stdYawdd, f.robustYawdd, f.impliedJerkFromA, f.impliedYawddFromYawd);
}

void writeProcessNoise(std::FILE* md, const QResult& q)
{
    std::fprintf(md, "\n\n## Process noise\n\nσ per UKF `Prediction()` call, valid for the `UKF::Predict` "
                     "sub-stepping (0.05 s pieces above 0.1 s). ML: σ² = mean(r²/c); robust: median |r|/√c / 0.6745.\n\n");
    std::fprintf(md, "| fit | n | σ_jerk [m/s³] (from v) | robust | σ_yawdd [rad/s²] (from yaw) | robust | "
                     "σ_jerk implied by a | σ_yawdd implied by yaw rate |\n|---|---|---|---|---|---|---|---|\n");
    writeFitRow(md, "smooth steps (adopted)", q.adopted);
    writeFitRow(md, "with junction heading jumps", q.withHeadingJumps);

    std::fprintf(md, "\nPer Δ (smooth steps):\n\n| Δ [s] | n | σ_jerk | σ_yawdd |\n|---|---|---|---|\n");
    for (int m = 1; m <= MAX_STEPS; ++m) {
        const Fit f = fitQ(q.samples, true, m);
        std::fprintf(md, "| %.1f | %zu | %.3f | %.3f |\n", m * POSE_INTERVAL, f.n, f.stdJerk, f.stdYawdd);
    }

    std::fprintf(md, "\nConsistency at the adopted σ, exact `Predict` covariance (normalised variance, target 1; "
                     "coverage, Gaussian 68.3 / 95.0 / 99.0 %%):\n\n| channel | NV | ≤1σ | ≤1.96σ | ≤2.58σ |\n"
                     "|---|---|---|---|---|\n");
    for (const Channel& c : q.channels) {
        std::fprintf(md, "| %s | %.3g | %.1f%% | %.1f%% | %.1f%% |\n", c.name, c.nv, 100 * c.c68, 100 * c.c95,
                     100 * c.c99);
    }
}

void writeInitialCovariance(std::FILE* md, const P0Result& r)
{
    const auto& p0 = r.p0;
    const auto& nv = r.p0NoVelocity;
    std::fprintf(md, "\n## Initial covariance\n\nTruth at %zu report times of the fit stations (%zu with yaw rate "
                     "> %.1f rad/s left out); second moments, the UKF starts yaw rate and a at 0. The measured states "
                     "start from the first vector: P0 = R.\n\n",
                 r.n, r.dropped, MAX_YAW_RATE);
    std::fprintf(md, "| | px [m²] | py [m²] | v [(m/s)²] | yaw [rad²] | yaw rate [(rad/s)²] | a [(m/s²)²] |\n"
                     "|---|---|---|---|---|---|---|\n");
    std::fprintf(md, "| P0 | %.4g | %.4g | %.4g | %.4g | **%.4g** | **%.4g** |\n", p0[0], p0[1], p0[2], p0[3], p0[4],
                 p0[5]);
    std::fprintf(md, "| P0 without velocity | %.4g | %.4g | **%.4g** | %.4g (uniform) | %.4g | %.4g |\n", nv[0], nv[1],
                 nv[2], nv[3], nv[4], nv[5]);
    std::fprintf(md, "| first report of each station only (n = %zu) | | | | | %.4g | %.4g |\n", r.nFirst,
                 r.firstYawd2, r.firstA2);
}

void writeNeesTable(std::FILE* md, const char* title, const NeesBins& bins)
{
    std::fprintf(md, "\n%s\n\n", title);
    std::fprintf(md, "| since start | n | NEES 6 mean / median | ≤ χ²₆(95%%) | pos NEES 2 mean / median | ≤ χ²₂(95%%) "
                     "| prior pos NEES 2 mean / median | ≤ χ²₂(95%%) |\n|---|---|---|---|---|---|---|---|\n");
    for (const NeesBin& b : bins) {
        std::fprintf(md, "| %s | %zu | %.2f / %.2f | %.1f%% | %.2f / %.2f | %.1f%% | %.2f / %.2f | %.1f%% |\n", b.name,
                     b.post6.size(), mean(b.post6), median(b.post6), 100 * within(b.post6, CHI2_95[6]),
                     mean(b.post2), median(b.post2), 100 * within(b.post2, CHI2_95[2]), mean(b.prior2),
                     median(b.prior2), 100 * within(b.prior2, CHI2_95[2]));
    }
}

void writeNees(std::FILE* md, const NeesResult& r, int runs)
{
    const auto R = measurementStd();
    std::fprintf(md, "\n## NEES on the held-out stations\n\nUKF over each station's report times from its first, "
                     "measurements = truth + N(0, R), R = (%.2f m, %.2f m, %.2f m/s, %.2f rad); %d runs. Posterior "
                     "after each update; prior = projection to the report time (checks, gate).\n",
                 R[0], R[1], R[2], R[3], runs);
    char title[256];
    std::snprintf(title, sizeof title, "Previous: σ_jerk %.3f, σ_yawdd %.3f, P0 = I", r.previous.stdJerk,
                  r.previous.stdYawdd);
    writeNeesTable(md, title, r.previousBins);
    std::snprintf(title, sizeof title, "Calibrated: σ_jerk %.3f, σ_yawdd %.3f, P0 above", r.calibrated.stdJerk,
                  r.calibrated.stdYawdd);
    writeNeesTable(md, title, r.calibratedBins);
    std::snprintf(title, sizeof title, "Sensitivity, σ matched on a and yaw rate: σ_jerk %.3f, σ_yawdd %.3f, P0 above",
                  r.derivative.stdJerk, r.derivative.stdYawdd);
    writeNeesTable(md, title, r.derivativeBins);
    writeNeesTable(md, "Calibrated, noise-free measurements (as the current experiments; R overstated)",
                   r.calibratedExactBins);
}

bool writeReport(const Options& opt, std::size_t stations, std::size_t fit, std::size_t eval, const StepCounts& steps,
                 const IntervalShares& share, std::size_t beyond, const QResult& q, const P0Result& p0,
                 const NeesResult& nees)
{
    std::FILE* md = std::fopen(opt.reportPath.c_str(), "w");
    if (md == nullptr) {
        return false;
    }
    std::fprintf(md, "# UKF calibration on ground truth\n\nSource: `%s` (tools/calibrate_ukf.cc). "
                     "%zu stations: Q and P0 fitted on %zu (every other by id), NEES on the other %zu.\n",
                 opt.dir.c_str(), stations, fit, eval);
    writeData(md, steps, share, beyond);
    writeProcessNoise(md, q);
    writeInitialCovariance(md, p0);
    writeNees(md, nees, opt.runs);
    std::fclose(md);
    return true;
}

void printSummary(const Options& opt, const QResult& q, const P0Result& r)
{
    const auto& p0 = r.p0;
    const auto& nv = r.p0NoVelocity;
    std::printf("sigma_jerk %.4f  sigma_yawdd %.4f  (n %zu)\n", q.adopted.stdJerk, q.adopted.stdYawdd, q.adopted.n);
    std::printf("P0 {%.4g, %.4g, %.4g, %.4g, %.4g, %.4g}\n", p0[0], p0[1], p0[2], p0[3], p0[4], p0[5]);
    std::printf("P0 no velocity {%.4g, %.4g, %.4g, %.4g, %.4g, %.4g}\n", nv[0], nv[1], nv[2], nv[3], nv[4], nv[5]);
    std::printf("report: %s\n", opt.reportPath.c_str());
}

// ================================================================================================
// Command line
// ================================================================================================

int usage(const char* program)
{
    std::fprintf(stderr, "usage: %s <dir with truth.csv, report_times.csv> <report.md> [--runs N] [--seed S]\n",
                 program);
    return EXIT_FAILURE;
}

bool parseOptions(int argc, char** argv, Options& opt)
{
    if (argc < 3) {
        return false;
    }
    opt.dir = argv[1];
    opt.reportPath = argv[2];
    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--runs" && i + 1 < argc) {
            opt.runs = std::atoi(argv[++i]);
        } else if (arg == "--seed" && i + 1 < argc) {
            opt.seed = static_cast<unsigned>(std::strtoul(argv[++i], nullptr, 10));
        } else {
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    Options opt;
    if (!parseOptions(argc, argv, opt)) {
        return usage(argv[0]);
    }

    Stations stations;
    if (!load(opt.dir, stations)) {
        std::fprintf(stderr, "cannot read %s/truth.csv or report_times.csv\n", opt.dir.c_str());
        return EXIT_FAILURE;
    }
    StationSet fitSet, evalSet;
    splitStations(stations, fitSet, evalSet);

    std::size_t beyond = 0;
    const IntervalShares share = intervalShares(stations, beyond);

    const QResult q = calibrateQ(fitSet, share);
    const P0Result p0 = calibrateP0(fitSet);
    const NeesResult nees = validate(evalSet, q, p0, opt);

    if (!writeReport(opt, stations.size(), fitSet.size(), evalSet.size(), countSteps(stations), share, beyond, q, p0,
                     nees)) {
        std::fprintf(stderr, "cannot open %s\n", opt.reportPath.c_str());
        return EXIT_FAILURE;
    }
    printSummary(opt, q, p0);
    return EXIT_SUCCESS;
}
