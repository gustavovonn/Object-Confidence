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

VectorXd stateOf(const Pose& p)
{
    VectorXd x(6);
    x << p.x, p.y, p.v, p.yaw, p.yawd, p.a;
    return x;
}

bool load(const std::string& dir, std::map<int64_t, Station>& stations)
{
    std::ifstream truth(dir + "/truth.csv");
    std::ifstream reports(dir + "/report_times.csv");
    if (!truth || !reports) {
        return false;
    }
    std::string line;
    std::getline(truth, line);
    std::unique_ptr<LocalTangentPlane> plane;
    while (std::getline(truth, line)) {
        // station,time,lat,lon,heading,speed,accel,yawRate,length
        const auto c = split(line);
        if (c.size() < 9) {
            continue;
        }
        if (!plane) {
            plane = std::make_unique<LocalTangentPlane>(std::stod(c[2]), std::stod(c[3]));
        }
        // a pose is a perfect CAM: same front -> centre conversion as the receptions
        CamReport cam;
        cam.sender = static_cast<uint64_t>(std::stoll(c[0]));
        cam.t = std::stod(c[1]);
        cam.lat = std::stod(c[2]);
        cam.lon = std::stod(c[3]);
        cam.hasVelocity = true;
        cam.headingDeg = std::stod(c[4]);
        cam.speed = std::stod(c[5]);
        cam.length = std::stod(c[8]);
        const MotionVector mv = MessageAdapter::fromCam(cam, *plane);

        Pose p;
        p.t = mv.t;
        p.x = mv.x;
        p.y = mv.y;
        p.v = mv.speed;
        p.yaw = Angles::wrapPi(M_PI / 2.0 - mv.heading);
        p.yawd = std::stod(c[7]) * M_PI / 180.0;  // positive to the left = counter-clockwise
        p.a = std::stod(c[6]);
        Station& s = stations[std::stoll(c[0])];
        s.id = std::stoll(c[0]);
        s.poses.push_back(p);
    }
    std::getline(reports, line);
    while (std::getline(reports, line)) {
        const auto c = split(line);
        if (c.size() < 2) {
            continue;
        }
        auto it = stations.find(std::stoll(c[0]));
        if (it != stations.end()) {
            it->second.reports.push_back(std::stod(c[1]));
        }
    }

    for (auto& [id, s] : stations) {
        std::sort(s.poses.begin(), s.poses.end(), [](const Pose& a, const Pose& b) { return a.t < b.t; });
        std::sort(s.reports.begin(), s.reports.end());
        s.reports.erase(std::unique(s.reports.begin(), s.reports.end(),
                                    [](double a, double b) { return b - a < 1e-6; }),
                        s.reports.end());
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
    return true;
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

// Operational Δ: share of the intervals between consecutive reports of a station, in pose steps
std::array<double, MAX_STEPS + 1> intervalShares(const std::map<int64_t, Station>& stations, std::size_t& beyond)
{
    std::array<double, MAX_STEPS + 1> share{};
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

// One prediction of exact truth over m pose steps
struct QSample {
    int m = 0;
    double w = 0.0;
    bool smooth = false;
    double rv = 0, cv = 0, ryaw = 0, cyaw = 0, ra = 0, ca = 0, ryawd = 0, cyawd = 0;
    std::size_t station = 0;
    std::size_t k = 0;
};

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

struct Fit {
    double stdJerk = NAN;
    double stdYawdd = NAN;
    double robustJerk = NAN;
    double robustYawdd = NAN;
    double impliedJerkFromA = NAN;
    double impliedYawddFromYawd = NAN;
    std::size_t n = 0;
};

// closed-form ML, σ² = Σ w r² / c / Σ w; robust: weighted median |r| / √c / 0.6745
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

// Normalised variance and coverage of each channel at (stdJerk, stdYawdd), exact Predict covariance
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
};

std::vector<Channel> diagnose(const std::vector<QSample>& samples, const std::vector<const Station*>& stations,
                              double stdJerk, double stdYawdd)
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
        // position in the frame of the predicted yaw
        const double c = std::cos(x(3));
        const double sn = std::sin(x(3));
        Eigen::Matrix2d rot;
        rot << c, sn, -sn, c;
        const Eigen::Vector2d r = rot * Eigen::Vector2d(e.x - x(0), e.y - x(1));
        const Eigen::Matrix2d Pp = rot * P.topLeftCorner(2, 2) * rot.transpose();
        // one step gives the cross-track position no variance (the yaw noise reaches it through later
        // sub-steps only), so position is judged from two steps on
        if (q.m >= 2) {
            ch[4].add(r(0), Pp(0, 0), q.w);
            ch[5].add(r(1), Pp(1, 1), q.w);
        }
    }
    for (Channel& c : ch) {
        c.nv /= c.sw;
        c.c68 /= c.sw;
        c.c95 /= c.sw;
        c.c99 /= c.sw;
    }
    return ch;
}

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

std::vector<NeesBin> runNees(const std::vector<const Station*>& stations, const UkfConfig& cfg, double noiseScale,
                             int runs, unsigned seed)
{
    std::vector<NeesBin> bins = {{"0-0.5 s", 0.0, 0.5, {}, {}, {}},
                                 {"0.5-1 s", 0.5, 1.0, {}, {}, {}},
                                 {"1-2 s", 1.0, 2.0, {}, {}, {}},
                                 {"2-5 s", 2.0, 5.0, {}, {}, {}},
                                 {">5 s", 5.0, 1e9, {}, {}, {}}};
    const UKF defaults;
    const double R[4] = {defaults.std_laspx_, defaults.std_laspy_, defaults.std_spd_, defaults.std_yaw_};
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> gauss(0.0, 1.0);

    for (int run = 0; run < runs; ++run) {
        for (const Station* s : stations) {
            UKF f;
            f.std_jerk_ = cfg.stdJerk;
            f.std_yawdd_ = cfg.stdYawdd;
            f.P0_diag_ = Eigen::Map<const VectorXd>(cfg.p0.data(), 6);
            double start = NAN;
            for (double t : s->reports) {
                Pose truth;
                if (!truthAt(*s, t, truth)) {
                    continue;
                }
                MeasurementPackage m;
                m.sensor_type_ = MeasurementPackage::LASER;
                m.timestamp_ = std::llround(t * 1e6);
                m.raw_measurements_ = VectorXd(4);
                m.raw_measurements_ << truth.x + noiseScale * R[0] * gauss(rng),
                    truth.y + noiseScale * R[1] * gauss(rng), truth.v + noiseScale * R[2] * gauss(rng),
                    Angles::wrapPi(truth.yaw + noiseScale * R[3] * gauss(rng));
                if (!f.is_initialized_) {
                    f.ProcessMeasurement(m);
                    start = t;
                    continue;
                }
                NeesBin* bin = nullptr;
                for (NeesBin& b : bins) {
                    if (t - start >= b.from && t - start < b.to) {
                        bin = &b;
                    }
                }
                // prior: the projection the checks and the association gate use
                UKF prior = f;
                prior.Predict((m.timestamp_ - f.time_us_) / 1e6);
                const Eigen::Vector2d ep(truth.x - prior.x_(0), truth.y - prior.x_(1));
                const double n2prior = ep.dot(prior.P_.topLeftCorner(2, 2).ldlt().solve(ep));

                f.ProcessMeasurement(m);
                VectorXd e = stateOf(truth) - f.x_;
                e(3) = Angles::wrapPi(e(3));
                const double n6 = nees(e, f.P_);
                const double n2 = nees(e.head(2), f.P_.topLeftCorner(2, 2));
                if (std::isfinite(n6) && std::isfinite(n2) && std::isfinite(n2prior)) {
                    bin->post6.push_back(n6);
                    bin->post2.push_back(n2);
                    bin->prior2.push_back(n2prior);
                }
            }
        }
    }
    return bins;
}

void writeNees(std::FILE* md, const char* title, const std::vector<NeesBin>& bins)
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

int usage(const char* program)
{
    std::fprintf(stderr, "usage: %s <dir with truth.csv, report_times.csv> <report.md> [--runs N] [--seed S]\n",
                 program);
    return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 3) {
        return usage(argv[0]);
    }
    int runs = 20;
    unsigned seed = 1;
    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--runs" && i + 1 < argc) {
            runs = std::atoi(argv[++i]);
        } else if (arg == "--seed" && i + 1 < argc) {
            seed = static_cast<unsigned>(std::strtoul(argv[++i], nullptr, 10));
        } else {
            return usage(argv[0]);
        }
    }

    std::map<int64_t, Station> all;
    if (!load(argv[1], all)) {
        std::fprintf(stderr, "cannot read %s/truth.csv or report_times.csv\n", argv[1]);
        return EXIT_FAILURE;
    }
    // hold-out: every other station (sorted by id; the ids themselves are all even)
    std::vector<const Station*> fitSet, evalSet;
    std::size_t steps = 0, notContiguous = 0, notSmooth = 0;
    for (const auto& [id, s] : all) {
        (fitSet.size() == evalSet.size() ? fitSet : evalSet).push_back(&s);
        for (std::size_t k = 0; k + 1 < s.poses.size(); ++k) {
            ++steps;
            notContiguous += !s.contiguous[k];
            notSmooth += s.contiguous[k] && !s.smooth[k];
        }
    }

    std::size_t beyond = 0;
    const auto share = intervalShares(all, beyond);

    // ---- Q --------------------------------------------------------------------------------------
    std::vector<QSample> samples;
    std::array<std::size_t, MAX_STEPS + 1> perM{};
    for (std::size_t si = 0; si < fitSet.size(); ++si) {
        const Station& s = *fitSet[si];
        const std::size_t n = s.poses.size();
        for (std::size_t k = MARGIN; k + MARGIN < n; ++k) {
            bool contiguous = true;
            bool smooth = true;
            for (std::size_t m = 1; m <= MAX_STEPS && k + m + MARGIN < n; ++m) {
                contiguous = contiguous && s.contiguous[k + m - 1];
                smooth = smooth && s.smooth[k + m - 1];
                if (!contiguous) {
                    break;
                }
                const Pose& p = s.poses[k];
                const Pose& e = s.poses[k + m];
                const UKF f = predictFrom(p, e.t - p.t, REF_STD, REF_STD);
                QSample q;
                q.m = static_cast<int>(m);
                q.smooth = smooth;
                q.station = si;
                q.k = k;
                q.rv = e.v - f.x_(2);
                q.cv = f.P_(2, 2) / (REF_STD * REF_STD);
                q.ryaw = Angles::wrapPi(e.yaw - f.x_(3));
                q.cyaw = f.P_(3, 3) / (REF_STD * REF_STD);
                q.ra = e.a - f.x_(5);
                q.ca = f.P_(5, 5) / (REF_STD * REF_STD);
                q.ryawd = e.yawd - f.x_(4);
                q.cyawd = f.P_(4, 4) / (REF_STD * REF_STD);
                if (!(q.cv > 0 && q.cyaw > 0 && q.ca > 0 && q.cyawd > 0)) {
                    continue;
                }
                samples.push_back(q);
                perM[m] += smooth;
            }
        }
    }
    // weight: each Δ counts as its share of the report intervals
    for (QSample& q : samples) {
        q.w = perM[q.m] ? share[q.m] / static_cast<double>(perM[q.m]) : 0.0;
    }
    const Fit fit = fitQ(samples, true);
    const Fit fitAll = fitQ(samples, false);
    const auto channels = diagnose(samples, fitSet, fit.stdJerk, fit.stdYawdd);

    // ---- P0 -------------------------------------------------------------------------------------
    double sYawd2 = 0, sA2 = 0, sV2 = 0;
    std::size_t nP0 = 0, dropped = 0;
    double fYawd2 = 0, fA2 = 0;
    std::size_t nFirst = 0;
    for (const Station* s : fitSet) {
        bool first = true;
        for (double t : s->reports) {
            Pose p;
            if (!truthAt(*s, t, p)) {
                continue;
            }
            if (std::fabs(p.yawd) > MAX_YAW_RATE) {
                ++dropped;
                continue;
            }
            sYawd2 += p.yawd * p.yawd;
            sA2 += p.a * p.a;
            sV2 += p.v * p.v;
            ++nP0;
            if (first) {
                fYawd2 += p.yawd * p.yawd;
                fA2 += p.a * p.a;
                ++nFirst;
                first = false;
            }
        }
    }
    const UKF defaults;
    const double r2[4] = {defaults.std_laspx_ * defaults.std_laspx_, defaults.std_laspy_ * defaults.std_laspy_,
                          defaults.std_spd_ * defaults.std_spd_, defaults.std_yaw_ * defaults.std_yaw_};
    const std::array<double, 6> p0 = {r2[0], r2[1], r2[2], r2[3], sYawd2 / nP0, sA2 / nP0};
    const std::array<double, 6> p0NoVelocity = {r2[0], r2[1], sV2 / nP0, M_PI * M_PI / 3.0, sYawd2 / nP0,
                                                sA2 / nP0};

    // ---- NEES -----------------------------------------------------------------------------------
    // the values before this calibration (Udacity CTRV defaults, P0 = I)
    const UkfConfig previous{"previous", 0.6, 0.401, {1, 1, 1, 1, 1, 1}};
    const UkfConfig calibrated{"calibrated", fit.stdJerk, fit.stdYawdd, p0};
    const auto neesPrev = runNees(evalSet, previous, 1.0, runs, seed);
    const auto neesCal = runNees(evalSet, calibrated, 1.0, runs, seed);
    // sensitivity: σ matched on a and yaw rate instead of the measured v and yaw
    const UkfConfig derivative{"derivative", fit.impliedJerkFromA, fit.impliedYawddFromYawd, p0};
    const auto neesDer = runNees(evalSet, derivative, 1.0, runs, seed);
    const auto neesCalExact = runNees(evalSet, calibrated, 0.0, 1, seed);

    // ---- report ---------------------------------------------------------------------------------
    std::FILE* md = std::fopen(argv[2], "w");
    if (md == nullptr) {
        std::fprintf(stderr, "cannot open %s\n", argv[2]);
        return EXIT_FAILURE;
    }
    std::fprintf(md, "# UKF calibration on ground truth\n\nSource: `%s` (tools/calibrate_ukf.cc). "
                     "%zu stations: Q and P0 fitted on %zu (every other by id), NEES on the other %zu.\n",
                 argv[1], all.size(), fitSet.size(), evalSet.size());
    std::fprintf(md, "\n## Data\n\n%zu pose steps; %zu not contiguous (gap or teleport > %.1f m/s), %zu more with a "
                     "junction heading jump (> %.1f rad/s); %d poses skipped at each end of a vehicle.\n",
                 steps, notContiguous, MAX_SPEED_MISMATCH, notSmooth, MAX_YAW_RATE, MARGIN);
    std::fprintf(md, "\nOperational Δ (intervals between reports of a station, any source; %zu beyond %.1f s "
                     "left out):\n\n| Δ [s] |",
                 beyond, MAX_STEPS * POSE_INTERVAL);
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

    std::fprintf(md, "\n\n## Process noise\n\nσ per UKF `Prediction()` call, valid for the `UKF::Predict` "
                     "sub-stepping (0.05 s pieces above 0.1 s). ML: σ² = mean(r²/c); robust: median |r|/√c / 0.6745.\n\n");
    std::fprintf(md, "| fit | n | σ_jerk [m/s³] (from v) | robust | σ_yawdd [rad/s²] (from yaw) | robust | "
                     "σ_jerk implied by a | σ_yawdd implied by yaw rate |\n|---|---|---|---|---|---|---|---|\n");
    auto row = [&](const char* name, const Fit& f) {
        std::fprintf(md, "| %s | %zu | **%.3f** | %.3f | **%.3f** | %.3f | %.3f | %.3f |\n", name, f.n, f.stdJerk,
                     f.robustJerk, f.stdYawdd, f.robustYawdd, f.impliedJerkFromA, f.impliedYawddFromYawd);
    };
    row("smooth steps (adopted)", fit);
    row("with junction heading jumps", fitAll);
    std::fprintf(md, "\nPer Δ (smooth steps):\n\n| Δ [s] | n | σ_jerk | σ_yawdd |\n|---|---|---|---|\n");
    for (int m = 1; m <= MAX_STEPS; ++m) {
        const Fit f = fitQ(samples, true, m);
        std::fprintf(md, "| %.1f | %zu | %.3f | %.3f |\n", m * POSE_INTERVAL, f.n, f.stdJerk, f.stdYawdd);
    }
    std::fprintf(md, "\nConsistency at the adopted σ, exact `Predict` covariance (normalised variance, target 1; "
                     "coverage, Gaussian 68.3 / 95.0 / 99.0 %%):\n\n| channel | NV | ≤1σ | ≤1.96σ | ≤2.58σ |\n"
                     "|---|---|---|---|---|\n");
    for (const Channel& c : channels) {
        std::fprintf(md, "| %s | %.3g | %.1f%% | %.1f%% | %.1f%% |\n", c.name, c.nv, 100 * c.c68, 100 * c.c95,
                     100 * c.c99);
    }

    std::fprintf(md, "\n## Initial covariance\n\nTruth at %zu report times of the fit stations (%zu with yaw rate "
                     "> %.1f rad/s left out); second moments, the UKF starts yaw rate and a at 0. The measured states "
                     "start from the first vector: P0 = R.\n\n",
                 nP0, dropped, MAX_YAW_RATE);
    std::fprintf(md, "| | px [m²] | py [m²] | v [(m/s)²] | yaw [rad²] | yaw rate [(rad/s)²] | a [(m/s²)²] |\n"
                     "|---|---|---|---|---|---|---|\n");
    std::fprintf(md, "| P0 | %.4g | %.4g | %.4g | %.4g | **%.4g** | **%.4g** |\n", p0[0], p0[1], p0[2], p0[3], p0[4],
                 p0[5]);
    std::fprintf(md, "| P0 without velocity | %.4g | %.4g | **%.4g** | %.4g (uniform) | %.4g | %.4g |\n",
                 p0NoVelocity[0], p0NoVelocity[1], p0NoVelocity[2], p0NoVelocity[3], p0NoVelocity[4],
                 p0NoVelocity[5]);
    std::fprintf(md, "| first report of each station only (n = %zu) | | | | | %.4g | %.4g |\n", nFirst,
                 fYawd2 / nFirst, fA2 / nFirst);

    std::fprintf(md, "\n## NEES on the held-out stations\n\nUKF over each station's report times from its first, "
                     "measurements = truth + N(0, R), R = (%.2f m, %.2f m, %.2f m/s, %.2f rad); %d runs. Posterior "
                     "after each update; prior = projection to the report time (checks, gate).\n",
                 defaults.std_laspx_, defaults.std_laspy_, defaults.std_spd_, defaults.std_yaw_, runs);
    char title[256];
    std::snprintf(title, sizeof title, "Previous: σ_jerk %.3f, σ_yawdd %.3f, P0 = I", previous.stdJerk,
                  previous.stdYawdd);
    writeNees(md, title, neesPrev);
    std::snprintf(title, sizeof title, "Calibrated: σ_jerk %.3f, σ_yawdd %.3f, P0 above", calibrated.stdJerk,
                  calibrated.stdYawdd);
    writeNees(md, title, neesCal);
    std::snprintf(title, sizeof title, "Sensitivity, σ matched on a and yaw rate: σ_jerk %.3f, σ_yawdd %.3f, P0 above",
                  derivative.stdJerk, derivative.stdYawdd);
    writeNees(md, title, neesDer);
    writeNees(md, "Calibrated, noise-free measurements (as the current experiments; R overstated)", neesCalExact);
    std::fclose(md);

    std::printf("sigma_jerk %.4f  sigma_yawdd %.4f  (n %zu)\n", fit.stdJerk, fit.stdYawdd, fit.n);
    std::printf("P0 {%.4g, %.4g, %.4g, %.4g, %.4g, %.4g}\n", p0[0], p0[1], p0[2], p0[3], p0[4], p0[5]);
    std::printf("P0 no velocity {%.4g, %.4g, %.4g, %.4g, %.4g, %.4g}\n", p0NoVelocity[0], p0NoVelocity[1],
                p0NoVelocity[2], p0NoVelocity[3], p0NoVelocity[4], p0NoVelocity[5]);
    std::printf("report: %s\n", argv[2]);
    return EXIT_SUCCESS;
}
