#ifndef STAGE_TIMER_H_
#define STAGE_TIMER_H_

// Per-stage latency for cost evaluation. Header only; no allocation inside a timed region.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

// steady_clock: monotonic, not adjusted by NTP
using Clock = std::chrono::steady_clock;

inline double usSince(const Clock::time_point& t0)
{
    return std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
}

struct StageStats {
    std::string name;
    std::vector<double> samples;  // [us]

    void add(double us) { samples.push_back(us); }
    std::size_t count() const { return samples.size(); }

    double total() const
    {
        double s = 0.0;
        for (double v : samples) {
            s += v;
        }
        return s;
    }

    double mean() const { return samples.empty() ? 0.0 : total() / static_cast<double>(samples.size()); }

    // nearest rank, q in [0, 1]
    double percentile(double q) const
    {
        if (samples.empty()) {
            return 0.0;
        }
        std::vector<double> v = samples;
        std::sort(v.begin(), v.end());
        const std::size_t i = static_cast<std::size_t>(q * static_cast<double>(v.size() - 1) + 0.5);
        return v[std::min(i, v.size() - 1)];
    }

    double max() const { return samples.empty() ? 0.0 : *std::max_element(samples.begin(), samples.end()); }
};

class StageTimer;

// Scoped timer; no-op when the timer is null
class MaybeStage
{
public:
    MaybeStage(StageTimer* timer, const char* name);
    ~MaybeStage();
    MaybeStage(const MaybeStage&) = delete;
    MaybeStage& operator=(const MaybeStage&) = delete;

private:
    StageStats* m_stats;
    Clock::time_point m_t0;
};

// Named stages in pipeline order
class StageTimer
{
public:
    // deque: references stay valid when later stages are added
    StageStats& stage(const std::string& name)
    {
        for (StageStats& s : m_stages) {
            if (s.name == name) {
                return s;
            }
        }
        m_stages.push_back(StageStats{name, {}});
        m_stages.back().samples.reserve(4096);
        return m_stages.back();
    }

    const std::deque<StageStats>& stages() const { return m_stages; }

    // budgetUs: per-message deadline
    void report(const char* title, double budgetUs) const
    {
        std::printf("\n%s\n", title);
        std::printf("%-22s %8s %10s %10s %10s %10s %9s\n", "stage", "n", "mean us", "p50 us", "p99 us", "max us",
                    "% of whole");
        std::printf("%s\n", std::string(84, '-').c_str());

        const StageStats* whole = nullptr;
        for (const StageStats& s : m_stages) {
            if (s.name == "onMotionVectors") {
                whole = &s;
            }
        }
        const double wholeTotal = whole != nullptr ? whole->total() : 0.0;

        for (const StageStats& s : m_stages) {
            std::printf("%-22s %8zu %10.2f %10.2f %10.2f %10.2f %8.1f%%\n", s.name.c_str(), s.count(), s.mean(),
                        s.percentile(0.50), s.percentile(0.99), s.max(),
                        wholeTotal > 0.0 ? 100.0 * s.total() / wholeTotal : 0.0);
        }

        if (whole != nullptr && whole->count() > 0 && budgetUs > 0.0) {
            const double p99 = whole->percentile(0.99);
            std::printf("\n  budget %.0f us per message; p99 = %.2f us, max = %.2f us, headroom at p99 %.1fx -> %s\n",
                        budgetUs, p99, whole->max(), p99 > 0.0 ? budgetUs / p99 : 0.0,
                        whole->max() < budgetUs ? "fits" : (p99 < budgetUs ? "fits at p99" : "does not fit"));
        }
    }

private:
    std::deque<StageStats> m_stages;
};

inline MaybeStage::MaybeStage(StageTimer* timer, const char* name)
    : m_stats(timer != nullptr ? &timer->stage(name) : nullptr), m_t0(Clock::now())
{
}

inline MaybeStage::~MaybeStage()
{
    if (m_stats != nullptr) {
        m_stats->add(usSince(m_t0));
    }
}

#endif  // STAGE_TIMER_H_
