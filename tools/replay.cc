// Replays flattened receptions (scripts/flatten_dataset.py) through one engine per receiver.
// Usage: replay <receptions.csv> <records.csv> [--profile] [--max-msgs N] [--csv-out <file>] [--dataset <name>]
//   --profile   per-stage latency report
//   --max-msgs  stop after N messages
//   --csv-out   append the per-stage latency rows to <file> (header when new); implies --profile
//   --dataset   name in those rows; default: receptions file name without extension

#include "MessageAdapter.h"
#include "ObjectConfidenceEngine.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

enum Col { KIND, RECEIVER, RCV_TIME, MESSAGE_ID, SENDER, T, LAT, LON, HAS_VELOCITY, SPEED, HEADING, YAW_RATE,
           LENGTH, OBJECT_ID, X, Y, DIRECTION, TRUE_STATION, ATTACK_TYPE, NUM_COLS };

struct Row {
    bool cam = false;
    uint64_t receiver = 0;
    double rcvTime = 0.0;
    uint64_t messageId = 0;
    std::vector<double> f;
};

void parse(const std::string& line, Row& row)
{
    std::stringstream ss(line);
    std::string cell;
    row.f.assign(NUM_COLS, NAN);
    for (int c = 0; c < NUM_COLS && std::getline(ss, cell, ','); ++c) {
        if (c == KIND) {
            row.cam = cell == "cam";
        } else {
            row.f[c] = std::strtod(cell.c_str(), nullptr);
        }
    }
    row.receiver = static_cast<uint64_t>(row.f[RECEIVER]);
    row.rcvTime = row.f[RCV_TIME];
    row.messageId = static_cast<uint64_t>(row.f[MESSAGE_ID]);
}

MotionVector toMotionVector(const Row& row, const LocalTangentPlane& plane)
{
    const auto& f = row.f;
    if (row.cam) {
        CamReport cam;
        cam.sender = static_cast<uint64_t>(f[SENDER]);
        cam.t = f[T];
        cam.lat = f[LAT];
        cam.lon = f[LON];
        cam.hasVelocity = f[HAS_VELOCITY] > 0.5;
        cam.speed = f[SPEED];
        cam.headingDeg = f[HEADING];
        cam.hasYawRate = std::isfinite(f[YAW_RATE]);
        cam.yawRateDeg = cam.hasYawRate ? f[YAW_RATE] : 0.0;
        cam.length = std::isfinite(f[LENGTH]) ? f[LENGTH] : 0.0;
        return MessageAdapter::fromCam(cam, plane);
    }
    CpmObjectReport obj;
    obj.sender = static_cast<uint64_t>(f[SENDER]);
    obj.senderLat = f[LAT];
    obj.senderLon = f[LON];
    obj.objectId = static_cast<int64_t>(f[OBJECT_ID]);
    obj.t = f[T];
    obj.x = f[X];
    obj.y = f[Y];
    obj.hasVelocity = f[HAS_VELOCITY] > 0.5;
    obj.speed = f[SPEED];
    obj.directionDeg = f[DIRECTION];
    obj.objectStation = static_cast<int64_t>(f[TRUE_STATION]);
    return MessageAdapter::fromCpmObject(obj, plane);
}

const char* outcomeName(Outcome o)
{
    switch (o) {
        case Outcome::Dropped: return "dropped";
        case Outcome::StartedTrack: return "started";
        case Outcome::Fused: return "fused";
        case Outcome::Rejected: return "rejected";
    }
    return "?";
}

// file name without directory and extension
std::string stem(const std::string& path)
{
    const std::size_t slash = path.find_last_of('/');
    const std::string name = slash == std::string::npos ? path : path.substr(slash + 1);
    const std::size_t dot = name.find_last_of('.');
    return dot == std::string::npos ? name : name.substr(0, dot);
}

int usage(const char* program)
{
    std::fprintf(stderr,
                 "usage: %s <receptions.csv> <records.csv> [--profile] [--max-msgs N] [--csv-out <file>] "
                 "[--dataset <name>]\n",
                 program);
    return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 3) {
        return usage(argv[0]);
    }
    bool profile = false;
    std::size_t maxMsgs = 0;  // 0: all
    std::string csvOut;
    std::string dataset = stem(argv[1]);
    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--profile") {
            profile = true;
        } else if (arg == "--max-msgs" && i + 1 < argc) {
            maxMsgs = std::strtoull(argv[++i], nullptr, 10);
        } else if (arg == "--csv-out" && i + 1 < argc) {
            csvOut = argv[++i];
            profile = true;
        } else if (arg == "--dataset" && i + 1 < argc) {
            dataset = argv[++i];
        } else {
            return usage(argv[0]);
        }
    }

    std::ifstream in(argv[1]);
    std::ofstream out(argv[2]);
    if (!in || !out) {
        std::fprintf(stderr, "cannot open input or output\n");
        return EXIT_FAILURE;
    }
    out << "receiver,rcvTime,t,author,objectId,origin,trueStation,outcome,trackId,residual,consistencyFailed,confidence,attackType\n";

    std::string line;

    // a stage takes at most one sample per row; reserving that keeps reallocation out of the timed regions
    std::size_t rows = 0;
    if (profile) {
        while (std::getline(in, line)) {
            ++rows;
        }
        in.clear();
        in.seekg(0);
    }

    std::getline(in, line);  // header

    std::unique_ptr<LocalTangentPlane> plane;  // origin: first reported position
    std::unique_ptr<ObjectConfidenceEngine> engine;
    StageTimer timer(rows);
    uint64_t currentReceiver = 0;
    std::vector<MotionVector> message;
    std::vector<int> labels;  // attackType per vector, passed through
    std::size_t messages = 0;
    std::size_t vectors = 0;

    auto flush = [&](uint64_t receiver, double now) {
        if (message.empty()) {
            return;
        }
        const std::vector<VectorRecord> records = engine->onMotionVectors(message, receiver, now);
        for (std::size_t i = 0; i < records.size(); ++i) {
            const VectorRecord& r = records[i];
            out << r.receiver << ',' << now << ',' << r.t << ',' << r.author << ',' << r.objectId << ','
                << (r.origin == Origin::Cam ? "cam" : "cpm") << ',' << r.trueStation << ',' << outcomeName(r.outcome)
                << ',' << r.trackId << ',' << r.residual << ',' << r.consistencyFailed << ',' << r.confidence << ','
                << labels[i] << '\n';
        }
        ++messages;
        vectors += message.size();
        message.clear();
        labels.clear();
    };

    Row row;
    uint64_t lastMessage = 0;
    double lastRcv = 0.0;
    while (std::getline(in, line)) {
        parse(line, row);
        if (!plane) {
            plane = std::make_unique<LocalTangentPlane>(row.f[LAT], row.f[LON]);
        }
        if (!engine || row.receiver != currentReceiver) {
            flush(currentReceiver, lastRcv);
            currentReceiver = row.receiver;
            engine = std::make_unique<ObjectConfidenceEngine>();
            if (profile) {
                engine->setStageTimer(&timer);
            }
        } else if (row.messageId != lastMessage) {
            flush(currentReceiver, lastRcv);
        }
        // a flush just emptied the message, so nothing is left half processed
        if (maxMsgs > 0 && messages >= maxMsgs) {
            break;
        }
        message.push_back(toMotionVector(row, *plane));
        labels.push_back(std::isfinite(row.f[ATTACK_TYPE]) ? static_cast<int>(row.f[ATTACK_TYPE]) : 0);
        lastMessage = row.messageId;
        lastRcv = row.rcvTime;
    }
    flush(currentReceiver, lastRcv);

    std::printf("%zu messages, %zu motion vectors\n", messages, vectors);
    if (profile) {
        timer.report("per-stage latency", 100000.0);
    }
    if (!csvOut.empty()) {
        std::FILE* csv = std::fopen(csvOut.c_str(), "a");
        if (csv == nullptr) {
            std::fprintf(stderr, "cannot open %s\n", csvOut.c_str());
            return EXIT_FAILURE;
        }
        std::fseek(csv, 0, SEEK_END);
        if (std::ftell(csv) == 0) {
            std::fprintf(csv, "%s\n", StageTimer::csvHeader());
        }
        timer.writeCsv(csv, dataset.c_str());
        std::fclose(csv);
    }
    return EXIT_SUCCESS;
}
