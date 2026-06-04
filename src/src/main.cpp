/*
 * Note: Claude (Anthropic) was used under human supervision for two purposes here:
 * documenting non-obvious decisions, and assisting with debugging HTTP request
 * handling and JSON serialization/deserialization. All other implementation is
 * the author's own work unless stated otherwise.
 *
 * This file is large because it consolidates almost all HTTP route definitions, request
 * parsing, and simulation control in one place. The author is aware that splitting
 * it (e.g. by feature area) would improve structure, but the refactor would require
 * a disproportionate amount of time relative to the thesis scope.
 */
#include "../include/SimulationEngine.hpp"
#include "../include/SimulationTypes.hpp"
#include "../include/MetricsCollector.hpp"
#include "../include/AgentControllers.hpp"
#include "../include/SimConfig.hpp"
#include "../include/Optimizer.hpp"

#include <httplib.h>
#include <nlohmann/json.hpp>

#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocketServer.h>

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include <memory>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstring>
#include <set>
#include <unordered_map>
#include <deque>

using json = nlohmann::json;
using namespace httplib;

// -------------------- Globals --------------------
SimulationEngine engine;
std::mutex engineMutex;

// Metrics / analytics DB (single global instance; connects on startup)
MetricsCollector gMetrics;
// Active run tracking (protected by engineMutex)
std::string      gActiveRunId;
std::string      gActiveProjectName;
// Which project is currently loaded in the engine (set by /projects/:name/load)
std::string      gLoadedProjectName;
// OSM data for the currently loaded project (set by /projects/:name/load)
std::string      gCurrentOsmData;
// Set of OSM way IDs currently marked as closed (updated by PUT /road/close and WS setRoadClosed)
std::set<int64_t> gClosedRoadIds;

// ── Analysis session tracking ─────────────────────────────────────────────────
struct SessionState {
    std::string       sessionId;
    std::atomic<int>  completed{0};   // runs that finished successfully
    std::atomic<int>  terminated{0};  // runs that finished (success + fail)
    int               total = 0;
    std::atomic<bool> allDone{false};
    std::atomic<bool> cancelled{false};

    // Per-run progress [0.0, 1.0] — updated periodically from headless threads
    mutable std::mutex progressMutex;
    std::vector<double> runProgress;  // size = total, each in [0,1]

    void setRunProgress(int idx, double frac) {
        std::lock_guard<std::mutex> lk(progressMutex);
        if (idx >= 0 && idx < (int)runProgress.size())
            runProgress[idx] = frac;
    }
    // Overall percentage 0-100, counting completed runs as 100%
    int overallPct() const {
        std::lock_guard<std::mutex> lk(progressMutex);
        if (total <= 0) return 0;
        double sum = 0;
        for (double p : runProgress) sum += p;
        return std::min(100, (int)(sum / total * 100.0));
    }
};
std::mutex gSessionsMutex;
std::unordered_map<std::string, std::shared_ptr<SessionState>> gSessions;

// Per-client state: WS connection + the viewport the client is currently viewing
struct WsClient {
    std::weak_ptr<ix::WebSocket> ws;
    BoundingBox viewport = {{-1e9, -1e9}, {1e9, 1e9}}; // full world until client sets one
    // "agents" | "density" | "congestion"
    std::string mode = "agents";
};

std::mutex wsClientsMutex;
std::vector<WsClient> wsClients;

// Snapshot published by simLoop after each step; read by broadcastLoop without engineMutex.
struct SimSnapshot {
    std::vector<VehicleState>          vehicles;
    std::vector<IntersectionStateData> intersections;
    SimulationMetrics                  metrics;
    double simTime          = 0.0;
    double timeScale        = 0.0;
    double stepMs           = 0.0;  // EMA of engine.step() wall-clock time (ms)
    double maxTimeScale     = 0.0;  // how fast the sim COULD run without the sleep cap
};
std::shared_ptr<SimSnapshot> gSnapshot;
std::mutex gSnapshotMutex;
// Set to true by broadcastLoop after it consumes a snapshot; cleared by simLoop when it
// builds a new one.  Prevents rebuilding the full vehicle-state vector every step when the
// broadcast thread hasn't consumed the previous frame yet (critical at high time scales).
std::atomic<bool> gSnapshotConsumed{true};

// Global simulation time
std::atomic<double> gSimTimeSec{0.0};
// Global time scale (1.0 = normal speed, 0.0 = paused)
std::atomic<double> gTimeScale{1.0};
// WebSocket broadcast FPS — set from simulation.json at startup via setSimulationTuning
std::atomic<int> gBroadcastFps{30};

// Shutdown flag — set by signal handler; loops poll this
std::atomic<bool> gShutdown{false};
// Raw pointers set in main so the signal handler can stop the servers
static Server*               gSvr      = nullptr;
static ix::WebSocketServer*  gWsServer = nullptr;

static void handleSignal(int) {
    gShutdown.store(true, std::memory_order_relaxed);
    if (gSvr)      gSvr->stop();
    if (gWsServer) gWsServer->stop();
}

// -------------------- Simulation Loop --------------------
void simLoop() {
    const double FIXED_DT = 0.0166; // 60 FPS base step
    auto lastTime = std::chrono::high_resolution_clock::now();
    double simulationTime = 0.0; // Local clock
    double emaStepMs      = 0.0; // Exponential moving average of step wall-clock time
    bool   emaInit        = false;
    constexpr double EMA_ALPHA = 0.1; // smoothing factor (lower = slower to react)

    std::cout << "[SimThread] Simulation loop started." << std::endl;

    while (!gShutdown.load(std::memory_order_relaxed)) {
        try {
            auto now = std::chrono::high_resolution_clock::now();
            // We don't use wall-clock delta for step logic to keep it deterministic
            lastTime = now;

            double scale = gTimeScale.load(std::memory_order_relaxed);

            // If paused (scale ~0), just sleep
            if (scale < 0.001) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            // Calculate how much sim time we need to cover this frame
            double timeToSimulate = FIXED_DT * scale;

            // Sub-stepping: Maximum 66ms per physics step.
            // Safe for city speeds (~14 m/s max) since agents travel at most
            // ~0.9 m per substep — well below any realistic lane length.
            const double MAX_SUBSTEP = 0.066;

            {
                std::lock_guard<std::mutex> lock(engineMutex);

                // Time only the engine step, not the snapshot queries
                auto stepStart = std::chrono::high_resolution_clock::now();

                // Process the time in small chunks
                while (timeToSimulate > 0.0001) {
                    double step = std::min(timeToSimulate, MAX_SUBSTEP);
                    engine.step(step);
                    simulationTime += step;
                    timeToSimulate -= step;
                }

                double measuredMs = std::chrono::duration<double, std::milli>(
                    std::chrono::high_resolution_clock::now() - stepStart).count();

                if (!emaInit) { emaStepMs = measuredMs; emaInit = true; }
                else emaStepMs = EMA_ALPHA * measuredMs + (1.0 - EMA_ALPHA) * emaStepMs;

                // Build snapshot only when the broadcast thread has consumed the previous
                // one.  At high time scales the sim runs many steps per broadcast frame;
                // skipping redundant snapshot builds is the single largest throughput win.
                if (gSnapshotConsumed.exchange(false, std::memory_order_acq_rel)) {
                    auto snap = std::make_shared<SimSnapshot>();
                    snap->vehicles      = engine.getVehicleStatesInBounds({{-1e9,-1e9},{1e9,1e9}});
                    snap->intersections = engine.getIntersectionStates();
                    snap->metrics       = engine.getMetrics();
                    snap->simTime      = simulationTime;
                    snap->timeScale    = scale;
                    snap->stepMs       = emaStepMs;
                    snap->maxTimeScale = (emaStepMs > 0.0)
                        ? (FIXED_DT * scale * 1000.0 / emaStepMs)
                        : 0.0;
                    {
                        std::lock_guard<std::mutex> snapLock(gSnapshotMutex);
                        gSnapshot = std::move(snap);
                    }
                }
            }

            // Store the new time for any HTTP status endpoints
            gSimTimeSec.store(simulationTime, std::memory_order_relaxed);

            // Sleep only for whatever budget remains in the 16ms frame.
            // At low timescales the step finishes in ~2ms and we yield the rest.
            // At high timescales the step already exceeds 16ms so we skip the sleep.
            long sleepMs = 16L - static_cast<long>(emaStepMs);
            if (sleepMs > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
        }
        catch (const std::exception& e) {
            std::cerr << "[SimThread] CRITICAL ERROR: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    std::cout << "[SimThread] Exiting.\n";
}

/* [AI-generated: Claude] Refactor to this was done when trying to optimise the Simulation. */
// -------------------- Fast float formatters --------------------
// Avoids glibc's bignum path (hack_digit/__mpn_divrem) used by snprintf("%.Nf").
// Returns bytes written. buf must have at least 25 bytes of space.

static int fmtInt(char* buf, long long val) {
    int p = 0;
    if (val < 0) { buf[p++] = '-'; val = -val; }
    if (val == 0) { buf[p++] = '0'; return p; }
    char tmp[20]; int tlen = 0;
    for (long long n = val; n > 0; n /= 10) tmp[tlen++] = '0' + (int)(n % 10);
    for (int i = tlen - 1; i >= 0; --i) buf[p++] = tmp[i];
    return p;
}

// Fixed-decimal formatter: writes `val` with exactly `dec` decimal places.
// dec must be 0-5.
static int fmtFixed(char* buf, double val, int dec) {
    static constexpr long long pow10[] = {1, 10, 100, 1000, 10000, 100000};
    int p = 0;
    if (val < 0.0) { buf[p++] = '-'; val = -val; }
    long long scale = pow10[dec];
    long long ival  = (long long)(val * (double)scale + 0.5);
    long long ipart = ival / scale;
    long long fpart = ival % scale;
    // Integer part
    if (ipart == 0) {
        buf[p++] = '0';
    } else {
        char tmp[20]; int tlen = 0;
        for (long long n = ipart; n > 0; n /= 10) tmp[tlen++] = '0' + (int)(n % 10);
        for (int i = tlen - 1; i >= 0; --i) buf[p++] = tmp[i];
    }
    // Fractional part
    if (dec > 0) {
        buf[p++] = '.';
        for (int i = dec - 1; i >= 0; --i) {
            buf[p + i] = '0' + (int)(fpart % 10);
            fpart /= 10;
        }
        p += dec;
    }
    return p;
}

// Serialises one VehicleState into buf as a JSON object. Returns bytes written.
// buf must be at least 200 bytes.
static int appendVehicleJson(char* buf, const VehicleState& s) {
    const double safeGap   = std::isinf(s.gapToLeader)    ? 999999.0 : s.gapToLeader;
    const double safeAccel = std::isfinite(s.acceleration) ? s.acceleration : 0.0;
    int p = 0;
#define PUTLIT(str) do { std::memcpy(buf+p, str, sizeof(str)-1); p += sizeof(str)-1; } while(0)
    PUTLIT("{\"id\":");
    p += fmtInt(buf+p, (long long)s.id);
    PUTLIT(",\"x\":");
    p += fmtFixed(buf+p, s.pos.x, 4);
    PUTLIT(",\"y\":");
    p += fmtFixed(buf+p, s.pos.y, 4);
    PUTLIT(",\"h\":");
    p += fmtFixed(buf+p, s.heading, 5);
    PUTLIT(",\"speed\":");
    p += fmtFixed(buf+p, s.speed, 3);
    PUTLIT(",\"acceleration\":");
    p += fmtFixed(buf+p, safeAccel, 3);
    PUTLIT(",\"leaderId\":");
    p += fmtInt(buf+p, (long long)s.leaderId);
    PUTLIT(",\"gap\":");
    p += fmtFixed(buf+p, safeGap, 2);
    PUTLIT(",\"parked\":");
    if (s.parked) { PUTLIT("true"); } else { PUTLIT("false"); }
    PUTLIT(",\"timeUntilDeparture\":");
    p += fmtFixed(buf+p, s.timeUntilDeparture, 1);
    buf[p++] = '}';
#undef PUTLIT
    return p;
}
/* [end AI-generated] */

// -------------------- Broadcast Loop --------------------
void broadcastLoop() {
    const int broadcast_fps = gBroadcastFps.load(std::memory_order_relaxed);
    const auto interval = std::chrono::milliseconds(1000 / broadcast_fps);
    int broadcastCount = 0;

    std::cout << "[BroadcastThread] Broadcast loop started." << std::endl;

    while (!gShutdown.load(std::memory_order_relaxed)) {
        // --- Grab latest snapshot published by simLoop (no engine lock needed) ---
        std::shared_ptr<SimSnapshot> snap;
        {
            std::lock_guard<std::mutex> lock(gSnapshotMutex);
            snap = gSnapshot;
        }
        if (!snap) {
            std::this_thread::sleep_for(interval);
            continue;
        }

        // Signal simLoop that it may build the next snapshot.
        gSnapshotConsumed.store(true, std::memory_order_release);

        const auto& vehicleSnaps = snap->vehicles;
        const auto& interStates  = snap->intersections;
        const auto& metrics      = snap->metrics;
        const double simTime     = snap->simTime;
        const double timeScale   = snap->timeScale;

        // --- Build shared base payload as a raw JSON string (no nlohmann allocation) ---
        // This eliminates all json construction/destruction/serialisation cost in the
        // broadcast hot-path. The string ends with '}' so per-client code can splice
        // the "vehicles" key by stripping the trailing '}' and appending.
        std::string baseStr;
        baseStr.reserve(256 + interStates.size() * 64);
        char nbuf[32];

        baseStr += "{\"simTime\":";
        baseStr.append(nbuf, fmtFixed(nbuf, simTime, 3));
        baseStr += ",\"timeScale\":";
        baseStr.append(nbuf, fmtFixed(nbuf, timeScale, 3));
        baseStr += ",\"stepMs\":";
        baseStr.append(nbuf, fmtFixed(nbuf, snap->stepMs, 2));
        baseStr += ",\"maxTimeScale\":";
        baseStr.append(nbuf, fmtFixed(nbuf, snap->maxTimeScale, 2));

        // intersections array
        baseStr += ",\"intersections\":[";
        for (std::size_t ii = 0; ii < interStates.size(); ++ii) {
            const auto& is = interStates[ii];
            if (ii) baseStr += ',';
            baseStr += "{\"id\":";
            baseStr.append(nbuf, fmtInt(nbuf, (long long)is.id));
            baseStr += ",\"locks\":[";
            for (std::size_t li = 0; li < is.lockedConflicts.size(); ++li) {
                if (li) baseStr += ',';
                baseStr += "{\"c\":";
                baseStr.append(nbuf, fmtInt(nbuf, (long long)is.lockedConflicts[li].first));
                baseStr += ",\"a\":";
                baseStr.append(nbuf, fmtInt(nbuf, (long long)is.lockedConflicts[li].second));
                baseStr += '}';
            }
            baseStr += "],\"lights\":[";
            for (std::size_t gi = 0; gi < is.lightStates.size(); ++gi) {
                if (gi) baseStr += ',';
                baseStr += "{\"p\":";
                baseStr.append(nbuf, fmtInt(nbuf, (long long)is.lightStates[gi].first));
                baseStr += ",\"s\":";
                baseStr.append(nbuf, fmtInt(nbuf, (long long)is.lightStates[gi].second));
                baseStr += '}';
            }
            baseStr += "]}";
        }
        baseStr += ']';

        // metrics object
        baseStr += ",\"metrics\":{";
        baseStr += "\"totalVehicles\":";
        baseStr.append(nbuf, fmtInt(nbuf, (long long)metrics.totalVehicles));
        baseStr += ",\"vehiclesInQueue\":";
        baseStr.append(nbuf, fmtInt(nbuf, (long long)metrics.vehiclesInQueue));
        baseStr += ",\"vehiclesInIntersection\":";
        baseStr.append(nbuf, fmtInt(nbuf, (long long)metrics.vehiclesInIntersection));
        baseStr += ",\"yieldingVehicles\":";
        baseStr.append(nbuf, fmtInt(nbuf, (long long)metrics.yieldingVehicles));
        baseStr += ",\"avgSpeed\":";
        baseStr.append(nbuf, fmtFixed(nbuf, metrics.avgSpeed, 3));
        baseStr += ",\"maxSpeed\":";
        baseStr.append(nbuf, fmtFixed(nbuf, metrics.maxSpeed, 3));
        baseStr += ",\"avgGapToLeader\":";
        baseStr.append(nbuf, fmtFixed(nbuf, metrics.avgGapToLeader, 2));
        baseStr += ",\"totalSpawned\":";
        baseStr.append(nbuf, fmtInt(nbuf, (long long)metrics.totalSpawned));
        baseStr += ",\"totalCompleted\":";
        baseStr.append(nbuf, fmtInt(nbuf, (long long)metrics.totalCompleted));
        baseStr += ",\"throughputPerMinute\":";
        baseStr.append(nbuf, fmtFixed(nbuf, metrics.throughputPerMinute, 2));
        baseStr += ",\"avgTravelTime\":";
        baseStr.append(nbuf, fmtFixed(nbuf, metrics.avgTravelTime, 2));
        baseStr += "}}"; // close metrics + outer object

        // --- Snapshot client list (brief lock, then work outside it) ---
        struct ClientSnap {
            std::shared_ptr<ix::WebSocket> ws;
            BoundingBox viewport;
            std::string mode;
        };
        std::vector<ClientSnap> clientSnapshot;
        {
            std::lock_guard<std::mutex> lock(wsClientsMutex);
            for (auto it = wsClients.begin(); it != wsClients.end(); ) {
                if (auto ws = it->ws.lock()) {
                    clientSnapshot.push_back({std::move(ws), it->viewport, it->mode});
                    ++it;
                } else {
                    it = wsClients.erase(it);
                }
            }
        }

        // Heatmap updates fire at ~2 fps regardless of broadcast rate
        broadcastCount++;
        const bool doHeatmap = (broadcastCount % 15 == 0);

        // baseStr was built above as a raw JSON string ending with '}'.
        // Per-client code splices "vehicles" by stripping the last '}' and appending.

        // --- Per-client: send vehicles or heatmap data depending on mode ---
        for (auto& c : clientSnapshot) {

            if (c.mode == "agents") {
                // ── Agents mode: build vehicle array as a raw JSON string.
                // This avoids nlohmann allocation churn (push_back + ~basic_json)
                // and the costly grisu2 float serialiser.
                std::string vArr;
                vArr.reserve(vehicleSnaps.size() * 128);
                vArr += '[';
                bool first = true;
                char vbuf[200];
                for (const auto& s : vehicleSnaps) {
                    if (s.pos.x < c.viewport.min.x || s.pos.x > c.viewport.max.x ||
                        s.pos.y < c.viewport.min.y || s.pos.y > c.viewport.max.y) continue;
                    if (!first) vArr += ',';
                    first = false;
                    const int n = appendVehicleJson(vbuf, s);
                    vArr.append(vbuf, n);
                }
                vArr += ']';

                // Splice into the base payload string (strip trailing '}', append vehicles, close)
                std::string out;
                out.reserve(baseStr.size() + vArr.size() + 16);
                out.append(baseStr, 0, baseStr.size() - 1); // drop trailing '}'
                out += ",\"vehicles\":";
                out += vArr;
                out += '}';

                c.ws->send(out);

            } else {
                // ── Heatmap modes: vehicles=[], optional density/congestion data ─
                // Heatmap fires at ~2 fps so re-parsing the already-built baseStr
                // into a json object here is fine.
                json msg = json::parse(baseStr);
                msg["vehicles"] = json::array();

                if (doHeatmap) {
                    if (c.mode == "density") {
                        // KDE density: for each grid cell centre, count agents within RADIUS
                        constexpr int DX = 80, DY = 60;
                        constexpr double RADIUS = 20.0, R2 = RADIUS * RADIUS;
                        const BoundingBox& vp = c.viewport;
                        const double cellW = (vp.max.x - vp.min.x) / DX;
                        const double cellH = (vp.max.y - vp.min.y) / DY;

                        std::vector<int> counts(DX * DY, 0);
                        if (cellW > 0.0 && cellH > 0.0) {
                            for (const auto& v : vehicleSnaps) {
                                const int x0 = std::max(0,    (int)((v.pos.x - RADIUS - vp.min.x) / cellW));
                                const int x1 = std::min(DX-1, (int)((v.pos.x + RADIUS - vp.min.x) / cellW));
                                const int y0 = std::max(0,    (int)((v.pos.y - RADIUS - vp.min.y) / cellH));
                                const int y1 = std::min(DY-1, (int)((v.pos.y + RADIUS - vp.min.y) / cellH));
                                for (int cy = y0; cy <= y1; cy++) {
                                    const double py = vp.min.y + (cy + 0.5) * cellH;
                                    const double dy = v.pos.y - py;
                                    for (int cx = x0; cx <= x1; cx++) {
                                        const double px = vp.min.x + (cx + 0.5) * cellW;
                                        const double dx = v.pos.x - px;
                                        if (dx*dx + dy*dy <= R2) counts[cy * DX + cx]++;
                                    }
                                }
                            }
                        }

                        json density;
                        density["minX"] = vp.min.x; density["minY"] = vp.min.y;
                        density["maxX"] = vp.max.x; density["maxY"] = vp.max.y;
                        density["resX"] = DX;        density["resY"] = DY;
                        density["cells"] = std::move(counts);
                        msg["density"] = std::move(density);

                    } else if (c.mode == "congestion") {
                        // Lane stats: group all snapshot vehicles by their lane ID
                        struct LaneAcc { int count = 0; int stuck = 0; double speedSum = 0.0; };
                        std::unordered_map<int64_t, LaneAcc> laneAcc;
                        for (const auto& v : vehicleSnaps) {
                            if (v.laneId == -1) continue;
                            auto& a = laneAcc[v.laneId];
                            a.count++;
                            a.speedSum += v.speed;
                            if (v.speed < 1.0) a.stuck++;
                        }
                        json lsArr = json::array();
                        for (const auto& [id, a] : laneAcc) {
                            lsArr.push_back({
                                {"id",    id},
                                {"count", a.count},
                                {"stuck", a.stuck},
                                {"speed", a.count > 0 ? a.speedSum / a.count : 0.0}
                            });
                        }
                        msg["laneStats"] = std::move(lsArr);
                    }
                }

                c.ws->send(msg.dump());
            }
        }

        std::this_thread::sleep_for(interval);
    }
    std::cout << "[BroadcastThread] Exiting.\n";
}

// Helper: find frontend dir relative to the executable path (not CWD)
static std::string find_frontend_dir(const char* argv0) {
    namespace fs = std::filesystem;

    fs::path exe_path;
#if defined(__linux__)
    // Most robust on Linux
    try {
        exe_path = fs::read_symlink("/proc/self/exe");
    } catch (...) {
        exe_path = fs::absolute(argv0 ? fs::path(argv0) : fs::current_path());
    }
#else
    exe_path = fs::absolute(argv0 ? fs::path(argv0) : fs::current_path());
#endif
    auto exe_dir = exe_path.parent_path();

    // Try a few likely locations relative to the executable
    std::vector<fs::path> candidates = {
        exe_dir / "frontend",
        exe_dir / "../frontend",
        exe_dir / "../../frontend",
        exe_dir / "../../../frontend"
    };

    for (auto& p : candidates) {
        std::error_code ec;
        if (fs::exists(p, ec) && fs::is_directory(p, ec)) {
            return fs::weakly_canonical(p, ec).string();
        }
    }
    return {};
}

// -------------------- Config --------------------
// SimConfig is defined in SimConfig.hpp — included above.

static SimConfig loadConfig(const std::string& path) {
    SimConfig cfg;

    std::ifstream f(path);
    if (!f.is_open()) {
        std::cout << "[Config] '" << path << "' not found — using built-in defaults.\n";
        // Provide the default car type so the simulation isn't empty
        VehicleTypeConfig car;
        car.typeId       = "default_car";
        car.length       = 4.5;
        car.maxSpeed     = 15.0;
        car.acceleration = 2.5;
        car.deceleration = 4.0;
        cfg.vehicleTypes.push_back(car);
        return cfg;
    }

    try {
        auto j = json::parse(f);

        if (j.contains("mapFile"))          cfg.mapFile          = j["mapFile"];
        if (j.contains("defaultProject"))   cfg.defaultProject   = j["defaultProject"];
        if (j.contains("httpPort"))         cfg.httpPort         = j["httpPort"];
        if (j.contains("wsPort"))           cfg.wsPort           = j["wsPort"];
        if (j.contains("initialTimeScale")) cfg.initialTimeScale = j["initialTimeScale"];

        // --- Tuning parameters ---
        if (j.contains("broadcastFps"))                  cfg.tuning.broadcastFps                  = j["broadcastFps"];
        if (j.contains("parkingDestinationProbability")) cfg.tuning.parkingDestinationProbability = j["parkingDestinationProbability"];
        if (j.contains("speedUpdateIntervalS"))          cfg.tuning.speedUpdateIntervalS          = j["speedUpdateIntervalS"];
        if (j.contains("routeCacheTtlS"))                cfg.tuning.routeCacheTtlS               = j["routeCacheTtlS"];
        if (j.contains("isectSnapshotIntervalS"))        cfg.tuning.isectSnapshotIntervalS       = j["isectSnapshotIntervalS"];
        if (j.contains("agentPoolCapacity"))             cfg.tuning.agentPoolCapacity            = j["agentPoolCapacity"].get<std::size_t>();
        if (j.contains("debugBlockedReason"))            cfg.tuning.debugBlockedReason           = j["debugBlockedReason"].get<bool>();

        if (j.contains("mapParams")) {
            auto& mp = j["mapParams"];
            if (mp.contains("defaultLaneWidth"))
                cfg.mapParams.defaultLaneWidth = mp["defaultLaneWidth"];
            if (mp.contains("defaultSpeedLimit"))
                cfg.mapParams.defaultSpeedLimit = mp["defaultSpeedLimit"];
            if (mp.contains("defaultIntersectionCurbRadius"))
                cfg.mapParams.defaultIntersectionCurbRadius = mp["defaultIntersectionCurbRadius"];
            if (mp.contains("defaultTurnCurvatureWeight"))
                cfg.mapParams.defaultTurnCurvatureWeight = mp["defaultTurnCurvatureWeight"];
        }

        for (const auto& vt : j.value("vehicleTypes", json::array())) {
            VehicleTypeConfig v;
            v.typeId       = vt.at("typeId");
            v.length       = vt.at("length");
            v.maxSpeed     = vt.at("maxSpeed");
            v.acceleration = vt.value("acceleration", 2.5);
            v.deceleration = vt.value("deceleration", 4.0);
            cfg.vehicleTypes.push_back(std::move(v));
        }

        for (const auto& r : j.value("routes", json::array())) {
            RouteDefinition rd;
            rd.routeId = r.at("routeId");
            rd.wayIds  = r.at("wayIds").get<std::vector<int64_t>>();
            cfg.routes.push_back(std::move(rd));
        }

        for (const auto& sr : j.value("spawnRegions", json::array())) {
            SpawnRegionConfig s;
            s.regionId        = sr.at("regionId");
            s.spawnRatePerSec = sr.at("spawnRatePerSec");
            s.vehicleTypeId   = sr.at("vehicleTypeId");
            s.routePolicy     = sr.value("routePolicy", "random");
            s.bounds.min.x    = sr.at("bounds").at("min").at("x");
            s.bounds.min.y    = sr.at("bounds").at("min").at("y");
            s.bounds.max.x    = sr.at("bounds").at("max").at("x");
            s.bounds.max.y    = sr.at("bounds").at("max").at("y");
            cfg.spawnRegions.push_back(std::move(s));
        }

        std::cout << "[Config] Loaded from '" << path << "'\n";
    } catch (const std::exception& e) {
        std::cerr << "[Config] Parse error in '" << path << "': " << e.what()
                  << " — using built-in defaults.\n";
    }

    return cfg;
}

// ── SimConfig global (set once in main, read-only by headless threads) ────────
SimConfig gCfg;

// ── Headless runner ───────────────────────────────────────────────────────────
// Runs a complete simulation in the calling thread without any sleep cap.
// Writes trip/intersection data to gMetrics and finalises the run when done.
// Should be called from a detached std::thread.

static void runHeadlessSim(std::string runId,
                            int runIndex,
                            std::shared_ptr<SessionState> session,
                            std::string osmData,
                            SimConfig cfg,
                            double trafficMultiplier,
                            std::set<int64_t> closedRoads,
                            double targetDurationS,
                            uint64_t simSeed) {
    bool success = false;
    try {
        SimulationEngine eng;
        eng.setSeed(simSeed);  // deterministic demand: same seed = same spawn timing + destinations
        eng.setGlobalMapParameters(cfg.mapParams);
        eng.loadMapFromOSM(osmData);
        for (const auto& vt : cfg.vehicleTypes) eng.addVehicleType(vt);
        for (const auto& r  : cfg.routes)       eng.defineRoute(r);
        for (auto sr : cfg.spawnRegions) {
            sr.spawnRatePerSec *= trafficMultiplier;
            eng.addSpawnRegion(sr);
        }
        for (int64_t wayId : closedRoads) eng.setRoadClosed(wayId, true);

        eng.setMetricsCollector(&gMetrics, runId);

        // Step as fast as possible — no sleep, no WS overhead, no global state access
        constexpr double STEP = 0.066; // max safe step (66ms sim-time)
        double simTime = 0.0;
        int    stepCount = 0;

        while (simTime < targetDurationS && !session->cancelled.load(std::memory_order_relaxed)) {
            eng.step(STEP);
            simTime += STEP;
            ++stepCount;

            // Update progress every 100 steps (~6.6 sim-seconds)
            if (stepCount % 100 == 0) {
                session->setRunProgress(runIndex, simTime / targetDurationS);
            }
        }
        session->setRunProgress(runIndex, simTime / targetDurationS);

        if (session->cancelled.load()) {
            gMetrics.failRun(runId);
            std::cout << "[Headless] Run " << runId.substr(0, 8) << " cancelled.\n";
        } else {
            gMetrics.flush();
            gMetrics.finalizeRun(runId, simTime,
                                 eng.getTotalSpawned(),
                                 eng.getTotalSpawnedFreeflowSum());
            success = true;
            std::cout << "[Headless] Run " << runId.substr(0, 8)
                      << " done — " << simTime << "s sim\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "[Headless] Run " << runId.substr(0, 8)
                  << " FAILED: " << e.what() << "\n";
        gMetrics.failRun(runId);
    }

    if (success) ++session->completed;
    int term = ++session->terminated;
    if (term >= session->total) {
        if (session->cancelled.load() || session->completed.load() < session->total) {
            gMetrics.failSession(session->sessionId);
        } else {
            gMetrics.finalizeSession(session->sessionId);
        }
        session->allDone.store(true);
        std::cout << "[Headless] Session " << session->sessionId.substr(0, 8)
                  << " all done (" << session->completed.load() << "/" << session->total << " OK)\n";
    }
}

// -------------------- Main --------------------

int main(int argc, char** argv) {
    // Required on Windows; no-op elsewhere
    ix::initNetSystem();

    // --- Signal handlers for clean shutdown (SIGINT = Ctrl-C, SIGTERM = kill) ---
    std::signal(SIGINT,  handleSignal);
    std::signal(SIGTERM, handleSignal);

    // --- Load config (--config <path> or default simulation.json) ---
    std::string configPath = "simulation.json";
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--config") { configPath = argv[i + 1]; break; }
    }
    const SimConfig cfg = loadConfig(configPath);
    gCfg = cfg;                              // make accessible to headless threads
    gTimeScale.store(cfg.initialTimeScale);
    gBroadcastFps.store(cfg.tuning.broadcastFps);

    // Mark any sessions/runs that were left as 'running' from a previous server instance
    gMetrics.cleanupStaleRuns();

    // --- Initialize simulation ---
    std::cout << "--- Initializing Headless Simulation Engine ---\n";
    {
        std::lock_guard<std::mutex> lock(engineMutex);

        engine.setGlobalMapParameters(cfg.mapParams);
        engine.setSimulationTuning(cfg.tuning);
        setGlobalDebugBlockedReason(cfg.tuning.debugBlockedReason);

        std::ifstream osmFile(cfg.mapFile);
        if (!osmFile.is_open()) {
            std::cerr << "FATAL: Could not open OSM file: " << cfg.mapFile << "\n";
            return 1;
        }
        std::stringstream buffer;
        buffer << osmFile.rdbuf();

        try {
            engine.loadMapFromOSM(buffer.str());
        } catch (const std::exception& e) {
            std::cerr << "FATAL: Engine initialization failed: " << e.what() << "\n";
            return 1;
        }

        for (const auto& vt : cfg.vehicleTypes)   engine.addVehicleType(vt);
        for (const auto& r  : cfg.routes)          engine.defineRoute(r);
        for (const auto& sr : cfg.spawnRegions)    engine.addSpawnRegion(sr);
    }
    std::cout << "--- Simulation Initialized Successfully ---\n";

    // --- Start Simulation + Broadcast threads ---
    std::thread simThread(simLoop);
    std::thread broadcastThread(broadcastLoop);

    // --- Start WebSocket server (IXWebSocket) ---
    {
        int port = cfg.wsPort;
        std::string host = "0.0.0.0";

        ix::WebSocketServer wsServer(port, host);
        gWsServer = &wsServer;

        // Disable per-message deflate at the server level so it is never negotiated
        // during the WebSocket handshake.  The sim is CPU-bound; zlib compression
        // was consuming ~4-8% of total CPU for no bandwidth benefit on localhost.
        wsServer.disablePerMessageDeflate();

        wsServer.setOnConnectionCallback(
            [&](std::weak_ptr<ix::WebSocket> webSocket,
                std::shared_ptr<ix::ConnectionState> connectionState)
            {
                if (auto ws = webSocket.lock()) {
                    ws->setOnMessageCallback(
                        [webSocket, connectionState](const ix::WebSocketMessagePtr& msg)
                        {
                            if (msg->type == ix::WebSocketMessageType::Open) {
                                {
                                    std::lock_guard<std::mutex> lk(wsClientsMutex);
                                    wsClients.push_back(WsClient{webSocket});
                                }
                                std::cout << "WS open id=" << connectionState->getId() << "\n";
                            }
                            else if (msg->type == ix::WebSocketMessageType::Close) {
                                {
                                    std::lock_guard<std::mutex> lk(wsClientsMutex);
                                    wsClients.erase(
                                        std::remove_if(wsClients.begin(), wsClients.end(),
                                            [](const WsClient& c) { return c.ws.expired(); }),
                                        wsClients.end());
                                }
                                std::cout << "WS close id=" << connectionState->getId() << "\n";
                            }
                            else if (msg->type == ix::WebSocketMessageType::Message) {
                                try {
                                    auto cmd = json::parse(msg->str);
                                    const std::string type = cmd.at("cmd").get<std::string>();

                                    if (type == "setSpeed") {
                                        double val = std::clamp(
                                            cmd.at("value").get<double>(), 0.0, 100.0);
                                        gTimeScale.store(val);
                                    }
                                    else if (type == "setVehicleRoute") {
                                        int64_t vehicleId = cmd.at("vehicleId").get<int64_t>();
                                        std::string routeId = cmd.at("routeId").get<std::string>();
                                        std::lock_guard<std::mutex> lk(engineMutex);
                                        engine.setVehicleRoute(vehicleId, routeId);
                                    }
                                    else if (type == "setRoadClosed") {
                                        int64_t wayId   = cmd.at("wayId").get<int64_t>();
                                        bool    isClosed = cmd.at("isClosed").get<bool>();
                                        std::lock_guard<std::mutex> lk(engineMutex);
                                        engine.setRoadClosed(wayId, isClosed);
                                        if (isClosed) gClosedRoadIds.insert(wayId);
                                        else          gClosedRoadIds.erase(wayId);
                                        std::cout << "[WS cmd] way " << wayId
                                                  << (isClosed ? " closed" : " opened") << "\n";
                                    }
                                    else if (type == "setViewport") {
                                        BoundingBox vp;
                                        vp.min.x = cmd.at("minX").get<double>();
                                        vp.min.y = cmd.at("minY").get<double>();
                                        vp.max.x = cmd.at("maxX").get<double>();
                                        vp.max.y = cmd.at("maxY").get<double>();
                                        if (auto thisSp = webSocket.lock()) {
                                            std::lock_guard<std::mutex> lk(wsClientsMutex);
                                            for (auto& c : wsClients) {
                                                if (auto sp = c.ws.lock();
                                                    sp.get() == thisSp.get()) {
                                                    c.viewport = vp;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    else if (type == "setStreamMode") {
                                        std::string newMode = cmd.value("mode", "agents");
                                        if (newMode != "agents" && newMode != "density"
                                                                 && newMode != "congestion")
                                            newMode = "agents";
                                        if (auto thisSp = webSocket.lock()) {
                                            std::lock_guard<std::mutex> lk(wsClientsMutex);
                                            for (auto& c : wsClients) {
                                                if (auto sp = c.ws.lock();
                                                    sp.get() == thisSp.get()) {
                                                    c.mode = newMode;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    else {
                                        std::cerr << "[WS cmd] Unknown command: " << type << "\n";
                                    }
                                } catch (const std::exception& e) {
                                    std::cerr << "[WS cmd] Parse error from "
                                              << connectionState->getId()
                                              << ": " << e.what() << "\n";
                                }
                            }
                        }
                    );
                }
            }
        );

        auto res = wsServer.listen();
        if (!res.first) {
            std::cerr << "WS Failed to listen on port " << port << ": " << res.second << "\n";
            gShutdown.store(true, std::memory_order_relaxed);
            simThread.join();
            broadcastThread.join();
            return 1;
        }
        wsServer.start(); // background
        std::cout << "WebSocket server on ws://" << host << ":" << port << "\n";

        // --- Start HTTP server (httplib) ---
        Server svr;
        gSvr = &svr;

    // ---- Helpers ----

    auto jsonOk = [](Response& res, const json& body) {
        res.status = 200;
        res.set_content(body.dump(), "application/json");
    };

    auto jsonErr = [](Response& res, int status, const std::string& msg) {
        json j;
        j["error"] = msg;
        j["code"]  = status;
        res.status = status;
        res.set_content(j.dump(), "application/json");
    };

    // CORS pre-flight + headers on every response
    svr.set_pre_routing_handler([](const Request& req, Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
        if (req.method == "OPTIONS") {
            res.status = 204;
            return Server::HandlerResponse::Handled;
        }
        return Server::HandlerResponse::Unhandled;
    });

    // ---- Map ----

    // GET /map
    svr.Get("/map", [&](const Request&, Response& res) {
        std::string mapJson;
        {
            std::lock_guard<std::mutex> lock(engineMutex);
            mapJson = engine.getMapDataJSON();
        }
        res.set_content(mapJson, "application/json");
    });

    // POST /map/reset  — reload the original mapFile from simulation.json (the "default" map)
    svr.Post("/map/reset", [&](const Request&, Response& res) {
        std::ifstream osmFile(cfg.mapFile);
        if (!osmFile.is_open()) {
            jsonErr(res, 404, "default map file not found: " + cfg.mapFile);
            return;
        }
        std::stringstream buf;
        buf << osmFile.rdbuf();
        try {
            std::lock_guard<std::mutex> lock(engineMutex);
            engine = SimulationEngine{};
            engine.setGlobalMapParameters(cfg.mapParams);
            engine.loadMapFromOSM(buf.str());
            for (const auto& vt : cfg.vehicleTypes)   engine.addVehicleType(vt);
            for (const auto& r  : cfg.routes)          engine.defineRoute(r);
            for (const auto& sr : cfg.spawnRegions)    engine.addSpawnRegion(sr);
            gSimTimeSec.store(0.0);
            std::cout << "[API] Map reset to default: " << cfg.mapFile << "\n";
        } catch (const std::exception& e) {
            jsonErr(res, 500, std::string("map reset failed: ") + e.what());
            return;
        }
        jsonOk(res, {{"ok", true}});
    });

    // POST /map/reload  — body: raw OSM XML; reloads the entire map and restarts the simulation
    svr.Post("/map/reload", [&](const Request& req, Response& res) {
        if (req.body.empty()) {
            jsonErr(res, 400, "body must be raw OSM XML");
            return;
        }
        try {
            std::lock_guard<std::mutex> lock(engineMutex);
            engine = SimulationEngine{};
            engine.setGlobalMapParameters(cfg.mapParams);
            engine.loadMapFromOSM(req.body);   // internally calls pimpl_->initialize()
            // NOTE: do NOT call engine.initialize() here — that is a legacy stub
            // that loads a hardcoded JSON file and would overwrite the map we just built.
            for (const auto& vt : cfg.vehicleTypes)   engine.addVehicleType(vt);
            for (const auto& r  : cfg.routes)          engine.defineRoute(r);
            for (const auto& sr : cfg.spawnRegions)    engine.addSpawnRegion(sr);
            gSimTimeSec.store(0.0);
            std::cout << "[API] Map reloaded via POST /map/reload ("
                      << req.body.size() << " bytes)\n";
        } catch (const std::exception& e) {
            jsonErr(res, 500, std::string("map load failed: ") + e.what());
            return;
        }
        jsonOk(res, {{"ok", true}});
    });

    // ---- Simulation control ----

    // GET /simulation/project — which project is currently loaded in the engine
    svr.Get("/simulation/project", [&](const Request&, Response& res) {
        jsonOk(res, {{"projectName", gLoadedProjectName}});
    });

    // GET /simulation/status
    svr.Get("/simulation/status", [&](const Request&, Response& res) {
        SimulationMetrics metrics;
        {
            std::lock_guard<std::mutex> lock(engineMutex);
            metrics = engine.getMetrics();
        }
        json m;
        m["totalVehicles"]          = metrics.totalVehicles;
        m["vehiclesInQueue"]        = metrics.vehiclesInQueue;
        m["vehiclesInIntersection"] = metrics.vehiclesInIntersection;
        m["yieldingVehicles"]       = metrics.yieldingVehicles;
        m["avgSpeed"]               = metrics.avgSpeed;
        m["maxSpeed"]               = metrics.maxSpeed;
        m["avgGapToLeader"]         = metrics.avgGapToLeader;
        m["totalSpawned"]           = metrics.totalSpawned;
        m["totalCompleted"]         = metrics.totalCompleted;
        m["throughputPerMinute"]    = metrics.throughputPerMinute;
        m["avgTravelTime"]          = metrics.avgTravelTime;
        json out;
        out["simTime"]   = gSimTimeSec.load(std::memory_order_relaxed);
        out["timeScale"] = gTimeScale.load(std::memory_order_relaxed);
        out["metrics"]   = std::move(m);
        jsonOk(res, out);
    });

    // PUT /simulation/speed  { "value": 2.0 }
    // Also keep POST for backwards-compat with the frontend
    auto handleSpeed = [&](const Request& req, Response& res) {
        double val = 1.0;
        // Accept JSON body (new) or legacy query param (old frontend)
        if (!req.body.empty()) {
            try {
                val = json::parse(req.body).at("value").get<double>();
            } catch (...) {
                jsonErr(res, 400, "body must be JSON with a 'value' field");
                return;
            }
        } else if (req.has_param("value")) {
            try { val = std::stod(req.get_param_value("value")); }
            catch (...) { jsonErr(res, 400, "invalid 'value' parameter"); return; }
        } else {
            jsonErr(res, 400, "missing required field: value");
            return;
        }
        val = std::clamp(val, 0.0, 100.0);
        gTimeScale.store(val);
        std::cout << "[API] timeScale -> " << val << "x\n";
        jsonOk(res, {
            {"simTime",   gSimTimeSec.load(std::memory_order_relaxed)},
            {"timeScale", val}
        });
    };
    svr.Put("/simulation/speed", handleSpeed);
    svr.Post("/simulation/speed", handleSpeed); // legacy compat

    // ---- Vehicle queries ----

    // GET /vehicles?minX=&minY=&maxX=&maxY=
    svr.Get("/vehicles", [&](const Request& req, Response& res) {
        for (const auto& p : {"minX", "minY", "maxX", "maxY"}) {
            if (!req.has_param(p)) { jsonErr(res, 400, std::string("missing param: ") + p); return; }
        }
        BoundingBox bounds{};
        try {
            bounds.min.x = std::stod(req.get_param_value("minX"));
            bounds.min.y = std::stod(req.get_param_value("minY"));
            bounds.max.x = std::stod(req.get_param_value("maxX"));
            bounds.max.y = std::stod(req.get_param_value("maxY"));
        } catch (...) { jsonErr(res, 400, "invalid bounding-box parameter"); return; }

        std::vector<VehicleState> states;
        {
            std::lock_guard<std::mutex> lock(engineMutex);
            states = engine.getVehicleStatesInBounds(bounds);
        }

        json arr = json::array();
        for (const auto& s : states) {
            arr.push_back({
                {"id", s.id}, {"x", s.pos.x}, {"y", s.pos.y},
                {"h", s.heading}, {"speed", s.speed}, {"acceleration", s.acceleration}
            });
        }
        jsonOk(res, {{"vehicles", arr}});
    });

    // GET /vehicle/detail?id=
    svr.Get("/vehicle/detail", [&](const Request& req, Response& res) {
        if (!req.has_param("id")) { jsonErr(res, 400, "missing param: id"); return; }
        int64_t id = 0;
        try { id = std::stoll(req.get_param_value("id")); }
        catch (...) { jsonErr(res, 400, "invalid id"); return; }

        std::lock_guard<std::mutex> lock(engineMutex);
        auto detail = engine.getDetailedVehicleState(id);
        if (!detail.has_value()) { jsonErr(res, 404, "vehicle not found"); return; }

        json points = json::array();
        for (const auto& p : detail->debugHitbox) {
            json pt; pt["x"] = p.x; pt["y"] = p.y;
            points.push_back(pt);
        }

        json routePoly = json::array();
        for (const auto& p : detail->routePolyline) {
            routePoly.push_back({{"x", p.x}, {"y", p.y}});
        }

        json cp; cp["x"] = detail->conflictPointPos.x; cp["y"] = detail->conflictPointPos.y;

        json out;
        out["id"]           = detail->id;
        out["type"]         = detail->typeId;
        out["speed"]        = detail->speed;
        out["acceleration"] = detail->acceleration;
        out["state"]        = detail->controllerState;
        out["lane"]         = detail->currentLaneId;
        out["routeIdx"]     = detail->routeIndex;
        out["leaderId"]     = detail->leaderId;
        out["gap"]          = detail->gapToLeader;
        out["headway"]      = detail->timeHeadway;
        out["isYielding"]    = detail->isYielding;
        out["yieldingToId"]  = detail->yieldingToId;
        out["conflictPoint"] = cp;
        out["hitbox"]        = points;
        out["blockedReason"]  = detail->blockedReason;
        out["pathProgress"]   = detail->pathProgress;
        out["laneLength"]     = detail->laneLength;
        out["inIntersection"] = detail->inIntersection;
        out["routePolyline"]  = routePoly;
        out["isParkingRoute"] = detail->isParkingRoute;
        out["parkingDest"]    = {{"x", detail->parkingDest.x}, {"y", detail->parkingDest.y}};
        out["parkingSearchRadius"] = detail->parkingSearchRadius;
        out["isParked"]            = detail->isParked;
        out["timeUntilDeparture"]  = detail->timeUntilDeparture;

        json historyArr = json::array();
        for (const auto& ev : detail->history) {
            std::string typeStr;
            switch (ev.type) {
                case AgentHistoryEvent::Type::Spawned:              typeStr = "Spawned"; break;
                case AgentHistoryEvent::Type::StartedParkingSearch: typeStr = "StartedParkingSearch"; break;
                case AgentHistoryEvent::Type::TargetedSpot:         typeStr = "TargetedSpot"; break;
                case AgentHistoryEvent::Type::Parked:               typeStr = "Parked"; break;
                case AgentHistoryEvent::Type::Departed:             typeStr = "Departed"; break;
                case AgentHistoryEvent::Type::RouteRegenerated:     typeStr = "RouteRegenerated"; break;
                default:                                            typeStr = "Unknown"; break;
            }
            historyArr.push_back({{"type", typeStr}, {"simTime", ev.simTime}, {"detail", ev.detail}});
        }
        out["history"] = historyArr;
        jsonOk(res, out);
    });

    // PUT /vehicle/route  { "vehicleId": 42, "routeId": "commute_north" }
    svr.Put("/vehicle/route", [&](const Request& req, Response& res) {
        int64_t vehicleId = 0;
        std::string routeId;
        try {
            auto body = json::parse(req.body);
            vehicleId = body.at("vehicleId").get<int64_t>();
            routeId   = body.at("routeId").get<std::string>();
        } catch (...) {
            jsonErr(res, 400, "body must be JSON with 'vehicleId' (int) and 'routeId' (string)");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(engineMutex);
            engine.setVehicleRoute(vehicleId, routeId);
        }
        jsonOk(res, {{"ok", true}});
    });

    // ---- Road management ----

    // PUT /road/close  { "wayId": 12345678, "isClosed": true }
    svr.Put("/road/close", [&](const Request& req, Response& res) {
        int64_t wayId = 0;
        bool isClosed = false;
        try {
            auto body = json::parse(req.body);
            wayId    = body.at("wayId").get<int64_t>();
            isClosed = body.at("isClosed").get<bool>();
        } catch (...) {
            jsonErr(res, 400, "body must be JSON with 'wayId' (int) and 'isClosed' (bool)");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(engineMutex);
            engine.setRoadClosed(wayId, isClosed);
            if (isClosed) gClosedRoadIds.insert(wayId);
            else          gClosedRoadIds.erase(wayId);
        }
        std::cout << "[API] way " << wayId << (isClosed ? " closed" : " opened") << "\n";
        jsonOk(res, {{"ok", true}});
    });

    // ---- Analytics ----

    // GET /metrics
    svr.Get("/metrics", [&](const Request&, Response& res) {
        SimulationMetrics metrics;
        {
            std::lock_guard<std::mutex> lock(engineMutex);
            metrics = engine.getMetrics();
        }
        json m;
        m["totalVehicles"]          = metrics.totalVehicles;
        m["vehiclesInQueue"]        = metrics.vehiclesInQueue;
        m["vehiclesInIntersection"] = metrics.vehiclesInIntersection;
        m["yieldingVehicles"]       = metrics.yieldingVehicles;
        m["avgSpeed"]               = metrics.avgSpeed;
        m["maxSpeed"]               = metrics.maxSpeed;
        m["avgGapToLeader"]         = metrics.avgGapToLeader;
        m["totalSpawned"]           = metrics.totalSpawned;
        m["totalCompleted"]         = metrics.totalCompleted;
        m["throughputPerMinute"]    = metrics.throughputPerMinute;
        m["avgTravelTime"]          = metrics.avgTravelTime;
        jsonOk(res, m);
    });

    // GET /heatmap?minX=&minY=&maxX=&maxY=&resX=20&resY=20
    svr.Get("/heatmap", [&](const Request& req, Response& res) {
        for (const auto& p : {"minX", "minY", "maxX", "maxY"}) {
            if (!req.has_param(p)) { jsonErr(res, 400, std::string("missing param: ") + p); return; }
        }
        BoundingBox bounds{};
        int resX = 20, resY = 20;
        try {
            bounds.min.x = std::stod(req.get_param_value("minX"));
            bounds.min.y = std::stod(req.get_param_value("minY"));
            bounds.max.x = std::stod(req.get_param_value("maxX"));
            bounds.max.y = std::stod(req.get_param_value("maxY"));
            if (req.has_param("resX")) resX = std::stoi(req.get_param_value("resX"));
            if (req.has_param("resY")) resY = std::stoi(req.get_param_value("resY"));
        } catch (...) { jsonErr(res, 400, "invalid parameter"); return; }

        resX = std::clamp(resX, 1, 200);
        resY = std::clamp(resY, 1, 200);

        std::vector<HeatmapCell> cells;
        {
            std::lock_guard<std::mutex> lock(engineMutex);
            cells = engine.getVehicleHeatmap(bounds, resX, resY);
        }

        json arr = json::array();
        for (const auto& c : cells) {
            json minJ, maxJ, boundsJ, cell;
            minJ["x"] = c.bounds.min.x; minJ["y"] = c.bounds.min.y;
            maxJ["x"] = c.bounds.max.x; maxJ["y"] = c.bounds.max.y;
            boundsJ["min"] = minJ; boundsJ["max"] = maxJ;
            cell["bounds"]       = boundsJ;
            cell["vehicleCount"] = c.vehicleCount;
            cell["averageSpeed"] = c.averageSpeed;
            arr.push_back(cell);
        }
        json result; result["cells"] = arr;
        jsonOk(res, result);
    });

    // GET /heatmap/lanes  — per-lane vehicle count, stuck count, average speed
    svr.Get("/heatmap/lanes", [&](const Request&, Response& res) {
        std::vector<LaneTrafficStat> stats;
        {
            std::lock_guard<std::mutex> lock(engineMutex);
            stats = engine.getLaneTrafficStats();
        }
        json arr = json::array();
        for (const auto& s : stats) {
            arr.push_back({
                {"id",    s.laneId},
                {"count", s.vehicleCount},
                {"stuck", s.stuckCount},
                {"speed", s.averageSpeed}
            });
        }
        jsonOk(res, {{"lanes", std::move(arr)}});
    });

    // GET /parking/spots — all parking spot positions, headings, occupancy
    // GET /spawning  — returns enabled flag, per-region info, and total rate
    svr.Get("/spawning", [&](const Request&, Response& res) {
        std::lock_guard<std::mutex> lock(engineMutex);
        bool enabled = engine.isSpawningEnabled();
        auto regions = engine.getSpawnRegionInfo();
        double totalRate = 0.0;
        json regArr = json::array();
        for (const auto& r : regions) {
            totalRate += r.totalRate;
            regArr.push_back({
                {"id",          r.id},
                {"laneCount",   r.laneCount},
                {"ratePerLane", r.ratePerLane},
                {"totalRate",   r.totalRate},
                {"x",           r.x},
                {"y",           r.y},
                {"spawnCount",  r.spawnCount}
            });
        }
        json resp;
        resp["enabled"]   = enabled;
        resp["regions"]   = regArr;
        resp["totalRate"] = totalRate;
        jsonOk(res, resp);
    });

    // POST /spawning  — body: { "enabled": bool }
    svr.Post("/spawning", [&](const Request& req, Response& res) {
        bool enabled = true;
        try {
            auto body = json::parse(req.body);
            enabled = body.at("enabled").get<bool>();
        } catch (...) {
            jsonErr(res, 400, "body must be JSON with 'enabled' (bool)");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(engineMutex);
            engine.setSpawningEnabled(enabled);
        }
        jsonOk(res, {{"ok", true}, {"enabled", enabled}});
    });

    svr.Get("/parking/spots", [&](const Request&, Response& res) {
        std::string spotsJson;
        {
            std::lock_guard<std::mutex> lock(engineMutex);
            spotsJson = engine.getParkingSpotsJSON();
        }
        res.set_content(spotsJson, "application/json");
    });

    // ---- Road closure query ----

    // GET /road/closed — returns the set of way IDs currently marked closed
    svr.Get("/road/closed", [&](const Request&, Response& res) {
        json arr = json::array();
        {
            std::lock_guard<std::mutex> lk(engineMutex);
            for (int64_t id : gClosedRoadIds) arr.push_back(id);
        }
        jsonOk(res, {{"closedRoadIds", arr}});
    });

    // ---- Run management (Analytics DB) ----

    // POST /runs/start
    // Body: { "runType": "baseline"|"scenario"|"optimization", "label": "..." }
    // projectName is taken from gLoadedProjectName (authoritative).
    // closedRoadIds are taken from gClosedRoadIds automatically.
    // baselineRunId is auto-looked up from the DB (most recent completed baseline for this project).
    svr.Post("/runs/start", [&](const Request& req, Response& res) {
        json body;
        try { body = json::parse(req.body); } catch (...) {
            jsonErr(res, 400, "invalid JSON body"); return;
        }
        if (!body.contains("runType")) {
            jsonErr(res, 400, "runType is required"); return;
        }
        std::string runType = body.at("runType").get<std::string>();
        if (runType != "baseline" && runType != "scenario" && runType != "optimization") {
            jsonErr(res, 400, "runType must be baseline, scenario, or optimization"); return;
        }
        std::string label = body.value("label", "");

        std::string projectName;
        std::string closedRoadsStr;
        {
            std::lock_guard<std::mutex> lk(engineMutex);
            projectName = gLoadedProjectName;
            json arr = json::array();
            for (int64_t id : gClosedRoadIds) arr.push_back(id);
            closedRoadsStr = arr.dump();
        }

        if (projectName.empty()) {
            jsonErr(res, 400, "no project loaded — load a project first"); return;
        }

        // Auto-find baseline: most recent completed baseline run for this project
        std::string baselineId = (runType != "baseline")
            ? gMetrics.findBaselineRun(projectName)
            : "";

        std::string runId = gMetrics.startRun(projectName, runType, label,
                                               closedRoadsStr, baselineId);
        if (runId.empty()) {
            jsonErr(res, 500, "failed to create run record — check server log"); return;
        }
        {
            std::lock_guard<std::mutex> lk(engineMutex);
            gActiveRunId       = runId;
            gActiveProjectName = projectName;
            engine.setMetricsCollector(&gMetrics, runId);
        }
        jsonOk(res, {{"ok", true}, {"runId", runId},
                     {"projectName", projectName}, {"baselineRunId", baselineId},
                     {"closedRoadIds", json::parse(closedRoadsStr)}});
    });

    // POST /runs/:id/finish
    svr.Post(R"(/runs/([^/]+)/finish$)", [&](const Request& req, Response& res) {
        const std::string runId = req.matches[1];
        double  simDuration  = 0.0;
        int64_t spawned      = 0;
        double  freeflowSum  = 0.0;
        {
            std::lock_guard<std::mutex> lk(engineMutex);
            simDuration  = gSimTimeSec.load(std::memory_order_relaxed);
            spawned      = engine.getTotalSpawned();
            freeflowSum  = engine.getTotalSpawnedFreeflowSum();
            // Detach collector so new trips don't get written mid-finalize
            if (gActiveRunId == runId) {
                engine.setMetricsCollector(nullptr, "");
                gActiveRunId.clear();
                gActiveProjectName.clear();
            }
        }
        gMetrics.finalizeRun(runId, simDuration, spawned, freeflowSum);
        jsonOk(res, {{"ok", true}, {"runId", runId}});
    });

    // POST /runs/:id/fail
    svr.Post(R"(/runs/([^/]+)/fail$)", [&](const Request& req, Response& res) {
        const std::string runId = req.matches[1];
        {
            std::lock_guard<std::mutex> lk(engineMutex);
            if (gActiveRunId == runId) {
                engine.setMetricsCollector(nullptr, "");
                gActiveRunId.clear();
                gActiveProjectName.clear();
            }
        }
        gMetrics.failRun(runId);
        jsonOk(res, {{"ok", true}});
    });

    // GET /projects/:name/runs  — list all runs for a project, newest first
    svr.Get(R"(/projects/([^/]+)/runs$)", [&](const Request& req, Response& res) {
        const std::string name = req.matches[1];
        if (!gMetrics.isConnected()) {
            jsonErr(res, 503, "database unavailable"); return;
        }
        std::string runsJson = gMetrics.queryRunsForProject(name);
        res.status = 200;
        res.set_content(runsJson, "application/json");
    });

    // GET /runs/:id/trips  — trip records for histogram (capped at 5000)
    svr.Get(R"(/runs/([^/]+)/trips$)", [&](const Request& req, Response& res) {
        const std::string runId = req.matches[1];
        if (!gMetrics.isConnected()) {
            jsonErr(res, 503, "database unavailable"); return;
        }
        std::string tripsJson = gMetrics.queryTripsForRun(runId);
        res.status = 200;
        res.set_content(tripsJson, "application/json");
    });

    // GET /runs/active  — returns the currently active run ID (if any)
    svr.Get("/runs/active", [&](const Request&, Response& res) {
        std::lock_guard<std::mutex> lk(engineMutex);
        if (gActiveRunId.empty()) {
            jsonOk(res, {{"active", false}});
        } else {
            jsonOk(res, {{"active", true}, {"runId", gActiveRunId},
                          {"projectName", gActiveProjectName}});
        }
    });

    // ---- Analyses (top-level grouping) ----

    // POST /analyses
    // Body: { "name": "..." }
    svr.Post("/analyses", [&](const Request& req, Response& res) {
        json body;
        try { body = json::parse(req.body); } catch (...) {
            jsonErr(res, 400, "invalid JSON body"); return;
        }
        std::string name = body.value("name", "");
        if (name.empty()) { jsonErr(res, 400, "name is required"); return; }
        if (!gMetrics.isConnected()) { jsonErr(res, 503, "database unavailable"); return; }

        std::string projectName;
        { std::lock_guard<std::mutex> lk(engineMutex); projectName = gLoadedProjectName; }
        if (projectName.empty()) { jsonErr(res, 400, "no project loaded"); return; }

        // Generate a random seed once — all sims in this analysis share it for fair comparison
        uint64_t simSeed = std::random_device{}();
        std::string analysisId = gMetrics.createAnalysis(projectName, name, simSeed);
        if (analysisId.empty()) { jsonErr(res, 500, "failed to create analysis"); return; }
        jsonOk(res, {{"ok", true}, {"analysisId", analysisId}, {"name", name}});
    });

    // DELETE /analyses/:id
    svr.Delete(R"(/analyses/([^/]+)$)", [&](const Request& req, Response& res) {
        const std::string analysisId = req.matches[1];
        // Get sessions for this analysis and cancel any that are running
        std::string sessionsJson = gMetrics.querySessionsForAnalysis(analysisId);
        // Simple extraction of session IDs from JSON (look for "sessionId":"..." patterns)
        {
            std::lock_guard<std::mutex> lk(gSessionsMutex);
            size_t pos = 0;
            const std::string key = "\"sessionId\":\"";
            while ((pos = sessionsJson.find(key, pos)) != std::string::npos) {
                pos += key.size();
                size_t end = sessionsJson.find('"', pos);
                if (end == std::string::npos) break;
                std::string sid = sessionsJson.substr(pos, end - pos);
                auto it = gSessions.find(sid);
                if (it != gSessions.end()) {
                    it->second->cancelled.store(true);
                    gSessions.erase(it);
                }
                pos = end + 1;
            }
        }
        gMetrics.deleteAnalysis(analysisId);
        std::cout << "[Analysis] Analysis " << analysisId.substr(0, 8) << " deleted.\n";
        jsonOk(res, {{"ok", true}});
    });

    // GET /analyses/:id/sessions — all sessions for comparison
    svr.Get(R"(/analyses/([^/]+)/sessions$)", [&](const Request& req, Response& res) {
        const std::string analysisId = req.matches[1];
        if (!gMetrics.isConnected()) { jsonErr(res, 503, "database unavailable"); return; }
        std::string sessionsJson = gMetrics.querySessionsForAnalysis(analysisId);
        res.status = 200;
        res.set_content(sessionsJson, "application/json");
    });

    // GET /sessions/:id/hotspots
    svr.Get(R"(/sessions/([^/]+)/hotspots$)", [&](const Request& req, Response& res) {
        const std::string sessionId = req.matches[1];
        if (!gMetrics.isConnected()) { jsonErr(res, 503, "database unavailable"); return; }
        std::string json_out = gMetrics.queryHotspotsForSession(sessionId);
        res.status = 200;
        res.set_content(json_out, "application/json");
    });

    // GET /sessions/:id/trips — trip histogram data
    svr.Get(R"(/sessions/([^/]+)/trips$)", [&](const Request& req, Response& res) {
        const std::string sessionId = req.matches[1];
        if (!gMetrics.isConnected()) { jsonErr(res, 503, "database unavailable"); return; }
        std::string json_out = gMetrics.queryTripsForSession(sessionId);
        res.status = 200;
        res.set_content(json_out, "application/json");
    });

    // GET /intersections/positions — lightweight id→{x,y} for all intersections
    svr.Get("/intersections/positions", [&](const Request&, Response& res) {
        std::string pos;
        {
            std::lock_guard<std::mutex> lk(engineMutex);
            pos = engine.getIntersectionPositionsJSON();
        }
        res.status = 200;
        res.set_content(pos, "application/json");
    });

    // ---- Analysis sessions ----

    // POST /analysis/start
    // Body: { "analysisId": "...", "label": "Baseline", "runCount": 3, "simDurationS": 3600, "trafficMode": "medium" }
    // Spawns N headless simulation threads, each writing to its own run_id.
    svr.Post("/analysis/start", [&](const Request& req, Response& res) {
        json body;
        try { body = json::parse(req.body); } catch (...) {
            jsonErr(res, 400, "invalid JSON body"); return;
        }

        std::string analysisId  = body.value("analysisId", "");
        std::string label       = body.value("label", "");
        int runCount            = body.value("runCount", 1);
        double simDurationS     = body.value("simDurationS", 3600.0);
        std::string trafficMode = body.value("trafficMode", "medium");

        if (analysisId.empty()) { jsonErr(res, 400, "analysisId is required"); return; }
        if (trafficMode != "low" && trafficMode != "medium" && trafficMode != "high") {
            jsonErr(res, 400, "trafficMode must be low, medium, or high"); return;
        }
        runCount     = std::clamp(runCount, 1, 10);
        simDurationS = std::clamp(simDurationS, 60.0, 7200.0);

        if (!gMetrics.isConnected()) {
            jsonErr(res, 503, "database unavailable"); return;
        }

        // Fetch the shared RNG seed for this analysis (all runs in an analysis share the seed)
        uint64_t analysisSeed = gMetrics.queryAnalysisSeed(analysisId);

        // Capture current project state under the lock
        std::string projectName, osmData, closedRoadsStr;
        std::set<int64_t> closedCopy;
        {
            std::lock_guard<std::mutex> lk(engineMutex);
            projectName    = gLoadedProjectName;
            osmData        = gCurrentOsmData;
            closedCopy     = gClosedRoadIds;
            json arr = json::array();
            for (int64_t id : gClosedRoadIds) arr.push_back(id);
            closedRoadsStr = arr.dump();
        }

        if (projectName.empty()) {
            jsonErr(res, 400, "no project loaded — load a project first"); return;
        }
        if (osmData.empty()) {
            jsonErr(res, 400, "no OSM data — reload the project"); return;
        }

        // Baseline = no roads closed
        bool isBaseline = closedCopy.empty();
        std::string runType = isBaseline ? "baseline" : "scenario";
        if (label.empty()) label = isBaseline ? "Baseline" : "Scenario";

        // For scenario: find the baseline session within this same analysis.
        // Do not require status=completed — the baseline may still be running
        // when the scenario is created; the fitness is computed at finalization
        // time anyway, by which point the baseline will be done.
        std::string baselineSessionId;
        if (!isBaseline) {
            std::string sessionsJson = gMetrics.querySessionsForAnalysis(analysisId);
            // Find any session where isBaseline:true (no status requirement)
            auto findBaseline = [&]() -> std::string {
                size_t pos = 0;
                while ((pos = sessionsJson.find("\"isBaseline\":true", pos)) != std::string::npos) {
                    size_t sid_pos = sessionsJson.rfind("\"sessionId\":\"", pos);
                    if (sid_pos == std::string::npos) { pos++; continue; }
                    sid_pos += 13;
                    size_t sid_end = sessionsJson.find('"', sid_pos);
                    if (sid_end == std::string::npos) { pos++; continue; }
                    return sessionsJson.substr(sid_pos, sid_end - sid_pos);
                }
                return "";
            };
            baselineSessionId = findBaseline();
        }

        double multiplier = (trafficMode == "low") ? 0.25 : (trafficMode == "high") ? 3.0 : 1.0;

        std::string sessionId = gMetrics.createSession(
            projectName, label, trafficMode, runCount, simDurationS,
            closedRoadsStr, isBaseline, analysisId, baselineSessionId);
        if (sessionId.empty()) {
            jsonErr(res, 500, "failed to create session record — check server log"); return;
        }

        // Create in-memory session state for progress tracking
        auto state = std::make_shared<SessionState>();
        state->sessionId = sessionId;
        state->total     = runCount;
        state->runProgress.resize(runCount, 0.0);
        {
            std::lock_guard<std::mutex> lk(gSessionsMutex);
            gSessions[sessionId] = state;
        }

        // Snapshot config for threads (read-only after this point)
        SimConfig cfgCopy = gCfg;

        // Pre-create all run IDs before spawning threads
        std::vector<std::string> runIds;
        runIds.reserve(runCount);
        for (int i = 0; i < runCount; ++i) {
            std::string runLabel = label + " #" + std::to_string(i + 1);
            std::string runId = gMetrics.startRun(
                projectName, runType, runLabel, closedRoadsStr, "", sessionId);
            runIds.push_back(runId);
        }

        // Spawn N threads in parallel
        int launched = 0;
        for (int i = 0; i < runCount; ++i) {
            if (runIds[i].empty()) {
                --state->total;
                ++state->terminated;
                if (state->total <= 0) {
                    gMetrics.failSession(sessionId);
                    state->allDone.store(true);
                }
                continue;
            }
            ++launched;
            const int runIndex = i;
            const uint64_t runSeed = analysisSeed + static_cast<uint64_t>(i);
            std::thread([runId=runIds[i], runIndex, state, osmData, cfgCopy,
                          multiplier, closedCopy, simDurationS, runSeed]() {
                runHeadlessSim(runId, runIndex, state, osmData, cfgCopy,
                               multiplier, closedCopy, simDurationS, runSeed);
            }).detach();
        }

        std::cout << "[Analysis] Session " << sessionId.substr(0, 8)
                  << " started: " << launched << "/" << runCount
                  << " runs launched (" << trafficMode << " traffic, "
                  << simDurationS << "s sim)\n";

        jsonOk(res, {{"ok", true}, {"sessionId", sessionId},
                     {"runCount", launched}, {"projectName", projectName},
                     {"analysisId", analysisId}, {"isBaseline", isBaseline},
                     {"trafficMode", trafficMode}, {"simDurationS", simDurationS}});
    });

    // GET /analysis/:id/status  — poll for headless session progress
    svr.Get(R"(/analysis/([^/]+)/status$)", [&](const Request& req, Response& res) {
        const std::string sessionId = req.matches[1];

        std::shared_ptr<SessionState> state;
        {
            std::lock_guard<std::mutex> lk(gSessionsMutex);
            auto it = gSessions.find(sessionId);
            if (it != gSessions.end()) state = it->second;
        }

        if (!state) {
            // Not in memory — may have been from a previous server run
            jsonOk(res, {{"sessionId", sessionId},
                         {"status", "unknown"}, {"completed", 0}, {"total", 0}});
            return;
        }

        int  completed = state->completed.load();
        int  total     = state->total;
        bool done      = state->allDone.load();
        int  pct       = done ? 100 : state->overallPct();

        // Build per-run pct array
        json runPcts = json::array();
        {
            std::lock_guard<std::mutex> lk(state->progressMutex);
            for (double p : state->runProgress)
                runPcts.push_back((int)(p * 100));
        }

        jsonOk(res, {{"sessionId", sessionId},
                     {"status",    done ? "completed" : "running"},
                     {"completed", completed},
                     {"total",     total},
                     {"pct",       pct},
                     {"runPcts",   runPcts}});
    });

    // DELETE /analysis/:id  — cancel a running session and remove it from the DB
    svr.Delete(R"(/analysis/([^/]+)$)", [&](const Request& req, Response& res) {
        const std::string sessionId = req.matches[1];

        // Signal cancellation to any running threads
        std::shared_ptr<SessionState> state;
        {
            std::lock_guard<std::mutex> lk(gSessionsMutex);
            auto it = gSessions.find(sessionId);
            if (it != gSessions.end()) {
                state = it->second;
                gSessions.erase(it);
            }
        }
        if (state) {
            state->cancelled.store(true);
            // Don't wait for threads — they'll detect cancelled flag and exit cleanly
        }

        // Delete from DB (cascade removes runs, trips, intersection_snapshots)
        gMetrics.deleteSession(sessionId);
        std::cout << "[Analysis] Session " << sessionId.substr(0, 8) << " deleted.\n";
        jsonOk(res, {{"ok", true}});
    });

    // GET /projects/:name/analyses  — list all analyses for a project (with nested sessions)
    svr.Get(R"(/projects/([^/]+)/analyses$)", [&](const Request& req, Response& res) {
        const std::string name = req.matches[1];
        if (!gMetrics.isConnected()) {
            jsonErr(res, 503, "database unavailable"); return;
        }
        std::string analysesJson = gMetrics.queryAnalysesForProject(name);
        res.status = 200;
        res.set_content(analysesJson, "application/json");
    });

    // ── Optimizer API — registered from OptimizerRunner.cpp ──────────────────
    registerOptimizerRoutes(svr);

    // ── Sensitivity API — registered from SensitivityRunner.cpp ──────────────
    registerSensitivityRoutes(svr);

    // Resolve the frontend directory relative to the executable location
    std::string base_dir = find_frontend_dir(argc > 0 ? argv[0] : nullptr);
    if (base_dir.empty()) {
        std::cerr << "[HTTP] Error: could not locate 'frontend' relative to the executable.\n";
        std::cerr << "       Try placing 'frontend' next to your build's bin, or the project root.\n";
    } else {
        std::cout << "[HTTP] Serving static files from: " << base_dir << "\n";
        if (!svr.set_mount_point("/", base_dir.c_str())) {
            std::cerr << "[HTTP] Error: Could not mount '" << base_dir << "'.\n";
        }
    }

    // ---- Projects API ----
    // Projects are stored as subdirectories of <frontend>/../projects/
    // Each project dir contains:
    //   meta.json  — { name, description, createdAt, lastOpenedAt }
    //   map.osm    — raw OSM XML (optional; falls back to default map if absent)

    std::string projects_dir;
    if (!base_dir.empty()) {
        projects_dir = (std::filesystem::path(base_dir).parent_path() / "projects").string();
    } else {
        projects_dir = "projects";
    }
    {
        std::error_code ec;
        std::filesystem::create_directories(projects_dir, ec);
        if (ec) std::cerr << "[Projects] Warning: could not create projects dir: " << ec.message() << "\n";
        else    std::cout << "[Projects] Projects directory: " << projects_dir << "\n";
    }

    // ── Auto-load defaultProject from simulation.json ─────────────────────────
    if (!cfg.defaultProject.empty()) {
        const std::string& dpName = cfg.defaultProject;
        std::filesystem::path dpDir = std::filesystem::path(projects_dir) / dpName;
        if (!std::filesystem::exists(dpDir)) {
            std::cerr << "[Config] defaultProject '" << dpName << "' not found in " << projects_dir << " — skipping auto-load.\n";
        } else {
            std::filesystem::path osmPath = dpDir / "map.osm";
            std::string osmData;
            if (std::filesystem::exists(osmPath)) {
                std::ifstream f(osmPath, std::ios::binary);
                std::stringstream buf; buf << f.rdbuf();
                osmData = buf.str();
            } else {
                std::ifstream f(cfg.mapFile);
                std::stringstream buf; buf << f.rdbuf();
                osmData = buf.str();
            }
            try {
                std::lock_guard<std::mutex> lock(engineMutex);
                engine = SimulationEngine{};
                engine.setGlobalMapParameters(cfg.mapParams);
                engine.loadMapFromOSM(osmData);
                for (const auto& vt : cfg.vehicleTypes)  engine.addVehicleType(vt);
                for (const auto& r  : cfg.routes)         engine.defineRoute(r);
                for (const auto& sr : cfg.spawnRegions)   engine.addSpawnRegion(sr);
                gSimTimeSec.store(0.0);
                gLoadedProjectName = dpName;
                gCurrentOsmData    = osmData;
                gClosedRoadIds.clear();
                gTimeScale.store(0.0);
                std::cout << "[Config] Auto-loaded project '" << dpName << "' (" << osmData.size() << " bytes)\n";
            } catch (const std::exception& e) {
                std::cerr << "[Config] Auto-load of project '" << dpName << "' failed: " << e.what() << "\n";
            }
        }
    }

    // Helpers (lambdas capture projects_dir by reference)
    auto readProjectMeta = [&projects_dir](const std::string& name) -> json {
        std::filesystem::path p = std::filesystem::path(projects_dir) / name / "meta.json";
        std::ifstream f(p);
        if (!f.is_open()) return json{};
        try { json j; f >> j; return j; } catch (...) { return json{}; }
    };
    auto writeProjectMeta = [&projects_dir](const std::string& name, const json& meta) {
        std::filesystem::path p = std::filesystem::path(projects_dir) / name / "meta.json";
        std::ofstream f(p);
        if (f.is_open()) f << meta.dump(2);
    };
    auto nowIso = []() -> std::string {
        auto tp = std::chrono::system_clock::now();
        auto tt = std::chrono::system_clock::to_time_t(tp);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&tt));
        return std::string(buf);
    };
    auto sanitizeName = [](std::string s) -> std::string {
        for (char& c : s) {
            if (!std::isalnum((unsigned char)c) && c != '-' && c != '_' && c != ' ') c = '_';
        }
        // Trim leading/trailing spaces
        size_t a = s.find_first_not_of(' '), b = s.find_last_not_of(' ');
        return (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
    };

    // GET /projects — list all projects
    svr.Get("/projects", [&](const Request&, Response& res) {
        json arr = json::array();
        std::error_code ec;
        for (const auto& entry : std::filesystem::directory_iterator(projects_dir, ec)) {
            if (!entry.is_directory()) continue;
            const std::string name = entry.path().filename().string();
            json meta = readProjectMeta(name);
            json item;
            item["name"]         = name;
            item["description"]  = meta.value("description", "");
            item["createdAt"]    = meta.value("createdAt", "");
            item["lastOpenedAt"] = meta.value("lastOpenedAt", "");
            auto osmPath = entry.path() / "map.osm";
            std::error_code ec2;
            bool hasOsm = std::filesystem::exists(osmPath, ec2);
            item["hasOsm"]   = hasOsm;
            item["osmSize"]  = hasOsm ? (int64_t)std::filesystem::file_size(osmPath, ec2) : 0;
            arr.push_back(item);
        }
        // Sort by lastOpenedAt descending
        std::sort(arr.begin(), arr.end(), [](const json& a, const json& b) {
            return a["lastOpenedAt"].get<std::string>() > b["lastOpenedAt"].get<std::string>();
        });
        res.set_header("Content-Type", "application/json");
        res.set_content(arr.dump(), "application/json");
    });

    // POST /projects — create new project  { "name": "...", "description": "..." }
    svr.Post("/projects", [&](const Request& req, Response& res) {
        json body;
        try { body = json::parse(req.body); } catch (...) { jsonErr(res, 400, "invalid JSON body"); return; }
        if (!body.contains("name") || !body["name"].is_string()) { jsonErr(res, 400, "name is required"); return; }
        std::string name = sanitizeName(body["name"].get<std::string>());
        if (name.empty()) { jsonErr(res, 400, "invalid project name"); return; }

        std::filesystem::path projDir = std::filesystem::path(projects_dir) / name;
        if (std::filesystem::exists(projDir)) { jsonErr(res, 409, "project already exists"); return; }

        std::error_code ec;
        std::filesystem::create_directories(projDir, ec);
        if (ec) { jsonErr(res, 500, "could not create project directory"); return; }

        std::string ts = nowIso();
        json meta;
        meta["name"]         = name;
        meta["description"]  = body.value("description", "");
        meta["createdAt"]    = ts;
        meta["lastOpenedAt"] = ts;
        writeProjectMeta(name, meta);

        jsonOk(res, {{"ok", true}, {"name", name}});
    });

    // POST /projects/:name/osm — save OSM XML to project file
    svr.Post(R"(/projects/([^/]+)/osm$)", [&](const Request& req, Response& res) {
        const std::string name = req.matches[1];
        if (req.body.empty()) { jsonErr(res, 400, "body must be OSM XML"); return; }
        std::filesystem::path projDir = std::filesystem::path(projects_dir) / name;
        if (!std::filesystem::exists(projDir)) { jsonErr(res, 404, "project not found"); return; }
        std::filesystem::path osmPath = projDir / "map.osm";
        std::ofstream f(osmPath, std::ios::binary);
        if (!f.is_open()) { jsonErr(res, 500, "could not write OSM file"); return; }
        f.write(req.body.data(), req.body.size());
        jsonOk(res, {{"ok", true}, {"bytes", (int)req.body.size()}});
    });

    // POST /projects/:name/load — load project's map.osm into simulation
    svr.Post(R"(/projects/([^/]+)/load$)", [&](const Request& req, Response& res) {
        const std::string name = req.matches[1];
        std::filesystem::path projDir = std::filesystem::path(projects_dir) / name;
        if (!std::filesystem::exists(projDir)) { jsonErr(res, 404, "project not found"); return; }

        std::string osmData;
        std::filesystem::path osmPath = projDir / "map.osm";
        if (std::filesystem::exists(osmPath)) {
            std::ifstream f(osmPath, std::ios::binary);
            if (!f.is_open()) { jsonErr(res, 500, "could not read project OSM"); return; }
            std::stringstream buf; buf << f.rdbuf();
            osmData = buf.str();
        } else {
            // Fall back to default map
            std::ifstream f(cfg.mapFile);
            if (!f.is_open()) { jsonErr(res, 404, "no map.osm in project and no default map found"); return; }
            std::stringstream buf; buf << f.rdbuf();
            osmData = buf.str();
        }

        try {
            std::lock_guard<std::mutex> lock(engineMutex);
            engine = SimulationEngine{};
            engine.setGlobalMapParameters(cfg.mapParams);
            engine.loadMapFromOSM(osmData);
            for (const auto& vt : cfg.vehicleTypes)   engine.addVehicleType(vt);
            for (const auto& r  : cfg.routes)          engine.defineRoute(r);
            for (const auto& sr : cfg.spawnRegions)    engine.addSpawnRegion(sr);
            gSimTimeSec.store(0.0);
            gLoadedProjectName = name;
            gCurrentOsmData    = osmData;
            gClosedRoadIds.clear();        // reset closures for the new project
            gTimeScale.store(0.0);         // start paused — user verifies manually
            std::cout << "[Projects] Loaded project '" << name << "' ("
                      << osmData.size() << " bytes OSM)\n";
        } catch (const std::exception& e) {
            jsonErr(res, 500, std::string("map load failed: ") + e.what()); return;
        }

        // Update lastOpenedAt
        json meta = readProjectMeta(name);
        meta["lastOpenedAt"] = nowIso();
        writeProjectMeta(name, meta);

        jsonOk(res, {{"ok", true}, {"name", name}});
    });

    // GET /projects/:name/osm/raw — serve raw OSM XML file
    svr.Get(R"(/projects/([^/]+)/osm/raw$)", [&](const Request& req, Response& res) {
        const std::string name = req.matches[1];
        std::filesystem::path osmPath = std::filesystem::path(projects_dir) / name / "map.osm";
        if (!std::filesystem::exists(osmPath)) { jsonErr(res, 404, "no map.osm for this project"); return; }
        std::ifstream f(osmPath, std::ios::binary);
        if (!f.is_open()) { jsonErr(res, 500, "could not read map.osm"); return; }
        std::string body((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        res.set_content(body, "application/xml; charset=UTF-8");
    });

    // GET /projects/:name — get project metadata
    svr.Get(R"(/projects/([^/]+)$)", [&](const Request& req, Response& res) {
        const std::string name = req.matches[1];
        std::filesystem::path projDir = std::filesystem::path(projects_dir) / name;
        if (!std::filesystem::exists(projDir)) { jsonErr(res, 404, "project not found"); return; }
        json meta = readProjectMeta(name);
        meta["hasOsm"] = std::filesystem::exists(projDir / "map.osm");
        res.set_content(meta.dump(), "application/json");
    });

    // DELETE /projects/:name — delete project directory
    svr.Delete(R"(/projects/([^/]+)$)", [&](const Request& req, Response& res) {
        const std::string name = req.matches[1];
        std::filesystem::path projDir = std::filesystem::path(projects_dir) / name;
        if (!std::filesystem::exists(projDir)) { jsonErr(res, 404, "project not found"); return; }
        std::error_code ec;
        std::filesystem::remove_all(projDir, ec);
        if (ec) { jsonErr(res, 500, "could not delete project: " + ec.message()); return; }
        jsonOk(res, {{"ok", true}});
    });

    // PATCH /projects/:name — update name and/or description
    svr.Patch(R"(/projects/([^/]+)$)", [&](const Request& req, Response& res) {
        const std::string oldName = req.matches[1];
        std::filesystem::path oldDir = std::filesystem::path(projects_dir) / oldName;
        if (!std::filesystem::exists(oldDir)) { jsonErr(res, 404, "project not found"); return; }

        json body;
        try { body = json::parse(req.body); } catch (...) { jsonErr(res, 400, "invalid JSON body"); return; }

        std::string newName = sanitizeName(body.value("name", oldName));
        if (newName.empty()) { jsonErr(res, 400, "invalid project name"); return; }
        std::string newDesc = body.value("description", "");

        // Rename directory if name changed
        if (newName != oldName) {
            std::filesystem::path newDir = std::filesystem::path(projects_dir) / newName;
            if (std::filesystem::exists(newDir)) { jsonErr(res, 409, "a project with that name already exists"); return; }
            std::error_code ec;
            std::filesystem::rename(oldDir, newDir, ec);
            if (ec) { jsonErr(res, 500, "could not rename project: " + ec.message()); return; }
        }

        json meta = readProjectMeta(newName);
        meta["name"]        = newName;
        meta["description"] = newDesc;
        writeProjectMeta(newName, meta);

        jsonOk(res, {{"ok", true}, {"name", newName}});
    });

    // "/" is served automatically by the static mount point — it serves index.html
    auto serveApp = [&base_dir](const Request&, Response& res) {
        const std::string path = base_dir + "/app.html";
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs) { res.status = 404; res.set_content("app.html not found\n", "text/plain"); return; }
        std::string body((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
        res.set_content(body, "text/html; charset=UTF-8");
    };
    svr.Get("/app", serveApp);

        std::cout << "HTTP server on http://0.0.0.0:" << cfg.httpPort << "\n";
        svr.listen("0.0.0.0", cfg.httpPort); // blocks until svr.stop()

        // svr.listen() returned — ensure shutdown is signalled to other threads
        gShutdown.store(true, std::memory_order_relaxed);
        wsServer.stop();
        gSvr      = nullptr;
        gWsServer = nullptr;
    } // wsServer destroyed here

    std::cout << "[Main] Waiting for threads to stop…\n";
    simThread.join();
    broadcastThread.join();

    ix::uninitNetSystem();
    std::cout << "[Main] Clean shutdown complete.\n";
    return 0;
}