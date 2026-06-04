/*
 * Note: Gemini (Gemini 3.1 Pro) was used to assist with debugging HTTP endpoints
 * and with commenting this file. The implementation logic was written by the author.
 */
#include "../include/Optimizer.hpp"
#include "../include/SimulationEngine.hpp"
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>

using json = nlohmann::json;
using namespace httplib;

// ── Per-road result ───────────────────────────────────────────────────────────
struct SensRoadResult {
    int64_t     wayId;
    std::string name;
    InlineSimResult simResult;
    double      danc;
    double      fitness;  // danc / baselineDANC
    int32_t     excludedCount = 0;
};

// ── Session state (shared between runner thread and HTTP threads) ─────────────
struct SensitivityState {
    std::string sessionId;
    std::atomic<bool> cancelled{false};
    std::atomic<bool> allDone{false};

    std::mutex  mu;
    std::string statusStr          = "loading";
    int         totalRoads         = 0;
    int         roadsDone          = 0;
    bool        baselineDone       = false;
    double      baselineDANC       = 1.0;
    InlineSimResult baselineResult{};
    double      avgBlTripS         = 0.0;  // avg baseline trip duration (= Te default)
    double      avgSimMs           = 5000.0;
    int         maxParallel        = 5;
    double      simDurS            = 3600.0;
    int64_t     baselineS0         = 1;

    std::vector<SensRoadResult> results;
};

// ── Session registry ──────────────────────────────────────────────────────────
static std::mutex gSensMu;
static std::map<std::string, std::shared_ptr<SensitivityState>> gSensSessions;

// ── Helpers ───────────────────────────────────────────────────────────────────
static void sensOk(Response& res, const json& body) {
    res.status = 200;
    res.set_content(body.dump(), "application/json");
}
static void sensErr(Response& res, int status, const std::string& msg) {
    res.status = status;
    res.set_content(json{{"error", msg}}.dump(), "application/json");
}

// ── Runner (runs in a detached thread) ───────────────────────────────────────
static void runSensitivity(
    std::shared_ptr<SensitivityState> state,
    std::string osmData, SimConfig cfg,
    double trafficMult, double simDurS, int maxParallel, uint64_t seed)
{
    const double Tp = simDurS / 2.0;

    // ── 1. Load a temporary engine to discover all drivable road way IDs ──────
    std::vector<std::pair<int64_t, std::string>> roads;
    {
        SimulationEngine mapEng;
        mapEng.setGlobalMapParameters(cfg.mapParams);
        mapEng.loadMapFromOSM(osmData);
        for (const auto& vt : cfg.vehicleTypes) mapEng.addVehicleType(vt);

        try {
            auto j = json::parse(mapEng.getMapDataJSON());

            // Collect way IDs that have actual lane geometry (drivable roads only)
            std::set<int64_t> drivable;
            if (j.contains("lanes")) {
                for (const auto& l : j["lanes"]) {
                    int64_t lid = l.value("id", (int64_t)0);
                    if (lid > 0) drivable.insert(lid / 1000);
                }
            }

            if (j.contains("ways")) {
                for (const auto& w : j["ways"]) {
                    int64_t id = w.value("id", (int64_t)0);
                    if (id <= 0 || !drivable.count(id)) continue;
                    std::string name = w.value("name", "Way " + std::to_string(id));
                    roads.push_back({id, name});
                }
            }
        } catch (const std::exception& ex) {
            std::cerr << "[Sensitivity] Map parse error: " << ex.what() << "\n";
        }
    }

    {
        std::lock_guard<std::mutex> lk(state->mu);
        state->totalRoads = (int)roads.size();
        state->statusStr  = "baseline";
    }

    if (state->cancelled.load() || roads.empty()) {
        std::lock_guard<std::mutex> lk(state->mu);
        state->statusStr = roads.empty() ? "error" : "cancelled";
        state->allDone.store(true);
        return;
    }

    // ── 2. Run baseline (no roads closed, records trip log for replay) ────────
    TripLog blTripLog;
    auto t0bl  = std::chrono::steady_clock::now();
    auto blRes = runInlineSim(osmData, cfg, trafficMult, {}, simDurS, seed,
                              nullptr, &blTripLog);  // record
    double msBl = std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - t0bl).count();

    int64_t S0       = std::max((int64_t)1, blRes.spawned);
    double  avgBlTrip = blRes.completed > 0
        ? blRes.completedTravelS / (double)blRes.completed : Tp;
    double blDANC = inlineDANCE(blRes, S0, Tp, 0.0); // excluded=0 for baseline

    {
        std::lock_guard<std::mutex> lk(state->mu);
        state->baselineDANC   = blDANC;
        state->baselineResult = blRes;
        state->avgBlTripS     = avgBlTrip;
        state->baselineS0     = S0;
        state->avgSimMs       = msBl;
        state->baselineDone   = true;
        state->statusStr      = "running";
    }

    if (state->cancelled.load()) {
        std::lock_guard<std::mutex> lk(state->mu);
        state->statusStr = "cancelled";
        state->allDone.store(true);
        return;
    }

    // ── 3. Run per-road simulations in parallel (replay baseline trip log) ───
    int N = (int)roads.size();
    std::atomic<int> workIdx{0};

    auto worker = [&]() {
        while (!state->cancelled.load()) {
            int i = workIdx.fetch_add(1);
            if (i >= N) break;

            std::vector<CandidateItem> closed = {{roads[i].first, -1}};
            auto t0w  = std::chrono::steady_clock::now();
            auto res  = runInlineSim(osmData, cfg, trafficMult, closed, simDurS, seed,
                                     &blTripLog, nullptr);  // replay
            double ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t0w).count();

            double danc    = inlineDANCE(res, S0, Tp, avgBlTrip);
            double fitness = danc / std::max(1e-9, blDANC);

            std::lock_guard<std::mutex> lk(state->mu);
            state->avgSimMs = state->avgSimMs * 0.85 + ms * 0.15;
            state->roadsDone++;
            state->results.push_back({roads[i].first, roads[i].second, res, danc, fitness,
                                      res.excludedCount});
        }
    };

    int nT = std::min(maxParallel, N);
    std::vector<std::thread> workers;
    workers.reserve(nT);
    for (int t = 0; t < nT; t++) workers.emplace_back(worker);
    for (auto& w : workers) w.join();

    {
        std::lock_guard<std::mutex> lk(state->mu);
        state->statusStr = state->cancelled.load() ? "cancelled" : "completed";
        state->allDone.store(true);
    }
    std::cout << "[Sensitivity] " << state->sessionId.substr(0, 12)
              << " done: " << state->roadsDone << "/" << state->totalRoads << " roads\n";
}

// ── HTTP routes ───────────────────────────────────────────────────────────────
void registerSensitivityRoutes(Server& svr) {

    // POST /sensitivity/start
    svr.Post("/sensitivity/start", [](const Request& req, Response& res) {
        json body = json::parse(req.body, nullptr, false);
        if (body.is_discarded()) { sensErr(res, 400, "invalid JSON"); return; }

        std::string osmCopy;
        {
            extern std::mutex engineMutex;
            std::lock_guard<std::mutex> lk(engineMutex);
            osmCopy = gCurrentOsmData;
        }
        if (osmCopy.empty()) { sensErr(res, 400, "no OSM data loaded"); return; }

        double simDurS   = std::clamp(body.value("simDurationS", 3600.0), 60.0, 7200.0);
        int maxParallel  = std::max(1, std::min(16, body.value("maxParallel", 5)));
        std::string tMode = body.value("trafficMode", "medium");
        double mult      = (tMode == "low") ? 0.25 : (tMode == "high") ? 3.0 : 1.0;

        uint64_t seed    = (uint64_t)std::chrono::steady_clock::now().time_since_epoch().count();
        std::string sid  = "sens-" + std::to_string(seed);

        auto state = std::make_shared<SensitivityState>();
        state->sessionId   = sid;
        state->maxParallel = maxParallel;
        state->simDurS     = simDurS;

        {
            std::lock_guard<std::mutex> lk(gSensMu);
            gSensSessions[sid] = state;
        }

        SimConfig cfgCopy = gCfg;
        std::thread([state, osmCopy, cfgCopy, mult, simDurS, maxParallel, seed]() {
            runSensitivity(state, osmCopy, cfgCopy, mult, simDurS, maxParallel, seed);
        }).detach();

        sensOk(res, {{"ok", true}, {"sessionId", sid}});
    });

    // GET /sensitivity/:id/status
    svr.Get(R"(/sensitivity/([^/]+)/status$)", [](const Request& req, Response& res) {
        const std::string sid = req.matches[1];
        std::shared_ptr<SensitivityState> state;
        {
            std::lock_guard<std::mutex> lk(gSensMu);
            auto it = gSensSessions.find(sid);
            if (it == gSensSessions.end()) { sensErr(res, 404, "session not found"); return; }
            state = it->second;
        }

        std::lock_guard<std::mutex> slk(state->mu);

        int remaining = state->totalRoads - state->roadsDone;
        double eta    = (state->avgSimMs > 0 && remaining > 0 && state->baselineDone)
            ? (double)remaining * state->avgSimMs / (std::max(1, state->maxParallel) * 1000.0)
            : -1.0;

        json results = json::array();
        for (const auto& r : state->results) {
            results.push_back({
                {"wayId",            r.wayId},
                {"name",             r.name},
                {"danc",             r.danc},
                {"fitness",          r.fitness},
                {"spawned",          r.simResult.spawned},
                {"completed",        r.simResult.completed},
                {"completedTravelS", r.simResult.completedTravelS},
                {"simDurS",          r.simResult.simDurS},
                {"excludedCount",    r.excludedCount}
            });
        }

        sensOk(res, {
            {"status",       state->statusStr},
            {"totalRoads",   state->totalRoads},
            {"roadsDone",    state->roadsDone},
            {"baselineDone", state->baselineDone},
            {"baselineDANC", state->baselineDANC},
            {"baselineS0",   state->baselineS0},
            {"avgBlTripS",   state->avgBlTripS},
            {"baselineResult", {
                {"spawned",          state->baselineResult.spawned},
                {"completed",        state->baselineResult.completed},
                {"completedTravelS", state->baselineResult.completedTravelS},
                {"simDurS",          state->baselineResult.simDurS},
                {"excludedCount",    0}
            }},
            {"etaSeconds",  eta},
            {"simDurS",     state->simDurS},
            {"maxParallel", state->maxParallel},
            {"results",     results}
        });
    });

    // GET /sensitivity — list active sessions
    svr.Get("/sensitivity", [](const Request&, Response& res) {
        json arr = json::array();
        std::lock_guard<std::mutex> lk(gSensMu);
        for (const auto& [sid, st] : gSensSessions) {
            std::lock_guard<std::mutex> slk(st->mu);
            arr.push_back({
                {"sessionId",  sid},
                {"status",     st->statusStr},
                {"totalRoads", st->totalRoads},
                {"roadsDone",  st->roadsDone}
            });
        }
        res.set_content(arr.dump(), "application/json");
    });

    // POST /sensitivity/:id/cancel
    svr.Post(R"(/sensitivity/([^/]+)/cancel$)", [](const Request& req, Response& res) {
        const std::string sid = req.matches[1];
        std::lock_guard<std::mutex> lk(gSensMu);
        auto it = gSensSessions.find(sid);
        if (it == gSensSessions.end()) { sensErr(res, 404, "session not found"); return; }
        it->second->cancelled.store(true);
        sensOk(res, {{"ok", true}});
    });
}
