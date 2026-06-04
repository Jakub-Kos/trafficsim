/*
 * Note: Claude (Anthropic) was used under human supervision to assist with
 * implementing this file. The code is largely repetitive HTTP handler and JSON
 * serialization boilerplate — structurally uniform but error-prone in detail
 * (field names, status codes, null checks) — making it a practical candidate
 * for AI-assisted authoring with the author reviewing, testing, and owning the result.
 */
#include "../include/Optimizer.hpp"
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <map>
#include <string>
#include <memory>
#include <iostream>
#include <set>

using json = nlohmann::json;
using namespace httplib;

// ── Session registry ──────────────────────────────────────────────────────────
std::mutex gOptMu;
std::map<std::string, std::shared_ptr<OptimizerState>> gOptSessions;

// ── Utility helpers ───────────────────────────────────────────────────────────

static void jsonOkR(Response& res, const json& body) {
    res.status = 200;
    res.set_content(body.dump(), "application/json");
}
static void jsonErrR(Response& res, int status, const std::string& msg) {
    res.status = status;
    res.set_content(json{{"error", msg}}.dump(), "application/json");
}

// Build a JSON status object from an in-memory OptimizerState
static json stateToJson(const std::shared_ptr<OptimizerState>& state) {
    std::lock_guard<std::mutex> lk(state->mu);
    double avgMs    = state->avgSimMs;
    int    evRemain = std::max(0, state->totalEvals - state->evalsDone);
    int    np       = std::max(1, state->maxParallel);
    double etaSec   = (avgMs > 0.0 && state->N > 0)
        ? (evRemain * avgMs / (np * 1000.0)) : -1.0;

    json asgn = json::array();
    for (int v : state->bestAssignment) asgn.push_back(v);
    json pf = json::array();
    for (double f : state->bestPhaseFits) pf.push_back(f);
    json logArr = json::array();
    for (const auto& e : state->evalLog) {
        logArr.push_back({
            {"n",    e.evalNum},
            {"ph",   e.phaseIdx},
            {"danc", e.fitnessDANC < 1e17 ? json(e.fitnessDANC) : json(nullptr)},
            {"ms",   e.durationMs},
            {"lbl",  e.label}
        });
    }
    json histArr = json::array();
    for (const auto& [n, f] : state->convergenceHistory)
        histArr.push_back({{"n", n}, {"fit", f}});

    return {
        {"status",           state->statusStr},
        {"algorithm",        state->algorithm},
        {"scenario",         scenarioToString(state->scenario)},
        {"itersDone",        state->itersDone},
        {"evalsDone",        state->evalsDone},
        {"totalEvals",       state->totalEvals},
        {"baselineDANC",     state->baselineDANC},
        {"bestFitness",      state->bestFitness < 1e17 ? json(state->bestFitness) : json(nullptr)},
        {"bestAssignment",   asgn},
        {"phaseFitnesses",   pf},
        {"phaseCount",       state->phaseCount},
        {"threshold",        state->threshold},
        {"roadCount",        state->N},
        {"bestClosedCount",  state->bestClosedCount},
        {"etaSeconds",       etaSec},
        {"recentEvals",      logArr},
        {"convergenceHistory", histArr},
    };
}

// ─────────────────────────────────────────────────────────────────────────────
void registerOptimizerRoutes(Server& svr) {

    // ── POST /optimizer/problems — create / upsert a problem ─────────────────
    svr.Post("/optimizer/problems", [](const Request& req, Response& res) {
        if (gLoadedProjectName.empty()) { jsonErrR(res, 400, "no project loaded"); return; }
        if (!gMetrics.isConnected())    { jsonErrR(res, 503, "DB not available");  return; }

        json body;
        try { body = json::parse(req.body); } catch(...) { jsonErrR(res, 400, "invalid JSON"); return; }

        std::string name      = body.value("name", "Problem");
        int         phases    = body.value("phaseCount", 2);
        double      simDur    = body.value("simDurationS", 3600.0);
        std::string tMode     = body.value("trafficMode", "medium");
        std::string scenStr   = body.value("scenarioType", "phased_construction");
        double      threshold = body.value("constraintThreshold", 1.15);

        // candidateItems: array of int (road wayId) or {k,w,i} objects
        std::string itemsJson = "[]";
        if (body.contains("candidateItems") && body["candidateItems"].is_array())
            itemsJson = body["candidateItems"].dump();
        else if (body.contains("candidateRoadIds") && body["candidateRoadIds"].is_array())
            itemsJson = body["candidateRoadIds"].dump();

        std::string pid = gMetrics.createOptimizationProblem(
            gLoadedProjectName, name, itemsJson, phases, simDur, tMode, scenStr, threshold);
        if (pid.empty()) { jsonErrR(res, 500, "failed to create problem"); return; }

        jsonOkR(res, {{"ok", true}, {"problemId", pid}});
    });

    // ── GET /optimizer/problems ───────────────────────────────────────────────
    svr.Get("/optimizer/problems", [](const Request&, Response& res) {
        if (gLoadedProjectName.empty()) { jsonErrR(res, 400, "no project loaded"); return; }
        if (!gMetrics.isConnected()) { res.set_content("[]", "application/json"); return; }
        res.set_content(gMetrics.queryOptimizationProblems(gLoadedProjectName), "application/json");
    });

    // ── DELETE /optimizer/problems/:id ────────────────────────────────────────
    svr.Delete(R"(/optimizer/problems/([^/]+)$)", [](const Request& req, Response& res) {
        gMetrics.deleteOptimizationProblem(req.matches[1]);
        jsonOkR(res, {{"ok", true}});
    });

    // ── POST /optimizer/sessions — start a new algorithm run ─────────────────
    svr.Post("/optimizer/sessions", [](const Request& req, Response& res) {
        json body;
        try { body = json::parse(req.body); } catch(...) { jsonErrR(res, 400, "invalid JSON"); return; }

        std::string problemId   = body.value("problemId", "");
        std::string algorithm   = body.value("algorithm", "hill_climbing");
        int         maxIter     = body.value("maxIter", 50);
        int         maxParallel = std::max(1, std::min(16, body.value("maxParallelSims", 4)));
        int         runsPerEval = std::max(1, std::min(5,  body.value("runsPerEval", 1)));
        double      teMult      = std::max(0.0, body.value("teMult", 1.0));

        // SA hyperparams
        double T0   = body.value("saT0",   0.10);
        double Tmin = body.value("saTmin", 0.005);

        // GA hyperparams
        int    popSize   = std::max(4, body.value("gaPopSize", 20));
        double mutProb   = body.value("gaMutProb",   -1.0);  // -1 → 1/N
        double crossProb = body.value("gaCrossProb",  0.9);

        if (problemId.empty()) { jsonErrR(res, 400, "problemId required"); return; }

        std::string osmCopy;
        {
            extern std::mutex engineMutex;
            std::lock_guard<std::mutex> lk(engineMutex);
            osmCopy = gCurrentOsmData;
        }
        if (osmCopy.empty()) { jsonErrR(res, 400, "no OSM data loaded"); return; }

        std::string probJson = gMetrics.queryOptimizationProblemById(problemId);
        if (probJson.empty()) { jsonErrR(res, 404, "problem not found"); return; }

        json pj;
        try { pj = json::parse(probJson); } catch(...) { jsonErrR(res, 500, "bad problem data"); return; }

        int    phaseCount = pj.value("phaseCount", 2);
        double simDurS    = pj.value("simDurationS", 3600.0);
        std::string tMode = pj.value("trafficMode", "medium");
        std::string scen  = pj.value("scenarioType", "phased_construction");
        double threshold  = pj.value("constraintThreshold", 1.15);

        // Parse candidate items (supports old integer format + new object format)
        std::string itemsStr = "[]";
        if (pj.contains("candidateRoadIds")) {
            // column name is still candidateRoadIds in the JSON for backward compat
            itemsStr = pj["candidateRoadIds"].is_string()
                ? pj["candidateRoadIds"].get<std::string>()
                : pj["candidateRoadIds"].dump();
        }
        std::vector<CandidateItem> candidates = parseCandidateItems(itemsStr);
        if (candidates.empty()) { jsonErrR(res, 400, "no candidate items in problem"); return; }

        double mult = (tMode == "low") ? 0.25 : (tMode == "high") ? 3.0 : 1.0;
        if (mutProb < 0) mutProb = 1.0 / candidates.size();

        // Use caller-supplied seed for reproducible / comparable runs; fall back to clock.
        uint64_t seed = body.contains("seed") && body["seed"].is_number_unsigned()
            ? body["seed"].get<uint64_t>()
            : (uint64_t)std::chrono::steady_clock::now().time_since_epoch().count();

        // Create session record in DB
        json hp = {
            {"maxIter",     maxIter},
            {"maxParallel", maxParallel},
            {"runsPerEval", runsPerEval},
            {"teMult",      teMult},
            {"saT0",        T0},
            {"saTmin",      Tmin},
            {"gaPopSize",   popSize},
            {"gaMutProb",   mutProb},
            {"gaCrossProb", crossProb},
            {"seed",        seed}
        };
        std::string sesId;
        if (gMetrics.isConnected()) {
            sesId = gMetrics.createOptimizationSession(
                problemId, gLoadedProjectName, algorithm, hp.dump());
        }
        if (sesId.empty()) sesId = "local-" + std::to_string(seed);

        auto state = std::make_shared<OptimizerState>();
        state->sessionId   = sesId;
        state->problemId   = problemId;
        state->scenario    = scenarioFromString(scen);
        state->phaseCount  = phaseCount;
        state->threshold   = threshold;
        state->N              = (int)candidates.size();
        state->maxIter        = maxIter;
        state->maxParallel    = maxParallel;
        state->runsPerEval    = runsPerEval;
        state->teMult         = teMult;
        state->algorithm      = algorithm;
        state->candidatesJson  = itemsStr;
        state->hyperparamsJson = hp.dump();
        {
            std::lock_guard<std::mutex> lk(gOptMu);
            gOptSessions[sesId] = state;
        }

        SimConfig cfgCopy = gCfg;

        if (algorithm == "simulated_annealing") {
            std::thread([state, osmCopy, cfgCopy, mult, candidates, simDurS, seed, T0, Tmin]() {
                runSA(state, osmCopy, cfgCopy, mult, candidates, simDurS, seed, T0, Tmin);
            }).detach();
        } else if (algorithm == "genetic_algorithm") {
            std::thread([state, osmCopy, cfgCopy, mult, candidates, simDurS, seed,
                         popSize, mutProb, crossProb]() {
                runGA(state, osmCopy, cfgCopy, mult, candidates, simDurS, seed,
                      popSize, mutProb, crossProb);
            }).detach();
        } else {
            // hill_climbing (default)
            std::thread([state, osmCopy, cfgCopy, mult, candidates, simDurS, seed]() {
                runHC(state, osmCopy, cfgCopy, mult, candidates, simDurS, seed);
            }).detach();
        }

        jsonOkR(res, {{"ok", true}, {"sessionId", sesId}, {"seed", seed}});
    });

    // ── GET /optimizer/sessions/:id/status — live state or DB fallback ────────
    svr.Get(R"(/optimizer/sessions/([^/]+)/status$)", [](const Request& req, Response& res) {
        const std::string sid = req.matches[1];
        {
            std::lock_guard<std::mutex> lk(gOptMu);
            auto it = gOptSessions.find(sid);
            if (it != gOptSessions.end()) {
                res.set_content(stateToJson(it->second).dump(), "application/json");
                return;
            }
        }
        // Not in memory — try DB (session survived server restart)
        if (gMetrics.isConnected()) {
            std::string dbJson = gMetrics.queryOptimizationSessionById(sid);
            if (!dbJson.empty()) {
                res.set_content(dbJson, "application/json");
                return;
            }
        }
        jsonErrR(res, 404, "session not found");
    });

    // ── GET /optimizer/sessions — all live sessions ───────────────────────────
    svr.Get("/optimizer/sessions", [](const Request&, Response& res) {
        json arr = json::array();
        std::lock_guard<std::mutex> lk(gOptMu);
        for (const auto& [sid, state] : gOptSessions) {
            std::lock_guard<std::mutex> slk(state->mu);
            arr.push_back({
                {"sessionId",  sid},
                {"problemId",  state->problemId},
                {"algorithm",  state->algorithm},
                {"scenario",   scenarioToString(state->scenario)},
                {"status",     state->statusStr},
                {"evalsDone",  state->evalsDone},
                {"totalEvals", state->totalEvals},
                {"bestFitness", state->bestFitness < 1e17 ? json(state->bestFitness) : json(nullptr)}
            });
        }
        res.set_content(arr.dump(), "application/json");
    });

    // ── GET /optimizer/problems/:id/sessions — past runs (live + DB) ──────────
    svr.Get(R"(/optimizer/problems/([^/]+)/sessions$)", [](const Request& req, Response& res) {
        const std::string pid = req.matches[1];

        // Collect live in-memory sessions for this problem
        json inMem = json::array();
        {
            std::lock_guard<std::mutex> lk(gOptMu);
            for (const auto& [sid, st] : gOptSessions) {
                if (st->problemId != pid) continue;
                std::lock_guard<std::mutex> slk(st->mu);
                json aj = json::array(); for (int v : st->bestAssignment) aj.push_back(v);
                json pf = json::array(); for (double f : st->bestPhaseFits) pf.push_back(f);
                json hj = json::array();
                for (const auto& [n, f2] : st->convergenceHistory)
                    hj.push_back({{"n", n}, {"fit", f2}});
                inMem.push_back({
                    {"sessionId",          sid},
                    {"algorithm",          st->algorithm},
                    {"scenario",           scenarioToString(st->scenario)},
                    {"status",             st->statusStr},
                    {"createdAt",          ""},
                    {"bestFitness",        st->bestFitness < 1e17 ? json(st->bestFitness) : json(nullptr)},
                    {"bestAssignment",     aj},
                    {"phaseFitnesses",     pf},
                    {"phaseCount",         st->phaseCount},
                    {"threshold",          st->threshold},
                    {"bestClosedCount",    st->bestClosedCount},
                    {"itersDone",          st->itersDone},
                    {"evalsDone",          st->evalsDone},
                    {"baselineDANC",       st->baselineDANC},
                    {"totalEvals",         st->totalEvals},
                    {"convergenceHistory", hj},
                    {"candidateRoadIds",   st->candidatesJson.empty()  ? json::array()  : json::parse(st->candidatesJson)},
                    {"hyperparams",        st->hyperparamsJson.empty() ? json::object() : json::parse(st->hyperparamsJson)},
                    {"inMemory",           true}
                });
            }
        }

        // Merge with DB sessions (DB is authoritative for completed runs)
        json dbArr = json::array();
        if (gMetrics.isConnected()) {
            try { dbArr = json::parse(gMetrics.queryOptimizationSessionsForProblem(pid)); }
            catch (...) {}
        }

        std::set<std::string> seen;
        json result = json::array();
        for (auto& s : inMem) { seen.insert(s["sessionId"].get<std::string>()); result.push_back(s); }
        for (auto& s : dbArr) { if (!seen.count(s["sessionId"].get<std::string>())) result.push_back(s); }
        res.set_content(result.dump(), "application/json");
    });

    // ── POST /optimizer/sessions/:id/pause ────────────────────────────────────
    svr.Post(R"(/optimizer/sessions/([^/]+)/pause$)", [](const Request& req, Response& res) {
        const std::string sid = req.matches[1];
        std::lock_guard<std::mutex> lk(gOptMu);
        auto it = gOptSessions.find(sid);
        if (it == gOptSessions.end()) { jsonErrR(res, 404, "not found"); return; }
        it->second->paused.store(true);
        jsonOkR(res, {{"ok", true}});
    });

    // ── POST /optimizer/sessions/:id/resume ───────────────────────────────────
    svr.Post(R"(/optimizer/sessions/([^/]+)/resume$)", [](const Request& req, Response& res) {
        const std::string sid = req.matches[1];
        std::lock_guard<std::mutex> lk(gOptMu);
        auto it = gOptSessions.find(sid);
        if (it == gOptSessions.end()) { jsonErrR(res, 404, "not found"); return; }
        it->second->paused.store(false);
        jsonOkR(res, {{"ok", true}});
    });

    // ── POST /optimizer/sessions/:id/cancel ───────────────────────────────────
    svr.Post(R"(/optimizer/sessions/([^/]+)/cancel$)", [](const Request& req, Response& res) {
        const std::string sid = req.matches[1];
        std::lock_guard<std::mutex> lk(gOptMu);
        auto it = gOptSessions.find(sid);
        if (it == gOptSessions.end()) { jsonErrR(res, 404, "not found"); return; }
        it->second->cancelled.store(true);
        it->second->paused.store(false);  // unblock if paused
        jsonOkR(res, {{"ok", true}});
    });
}
