/*
 * Note: Claude (Anthropic) was used under human supervision for two purposes here:
 * documenting non-obvious algorithmic decisions, and integrating the E term
 * (exclusion penalty) into the DANCE score across all three algorithms. E term was
 * created after the implementation of the following algorithms.
 * Core algorithm logic (HC, SA, GA) was written by the author.
 */
#include "../include/Optimizer.hpp"
#include "../include/SimulationEngine.hpp"
#include <nlohmann/json.hpp>

#include <chrono>
#include <thread>
#include <algorithm>
#include <random>
#include <cmath>
#include <iostream>
#include <atomic>
#include <mutex>

using json = nlohmann::json;

// ── parseCandidateItems ───────────────────────────────────────────────────────
// Handles old format [int, ...] and new format [{"k":"r","w":wayId}|{"k":"l","w":wayId,"i":laneIdx}]
/* [AI-generated: Claude] */
std::vector<CandidateItem> parseCandidateItems(const std::string& jsonStr) {
    std::vector<CandidateItem> items;
    if (jsonStr.empty()) return items;
    try {
        auto arr = json::parse(jsonStr);
        if (!arr.is_array()) return items;
        for (const auto& el : arr) {
            if (el.is_number_integer()) {
                items.push_back({el.get<int64_t>(), -1});
            } else if (el.is_object()) {
                std::string k = el.value("k", "r");
                int64_t w    = el.value("w", (int64_t)0);
                int     i    = el.value("i", -1);
                items.push_back({w, k == "l" ? i : -1});
            }
        }
    } catch (...) {}
    return items;
}
/* [end AI-generated] */

// ── runInlineSim ──────────────────────────────────────────────────────────────

InlineSimResult runInlineSim(
        const std::string&             osmData,
        const SimConfig&               cfg,
        double                         trafficMult,
        const std::vector<CandidateItem>& closedItems,
        double                         targetDurS,
        uint64_t                       seed,
        const TripLog*                 tripLogIn,
        TripLog*                       tripLogOut) {
    SimulationEngine eng;
    eng.setSeed(seed);
    eng.setGlobalMapParameters(cfg.mapParams);
    eng.loadMapFromOSM(osmData);
    for (const auto& vt : cfg.vehicleTypes) eng.addVehicleType(vt);
    for (const auto& r  : cfg.routes)       eng.defineRoute(r);
    for (auto sr : cfg.spawnRegions) {
        sr.spawnRatePerSec *= trafficMult;
        eng.addSpawnRegion(sr);
    }
    for (const auto& item : closedItems) {
        if (item.isLane()) eng.setLaneClosed(item.wayId, item.laneIdx, true);
        else               eng.setRoadClosed(item.wayId, true);
    }

    if (tripLogOut) { tripLogOut->clear(); eng.setTripLogRecording(tripLogOut); }
    if (tripLogIn)  { eng.setTripLogReplay(tripLogIn); }

    constexpr double STEP = 0.066;
    for (double t = 0.0; t < targetDurS; t += STEP) eng.step(STEP);

    auto m = eng.getMetrics();
    return {
        m.totalSpawned,
        m.totalCompleted,
        eng.getTotalCompletedTravelSum(),
        targetDurS,
        tripLogIn ? eng.getExcludedCount() : 0
    };
}

// ── inlineDANCE ───────────────────────────────────────────────────────────────
// DANC + Excluded destinations term (Approach B).
// excluded (r.excludedCount) agents pay Te each; they are NOT also counted as unserved.
// unserved = agents that neither completed nor were excluded (failed to spawn within sim time).
/* [AI-altered: Claude] - The addition of E term */
double inlineDANCE(const InlineSimResult& r, int64_t S0, double Tp, double Te) {
    double excluded = (double)r.excludedCount;
    double unserved = std::max(0.0, (double)S0 - excluded - (double)r.completed);
    return (r.completedTravelS + unserved * Tp + excluded * Te)
           / std::max((int64_t)1, S0);
}

// ── Shared helpers ────────────────────────────────────────────────────────────

// Pedestrianization objective: lower is better.
// Feasible: minimise (1 - closedCount/N), i.e. maximise closures.
// Infeasible: 1 + 100*(f - threshold), heavy penalty so no infeasible solution
// can beat any feasible one (100 >> 1, so even 1% over threshold costs more
// than closing all N roads is worth).
static double pedestrianObjective(double f, int closedCount, int N, double threshold) {
    if (f > threshold)
        return 1.0 + 100.0 * (f - threshold);
    return 1.0 - (double)closedCount / std::max(1, N);
}

// Evaluate the canonical starting-point fitness using a fixed seed so all three algorithm
// sessions record the same eval-0 convergence value regardless of their session seed.
// For pedestrianization the initial state is always all-open → no sim needed.
// For phased: runs K+1 sims (1 baseline + 1 per phase) with CANON_SEED; these are not
// counted in evalsDone or totalEvals.
static double canonicalInitFit(
        const std::string&                osmData,
        const SimConfig&                  cfg,
        double                            trafficMult,
        const std::vector<CandidateItem>& candidates,
        int N, int K, bool isPhased,
        double simDurS, double threshold, double teMult)
{
    if (!isPhased)
        return pedestrianObjective(1.0, 0, N, threshold);

    constexpr uint64_t CANON_SEED = 0xF1ED5EED00000000ULL;
    const double Tp = simDurS / 2.0;

    auto blRes = runInlineSim(osmData, cfg, trafficMult, {}, simDurS, CANON_SEED, nullptr, nullptr);
    int64_t S0 = std::max((int64_t)1, blRes.spawned);
    double  bl = inlineDANCE(blRes, S0, Tp, 0.0);
    double  Te = (blRes.completed > 0
                  ? blRes.completedTravelS / (double)blRes.completed : Tp) * teMult;

    double totalFit = 0;
    for (int k = 0; k < K; k++) {
        std::vector<CandidateItem> phaseItems;
        for (int i = 0; i < N; i++)
            if (i % K == k) phaseItems.push_back(candidates[i]);
        auto res = runInlineSim(osmData, cfg, trafficMult, phaseItems, simDurS, CANON_SEED, nullptr, nullptr);
        totalFit += inlineDANCE(res, S0, Tp, Te) / std::max(1e-9, bl);
    }
    return totalFit / K;
}

struct EvalCtx {
    const std::string&             osmData;
    const SimConfig&               cfg;
    double                         trafficMult;
    double                         simDurS;
    uint64_t                       seed;
    const std::vector<CandidateItem>& candidates;
    int                            N;
    int                            K;         // phases (phased) or 1 (pedestrianization)
    int64_t                        S0;
    double                         blDANC;
    double                         Tp;
    const std::vector<TripLog>&    tripLogs;  // one TripLog per baseline seed (size = R)
    double                         Te;        // exclusion penalty per agent (avgBlTrip × teMult)
    double                         threshold; // pedestrianization only
    ScenarioType                   scenario;
    int                            runsPerEval;
};

// Evaluate a full phased-construction assignment: each candidate assigned to one phase.
// Runs runsPerEval sims per phase (different seeds) and averages; phaseFits filled per phase.
static double evalPhased(
        const std::vector<int>& asgn,
        EvalCtx& ctx,
        std::shared_ptr<OptimizerState> state,
        std::vector<double>& phaseFitsOut,
        const std::string& iterLabel)
{
    const int N = ctx.N, K = ctx.K, R = ctx.runsPerEval;
    const int totalJobs = K * R;
    phaseFitsOut.assign(K, 1.0);

    std::vector<std::vector<CandidateItem>> phaseItems(K);
    for (int i = 0; i < N; i++) phaseItems[asgn[i]].push_back(ctx.candidates[i]);

    std::atomic<int> workIdx{0};
    std::vector<double> dancSum(K, 0.0);
    std::mutex dancMu;

    // Jobs are indexed [0, K*R): job = k*R + r → phase k, seed-offset r
    auto worker = [&]() {
        while (true) {
            int job = workIdx.fetch_add(1);
            if (job >= totalJobs) break;
            int k = job / R;
            int r = job % R;
            uint64_t runSeed = ctx.seed + (uint64_t)r * 1000003ULL;
            const TripLog* tlog = (r < (int)ctx.tripLogs.size()) ? &ctx.tripLogs[r] : nullptr;

            auto t0  = std::chrono::steady_clock::now();
            auto res = runInlineSim(ctx.osmData, ctx.cfg, ctx.trafficMult,
                                    phaseItems[k], ctx.simDurS, runSeed, tlog, nullptr);
            double ms = std::chrono::duration<double,std::milli>(
                            std::chrono::steady_clock::now() - t0).count();
            double danc = inlineDANCE(res, ctx.S0, ctx.Tp, ctx.Te);

            {
                std::lock_guard<std::mutex> lk(state->mu);
                state->avgSimMs = state->avgSimMs * 0.85 + ms * 0.15;
                state->evalsDone++;
                EvalLogEntry e;
                e.evalNum     = state->evalsDone;
                e.phaseIdx    = k;
                e.fitnessDANC = danc / std::max(1e-9, ctx.blDANC);
                e.durationMs  = ms;
                e.label       = iterLabel + "/p" + std::to_string(k+1)
                               + (R > 1 ? "r" + std::to_string(r+1) : "");
                state->evalLog.push_back(e);
                if (state->evalLog.size() > 30) state->evalLog.pop_front();
            }
            {
                std::lock_guard<std::mutex> lk(dancMu);
                dancSum[k] += danc;
            }
        }
    };

    int nT = std::min(state->maxParallel, totalJobs);
    std::vector<std::thread> workers;
    for (int t = 0; t < nT; t++) workers.emplace_back(worker);
    for (auto& w : workers) w.join();

    double totalFit = 0;
    for (int k = 0; k < K; k++) {
        phaseFitsOut[k] = (dancSum[k] / R) / std::max(1e-9, ctx.blDANC);
        totalFit += phaseFitsOut[k];
    }
    return totalFit / K;
}

// Evaluate a pedestrianization assignment (binary 0/1).
// Runs runsPerEval sims (different seeds) and averages; fills fitOut with averaged DANCE ratio.
static double evalPedestrian(
        const std::vector<int>& asgn,
        EvalCtx& ctx,
        std::shared_ptr<OptimizerState> state,
        double& fitOut,
        int& closedCountOut,
        const std::string& label)
{
    const int R = ctx.runsPerEval;

    closedCountOut = 0;
    std::vector<CandidateItem> closed;
    for (int i = 0; i < ctx.N; i++) {
        if (asgn[i] == 1) { closed.push_back(ctx.candidates[i]); closedCountOut++; }
    }

    std::atomic<int> workIdx{0};
    std::vector<double> dancRuns(R, 0.0);

    auto worker = [&]() {
        while (true) {
            int r = workIdx.fetch_add(1);
            if (r >= R) break;
            uint64_t runSeed = ctx.seed + (uint64_t)r * 1000003ULL;
            const TripLog* tlog = (r < (int)ctx.tripLogs.size()) ? &ctx.tripLogs[r] : nullptr;

            auto t0  = std::chrono::steady_clock::now();
            auto res = runInlineSim(ctx.osmData, ctx.cfg, ctx.trafficMult,
                                    closed, ctx.simDurS, runSeed, tlog, nullptr);
            double ms = std::chrono::duration<double,std::milli>(
                            std::chrono::steady_clock::now() - t0).count();
            dancRuns[r] = inlineDANCE(res, ctx.S0, ctx.Tp, ctx.Te);

            std::lock_guard<std::mutex> lk(state->mu);
            state->avgSimMs = state->avgSimMs * 0.85 + ms * 0.15;
            state->evalsDone++;
            EvalLogEntry e;
            e.evalNum     = state->evalsDone;
            e.phaseIdx    = -2;
            e.fitnessDANC = dancRuns[r] / std::max(1e-9, ctx.blDANC);
            e.durationMs  = ms;
            e.label       = label + (R > 1 ? "/r" + std::to_string(r+1) : "");
            state->evalLog.push_back(e);
            if (state->evalLog.size() > 30) state->evalLog.pop_front();
        }
    };

    int nT = std::min(state->maxParallel, R);
    std::vector<std::thread> workers;
    for (int t = 0; t < nT; t++) workers.emplace_back(worker);
    for (auto& w : workers) w.join();

    double avgDanc = 0;
    for (double d : dancRuns) avgDanc += d;
    avgDanc /= R;

    fitOut = avgDanc / std::max(1e-9, ctx.blDANC);
    return pedestrianObjective(fitOut, closedCountOut, ctx.N, ctx.threshold);
}

// Shared checkpoint helper
/* [AI-prototype: Claude] A different version was intially implemented but then reworked by the author */
static void doCheckpoint(std::shared_ptr<OptimizerState> state,
                          const std::vector<CandidateItem>& candidates) {
    int  iters, evals, total;
    double best, bl;
    std::string asgnJ, pfJ, histJ, candsJ;
    {
        std::lock_guard<std::mutex> lk(state->mu);
        iters = state->itersDone;
        evals = state->evalsDone;
        total = state->totalEvals;
        best  = state->bestFitness;
        bl    = state->baselineDANC;

        json aj = json::array();
        for (int v : state->bestAssignment) aj.push_back(v);
        asgnJ = aj.dump();
        json pf = json::array();
        for (double f : state->bestPhaseFits) pf.push_back(f);
        pfJ = pf.dump();
        json hj = json::array();
        for (const auto& [n, f] : state->convergenceHistory)
            hj.push_back({{"n", n}, {"fit", f}});
        histJ = hj.dump();
        json cj = json::array();
        for (const auto& c : candidates) {
            if (c.isLane()) cj.push_back({{"k","l"},{"w",c.wayId},{"i",c.laneIdx}});
            else            cj.push_back({{"k","r"},{"w",c.wayId}});
        }
        candsJ = cj.dump();
    }
    gMetrics.checkpointOptimizationSession(
        state->sessionId, best, asgnJ, pfJ, iters, evals, bl, total, histJ, candsJ);
}

static void setDone(std::shared_ptr<OptimizerState> state,
                    const std::vector<CandidateItem>& candidates) {
    doCheckpoint(state, candidates);
    std::string asgnJ, pfJ;
    int iters, evals;
    double best;
    {
        std::lock_guard<std::mutex> lk(state->mu);
        state->statusStr = "completed";
        state->allDone.store(true);
        json aj = json::array();
        for (int v : state->bestAssignment) aj.push_back(v);
        asgnJ = aj.dump();
        json pf = json::array();
        for (double f : state->bestPhaseFits) pf.push_back(f);
        pfJ   = pf.dump();
        iters = state->itersDone;
        evals = state->evalsDone;
        best  = state->bestFitness;
    }
    gMetrics.finalizeOptimizationSession(
        state->sessionId, best, asgnJ, pfJ, iters, evals);
    std::cout << "[Optimizer] Session " << state->sessionId.substr(0,8)
              << " (" << state->algorithm << ") done. best=" << best << "\n";
}

// Pause/cancel check helper — blocks while paused, returns true if cancelled
static bool checkPause(std::shared_ptr<OptimizerState> state) {
    while (state->paused.load() && !state->cancelled.load()) {
        { std::lock_guard<std::mutex> lk(state->mu); state->statusStr = "paused"; }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (state->cancelled.load()) return true;
    { std::lock_guard<std::mutex> lk(state->mu); state->statusStr = "running"; }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Hill Climbing
// ─────────────────────────────────────────────────────────────────────────────

void runHC(std::shared_ptr<OptimizerState> state,
           std::string osmData, SimConfig cfg,
           double trafficMult,
           std::vector<CandidateItem> candidates,
           double simDurS, uint64_t seed)
{
    const int  N       = (int)candidates.size();
    const int  K       = state->phaseCount;
    const bool isPhased = (state->scenario == ScenarioType::PhasedConstruction);
    const double Tp    = simDurS / 2.0;
    const int  R       = state->runsPerEval;

    // ── 1. Baseline (averaged over R seeds, records one TripLog per seed) ──────
    int64_t S0          = 1;
    double  blTravelSum = 0;
    int64_t blCompleted = 0;
    double  blDANCSum   = 0;
    std::vector<TripLog> tripLogs(R);
    for (int r = 0; r < R; r++) {
        uint64_t runSeed = seed + (uint64_t)r * 1000003ULL;
        auto t0  = std::chrono::steady_clock::now();
        auto res = runInlineSim(osmData, cfg, trafficMult, {}, simDurS, runSeed,
                                nullptr, &tripLogs[r]);  // record
        double ms = std::chrono::duration<double,std::milli>(
                        std::chrono::steady_clock::now()-t0).count();
        if (r == 0) S0 = std::max((int64_t)1, res.spawned);
        blTravelSum += res.completedTravelS;
        blCompleted += res.completed;
        blDANCSum   += inlineDANCE(res, S0, Tp, 0.0); // excluded=0 for baseline, Te irrelevant
        std::lock_guard<std::mutex> lk(state->mu);
        state->avgSimMs = (r == 0) ? ms : state->avgSimMs * 0.85 + ms * 0.15;
        state->evalsDone++;
        EvalLogEntry e; e.evalNum=-1; e.phaseIdx=-1;
        e.fitnessDANC=1.0; e.durationMs=ms;
        e.label = R > 1 ? "baseline/r" + std::to_string(r+1) : "baseline";
        state->evalLog.push_back(e);
        if (state->evalLog.size() > 30) state->evalLog.pop_front();
    }
    double blDANC = blDANCSum / R;
    double Te     = (blCompleted > 0 ? blTravelSum / (double)blCompleted : Tp) * state->teMult;
    double canonFit = canonicalInitFit(osmData, cfg, trafficMult, candidates, N, K, isPhased,
                                       simDurS, state->threshold, state->teMult);
    {
        std::lock_guard<std::mutex> lk(state->mu);
        state->baselineDANC = blDANC;
        state->convergenceHistory.push_back({0, canonFit});
    }

    EvalCtx ctx{osmData, cfg, trafficMult, simDurS, seed,
                candidates, N, isPhased ? K : 1, S0, blDANC, Tp,
                tripLogs, Te,
                state->threshold, state->scenario, R};

    // ── 2. Initial assignment ────────────────────────────────────────────────
    std::vector<int> asgn(N, 0);
    if (isPhased) for (int i = 0; i < N; i++) asgn[i] = i % K;

    double       curFit;
    std::vector<double> curPhaseFits;
    int          curClosed = 0;

    if (isPhased) {
        {   std::lock_guard<std::mutex> lk(state->mu);
            state->totalEvals = (state->maxIter * N * K + K + R) * R; }
        curFit = evalPhased(asgn, ctx, state, curPhaseFits, "init");
    } else {
        {   std::lock_guard<std::mutex> lk(state->mu);
            state->totalEvals = (state->maxIter * N + R) * R; }
        double rawFit;
        curFit = evalPedestrian(asgn, ctx, state, rawFit, curClosed, "init");
        curPhaseFits = {rawFit};
    }
    {
        std::lock_guard<std::mutex> lk(state->mu);
        state->bestFitness    = curFit;
        state->bestAssignment = asgn;
        state->bestPhaseFits  = curPhaseFits;
        state->bestClosedCount = curClosed;
        state->convergenceHistory.push_back({state->evalsDone, curFit});
    }
    doCheckpoint(state, candidates);

    // ── 3. HC iterations ─────────────────────────────────────────────────────
    for (int iter = 0; iter < state->maxIter && !state->cancelled.load(); iter++) {
        if (checkPause(state)) break;

        double                bestStepFit  = curFit;
        int                   bestMove     = -1;
        int                   bestMoveTo   = -1;
        std::vector<double>   bestNewPhaseFits = curPhaseFits;
        int                   bestNewClosed = curClosed;

        if (isPhased) {
            // Try every (road, new_phase) move; parallelise the evaluations
            // cache stores averaged DANCE ratio (over R seeds) per unique road set
            using RSet = std::vector<int>;
            std::map<RSet, double> cache;
            std::vector<RSet> toRun;

            for (int i = 0; i < N; i++) {
                for (int p = 0; p < K; p++) {
                    if (p == asgn[i]) continue;
                    RSet from, to;
                    for (int j = 0; j < N; j++) {
                        if (asgn[j] == asgn[i] && j != i) from.push_back(j);
                        if (asgn[j] == p)                  to.push_back(j);
                    }
                    to.push_back(i);
                    std::sort(to.begin(), to.end());
                    if (!from.empty()) cache[from] = 0.0;
                    cache[to] = 0.0;
                }
            }
            for (const auto& [rs, _] : cache) toRun.push_back(rs);

            std::atomic<int> workIdx{0};
            std::mutex cacheMu;
            auto worker = [&]() {
                while (true) {
                    int idx = workIdx.fetch_add(1);
                    if (idx >= (int)toRun.size()) break;
                    const RSet& rs = toRun[idx];
                    std::vector<CandidateItem> items;
                    for (int j : rs) items.push_back(candidates[j]);
                    std::string lbl = "hc" + std::to_string(iter+1) + "/" + std::to_string(idx+1);

                    double dancSum = 0;
                    for (int r = 0; r < R; r++) {
                        uint64_t runSeed = seed + (uint64_t)r * 1000003ULL;
                        const TripLog* tlog = (r < (int)tripLogs.size()) ? &tripLogs[r] : nullptr;
                        auto t0 = std::chrono::steady_clock::now();
                        auto res = runInlineSim(osmData, cfg, trafficMult, items, simDurS, runSeed,
                                               tlog, nullptr);
                        double ms = std::chrono::duration<double,std::milli>(
                                        std::chrono::steady_clock::now()-t0).count();
                        dancSum += inlineDANCE(res, S0, Tp, Te);
                        std::lock_guard<std::mutex> lk(state->mu);
                        state->avgSimMs  = state->avgSimMs * 0.85 + ms * 0.15;
                        state->evalsDone++;
                        EvalLogEntry e; e.evalNum=state->evalsDone; e.phaseIdx=-2;
                        e.fitnessDANC=(dancSum/(r+1))/std::max(1e-9,blDANC);
                        e.durationMs=ms;
                        e.label=lbl + (R>1 ? "/r"+std::to_string(r+1) : "");
                        state->evalLog.push_back(e);
                        if (state->evalLog.size() > 30) state->evalLog.pop_front();
                    }
                    std::lock_guard<std::mutex> lk(cacheMu);
                    cache[rs] = (dancSum / R) / std::max(1e-9, blDANC);
                }
            };
            int nT = std::min(state->maxParallel, (int)toRun.size());
            std::vector<std::thread> workers;
            for (int t = 0; t < nT; t++) workers.emplace_back(worker);
            for (auto& w : workers) w.join();

            // Score candidates (cache now holds averaged DANCE ratio directly)
            for (int i = 0; i < N; i++) {
                for (int p = 0; p < K; p++) {
                    if (p == asgn[i]) continue;
                    RSet from, to;
                    for (int j = 0; j < N; j++) {
                        if (asgn[j] == asgn[i] && j != i) from.push_back(j);
                        if (asgn[j] == p)                  to.push_back(j);
                    }
                    to.push_back(i); std::sort(to.begin(), to.end());

                    double fromFit = from.empty() ? 1.0 : cache[from];
                    double toFit   = cache[to];

                    double newTotal = curFit
                        - curPhaseFits[asgn[i]] / K + fromFit / K
                        - curPhaseFits[p]       / K + toFit   / K;

                    if (newTotal < bestStepFit - 1e-9) {
                        bestStepFit = newTotal;
                        bestMove  = i; bestMoveTo = p;
                        bestNewPhaseFits = curPhaseFits;
                        bestNewPhaseFits[asgn[i]] = fromFit;
                        bestNewPhaseFits[p]        = toFit;
                    }
                }
            }
        } else {
            // Pedestrianization HC: steepest-descent on single-road additions.
            // Evaluate closing each open road on top of the current closed set in parallel,
            // then accept the single move with the best pedestrianObjective.
            // The headroom term in pedestrianObjective breaks ties between equally-feasible
            // candidates, preferring the road that leaves the most DANCE headroom for future steps.
            std::atomic<int> workIdx{0};
            std::vector<double> indivDANC(N, 1e18);

            auto worker = [&]() {
                while (!state->cancelled.load()) {
                    int i = workIdx.fetch_add(1);
                    if (i >= N) break;
                    if (asgn[i] == 1) continue;  // already closed — skip
                    std::vector<int> trial = asgn;
                    trial[i] = 1;
                    double rawFit; int cc;
                    evalPedestrian(trial, ctx, state, rawFit, cc,
                                   "hc" + std::to_string(iter+1) + "/t" + std::to_string(i));
                    indivDANC[i] = rawFit;
                }
            };

            int nT = std::min(state->maxParallel, N);
            std::vector<std::thread> workers;
            for (int t = 0; t < nT; t++) workers.emplace_back(worker);
            for (auto& w : workers) w.join();

            if (state->cancelled.load()) break;

            for (int i = 0; i < N; i++) {
                if (asgn[i] == 1 || indivDANC[i] > 1e17) continue;
                double obj = pedestrianObjective(indivDANC[i], curClosed + 1, N, ctx.threshold);
                if (obj < bestStepFit - 1e-9) {
                    bestStepFit      = obj;
                    bestNewPhaseFits = {indivDANC[i]};
                    bestNewClosed    = curClosed + 1;
                    bestMove         = i;
                }
            }
        }

        if (bestMove < 0) break;  // no improvement (phased: local optimum; ped: no feasible roads)

        // Accept the best move.
        if (isPhased) asgn[bestMove] = bestMoveTo;
        else          asgn[bestMove] = 1;
        curFit         = bestStepFit;
        curPhaseFits   = bestNewPhaseFits;
        curClosed      = bestNewClosed;

        {
            std::lock_guard<std::mutex> lk(state->mu);
            state->itersDone = iter + 1;
            if (curFit < state->bestFitness) {
                state->bestFitness     = curFit;
                state->bestAssignment  = asgn;
                state->bestPhaseFits   = curPhaseFits;
                state->bestClosedCount = curClosed;
                state->convergenceHistory.push_back({state->evalsDone, curFit});
            }
        }
        doCheckpoint(state, candidates);
    }
    setDone(state, candidates);
}

// ─────────────────────────────────────────────────────────────────────────────
// Simulated Annealing
// ─────────────────────────────────────────────────────────────────────────────

void runSA(std::shared_ptr<OptimizerState> state,
           std::string osmData, SimConfig cfg,
           double trafficMult,
           std::vector<CandidateItem> candidates,
           double simDurS, uint64_t seed,
           double T0, double Tmin)
{
    const int    N       = (int)candidates.size();
    const int    K       = state->phaseCount;
    const bool   isPhased = (state->scenario == ScenarioType::PhasedConstruction);
    const double Tp      = simDurS / 2.0;
    const int    maxIter = state->maxIter;
    const int    R       = state->runsPerEval;

    // Geometric cooling: alpha such that T reaches Tmin after maxIter steps
    const double alpha = (maxIter > 0 && T0 > Tmin)
        ? std::pow(Tmin / T0, 1.0 / maxIter) : 0.95;

    // ── Baseline (averaged over R seeds, records one TripLog per seed) ──────────
    int64_t S0          = 1;
    double  blTravelSum = 0;
    int64_t blCompleted = 0;
    double  blDANCSum   = 0;
    std::vector<TripLog> tripLogs(R);
    for (int r = 0; r < R; r++) {
        uint64_t runSeed = seed + (uint64_t)r * 1000003ULL;
        auto t0  = std::chrono::steady_clock::now();
        auto res = runInlineSim(osmData, cfg, trafficMult, {}, simDurS, runSeed,
                                nullptr, &tripLogs[r]);  // record
        double ms = std::chrono::duration<double,std::milli>(
                        std::chrono::steady_clock::now()-t0).count();
        if (r == 0) S0 = std::max((int64_t)1, res.spawned);
        blTravelSum += res.completedTravelS;
        blCompleted += res.completed;
        blDANCSum   += inlineDANCE(res, S0, Tp, 0.0);
        std::lock_guard<std::mutex> lk(state->mu);
        state->avgSimMs = (r == 0) ? ms : state->avgSimMs * 0.85 + ms * 0.15;
        state->evalsDone++;
        EvalLogEntry e; e.evalNum=-1; e.phaseIdx=-1;
        e.fitnessDANC=1.0; e.durationMs=ms;
        e.label = R > 1 ? "baseline/r" + std::to_string(r+1) : "baseline";
        state->evalLog.push_back(e);
        if (state->evalLog.size() > 30) state->evalLog.pop_front();
    }
    double blDANC = blDANCSum / R;
    double Te     = (blCompleted > 0 ? blTravelSum / (double)blCompleted : Tp) * state->teMult;
    double canonFit = canonicalInitFit(osmData, cfg, trafficMult, candidates, N, K, isPhased,
                                       simDurS, state->threshold, state->teMult);
    {
        std::lock_guard<std::mutex> lk(state->mu);
        state->baselineDANC = blDANC;
        state->totalEvals   = isPhased
            ? (maxIter * K + K + R) * R
            : (maxIter + 1 + R) * R;
        state->convergenceHistory.push_back({0, canonFit});
    }

    EvalCtx ctx{osmData, cfg, trafficMult, simDurS, seed,
                candidates, N, isPhased ? K : 1, S0, blDANC, Tp,
                tripLogs, Te,
                state->threshold, state->scenario, R};

    // ── Initial state ─────────────────────────────────────────────────────────
    std::mt19937_64 rng(seed ^ 0xDEAD1234ULL);

    std::vector<int> asgn(N, 0);
    if (isPhased) for (int i = 0; i < N; i++) asgn[i] = i % K;

    double       curFit;
    std::vector<double> curPhaseFits;
    int          curClosed = 0;

    if (isPhased) {
        curFit = evalPhased(asgn, ctx, state, curPhaseFits, "sa-init");
    } else {
        double rawFit;
        curFit = evalPedestrian(asgn, ctx, state, rawFit, curClosed, "sa-init");
        curPhaseFits = {rawFit};
    }
    double bestFit = curFit;
    std::vector<int>    bestAsgn    = asgn;
    std::vector<double> bestPhFits  = curPhaseFits;
    int                 bestClosed  = curClosed;

    {
        std::lock_guard<std::mutex> lk(state->mu);
        state->bestFitness     = bestFit;
        state->bestAssignment  = bestAsgn;
        state->bestPhaseFits   = bestPhFits;
        state->bestClosedCount = bestClosed;
        state->convergenceHistory.push_back({state->evalsDone, bestFit});
    }
    doCheckpoint(state, candidates);

    double T = T0;

    // ── SA main loop ──────────────────────────────────────────────────────────
    for (int iter = 0; iter < maxIter && !state->cancelled.load(); iter++) {
        if (checkPause(state)) break;

        // Generate neighbour
        std::vector<int> trial = asgn;
        int i = std::uniform_int_distribution<int>(0, N-1)(rng);
        if (isPhased) {
            int newPhase;
            do { newPhase = std::uniform_int_distribution<int>(0, K-1)(rng); }
            while (newPhase == asgn[i]);
            trial[i] = newPhase;
        } else {
            trial[i] ^= 1;
        }

        // Evaluate
        double       trialFit;
        std::vector<double> trialPhaseFits;
        int          trialClosed = 0;
        std::string  lbl = "sa" + std::to_string(iter+1);

        if (isPhased) {
            trialFit = evalPhased(trial, ctx, state, trialPhaseFits, lbl);
        } else {
            double rawFit;
            trialFit = evalPedestrian(trial, ctx, state, rawFit, trialClosed, lbl);
            trialPhaseFits = {rawFit};
        }

        // Metropolis criterion
        double delta = trialFit - curFit;
        bool accept  = (delta < 0) ||
            (T > 1e-12 && std::uniform_real_distribution<double>(0,1)(rng) < std::exp(-delta / T));

        if (accept) {
            asgn         = trial;
            curFit       = trialFit;
            curPhaseFits = trialPhaseFits;
            curClosed    = trialClosed;
        }

        T *= alpha;

        if (curFit < bestFit) {
            bestFit    = curFit;
            bestAsgn   = asgn;
            bestPhFits = curPhaseFits;
            bestClosed = curClosed;
        }

        {
            std::lock_guard<std::mutex> lk(state->mu);
            state->itersDone = iter + 1;
            if (bestFit < state->bestFitness) {
                state->bestFitness     = bestFit;
                state->bestAssignment  = bestAsgn;
                state->bestPhaseFits   = bestPhFits;
                state->bestClosedCount = bestClosed;
                state->convergenceHistory.push_back({state->evalsDone, bestFit});
            }
        }
        doCheckpoint(state, candidates);
    }
    setDone(state, candidates);
}

// ─────────────────────────────────────────────────────────────────────────────
// Genetic Algorithm
// ─────────────────────────────────────────────────────────────────────────────

void runGA(std::shared_ptr<OptimizerState> state,
           std::string osmData, SimConfig cfg,
           double trafficMult,
           std::vector<CandidateItem> candidates,
           double simDurS, uint64_t seed,
           int popSize, double mutProb, double crossProb)
{
    const int    N        = (int)candidates.size();
    const int    K        = state->phaseCount;
    const bool   isPhased = (state->scenario == ScenarioType::PhasedConstruction);
    const double Tp       = simDurS / 2.0;
    const int    maxGens  = state->maxIter;
    const int    elitism  = std::min(2, popSize);
    const int    R        = state->runsPerEval;
    std::mt19937_64 rng(seed ^ 0xC0FFEE42ULL);

    // ── Baseline (averaged over R seeds, records one TripLog per seed) ──────────
    int64_t S0          = 1;
    double  blTravelSum = 0;
    int64_t blCompleted = 0;
    double  blDANCSum   = 0;
    std::vector<TripLog> tripLogs(R);
    for (int r = 0; r < R; r++) {
        uint64_t runSeed = seed + (uint64_t)r * 1000003ULL;
        auto t0  = std::chrono::steady_clock::now();
        auto res = runInlineSim(osmData, cfg, trafficMult, {}, simDurS, runSeed,
                                nullptr, &tripLogs[r]);  // record
        double ms = std::chrono::duration<double,std::milli>(
                        std::chrono::steady_clock::now()-t0).count();
        if (r == 0) S0 = std::max((int64_t)1, res.spawned);
        blTravelSum += res.completedTravelS;
        blCompleted += res.completed;
        blDANCSum   += inlineDANCE(res, S0, Tp, 0.0);
        std::lock_guard<std::mutex> lk(state->mu);
        state->avgSimMs = (r == 0) ? ms : state->avgSimMs * 0.85 + ms * 0.15;
        state->evalsDone++;
        EvalLogEntry e; e.evalNum=-1; e.phaseIdx=-1;
        e.fitnessDANC=1.0; e.durationMs=ms;
        e.label = R > 1 ? "baseline/r" + std::to_string(r+1) : "baseline";
        state->evalLog.push_back(e);
        if (state->evalLog.size() > 30) state->evalLog.pop_front();
    }
    double blDANC = blDANCSum / R;
    double Te     = (blCompleted > 0 ? blTravelSum / (double)blCompleted : Tp) * state->teMult;
    double canonFit = canonicalInitFit(osmData, cfg, trafficMult, candidates, N, K, isPhased,
                                       simDurS, state->threshold, state->teMult);
    {
        std::lock_guard<std::mutex> lk(state->mu);
        state->baselineDANC = blDANC;
        state->totalEvals   = isPhased
            ? ((maxGens + 1) * popSize * K + R) * R
            : ((maxGens + 1) * popSize + R) * R;
        state->convergenceHistory.push_back({0, canonFit});
    }

    EvalCtx ctx{osmData, cfg, trafficMult, simDurS, seed,
                candidates, N, isPhased ? K : 1, S0, blDANC, Tp,
                tripLogs, Te,
                state->threshold, state->scenario, R};

    // ── Generate initial population ───────────────────────────────────────────
    using Chromosome = std::vector<int>;
    std::vector<Chromosome> pop(popSize, Chromosome(N));
    for (auto& chrom : pop) {
        for (int i = 0; i < N; i++) {
            if (isPhased) chrom[i] = std::uniform_int_distribution<int>(0, K-1)(rng);
            else          chrom[i] = std::uniform_int_distribution<int>(0, 1)(rng);
        }
    }
    // First individual: round-robin (phased) or all-open (pedestrianization) as a sensible seed
    if (isPhased) for (int i = 0; i < N; i++) pop[0][i] = i % K;
    else          std::fill(pop[0].begin(), pop[0].end(), 0);

    // ── Evaluate initial population (parallel) ────────────────────────────────
    std::vector<double>              popFit(popSize, 1e18);
    std::vector<std::vector<double>> popPhaseFits(popSize);
    std::vector<int>                 popClosed(popSize, 0);

    auto evalPop = [&](int start, int end, const std::string& genLabel) {
        std::atomic<int> idx{start};
        auto worker = [&]() {
            while (true) {
                int i = idx.fetch_add(1);
                if (i >= end) break;
                if (isPhased) {
                    popFit[i] = evalPhased(pop[i], ctx, state, popPhaseFits[i],
                                           genLabel + "/c" + std::to_string(i));
                } else {
                    double rawFit;
                    popFit[i] = evalPedestrian(pop[i], ctx, state, rawFit, popClosed[i],
                                               genLabel + "/c" + std::to_string(i));
                    popPhaseFits[i] = {rawFit};
                }
            }
        };
        int nT = std::min(state->maxParallel, end - start);
        std::vector<std::thread> workers;
        for (int t = 0; t < nT; t++) workers.emplace_back(worker);
        for (auto& w : workers) w.join();
    };

    // Evaluate pop[0] (canonical seed: round-robin / all-open) first so GA records a
    // starting convergence point at the same eval count as HC and SA.
    evalPop(0, 1, "gen0");
    double bestFit = popFit[0];
    {
        std::lock_guard<std::mutex> lk(state->mu);
        state->bestFitness     = bestFit;
        state->bestAssignment  = pop[0];
        state->bestPhaseFits   = popPhaseFits[0];
        state->bestClosedCount = popClosed[0];
        state->convergenceHistory.push_back({state->evalsDone, bestFit});
    }
    doCheckpoint(state, candidates);

    // Evaluate the rest of the initial population, then update best if improved.
    evalPop(1, popSize, "gen0");
    {
        int bestIdx = (int)(std::min_element(popFit.begin(), popFit.end()) - popFit.begin());
        if (popFit[bestIdx] < bestFit) {
            bestFit = popFit[bestIdx];
            std::lock_guard<std::mutex> lk(state->mu);
            state->bestFitness     = bestFit;
            state->bestAssignment  = pop[bestIdx];
            state->bestPhaseFits   = popPhaseFits[bestIdx];
            state->bestClosedCount = popClosed[bestIdx];
            state->convergenceHistory.push_back({state->evalsDone, bestFit});
        }
    }
    doCheckpoint(state, candidates);

    // ── GA generational loop ──────────────────────────────────────────────────
    for (int gen = 0; gen < maxGens && !state->cancelled.load(); gen++) {
        if (checkPause(state)) break;

        // Sort by fitness (ascending = better)
        std::vector<int> order(popSize);
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](int a, int b){ return popFit[a] < popFit[b]; });

        // Build new generation
        std::vector<Chromosome> newPop(popSize, Chromosome(N));
        // Elitism
        for (int e = 0; e < elitism && e < popSize; e++) newPop[e] = pop[order[e]];

        // Fill rest via selection + crossover + mutation
        for (int ci = elitism; ci < popSize; ci++) {
            // Binary tournament selection x2
            auto select = [&]() -> const Chromosome& {
                int a = std::uniform_int_distribution<int>(0, popSize-1)(rng);
                int b = std::uniform_int_distribution<int>(0, popSize-1)(rng);
                return pop[popFit[a] < popFit[b] ? a : b];
            };
            const Chromosome& p1 = select();
            const Chromosome& p2 = select();

            Chromosome child = p1;
            // Single-point crossover
            if (std::uniform_real_distribution<double>(0,1)(rng) < crossProb) {
                int cx = std::uniform_int_distribution<int>(1, N-1)(rng);
                for (int j = cx; j < N; j++) child[j] = p2[j];
            }
            // Mutation
            for (int j = 0; j < N; j++) {
                if (std::uniform_real_distribution<double>(0,1)(rng) < mutProb) {
                    if (isPhased) {
                        int np;
                        do { np = std::uniform_int_distribution<int>(0,K-1)(rng); }
                        while (np == child[j]);
                        child[j] = np;
                    } else {
                        child[j] ^= 1;
                    }
                }
            }
            newPop[ci] = std::move(child);
        }

        // Copy elites' fitnesses, re-evaluate the rest
        std::vector<double>              newFit(popSize, 1e18);
        std::vector<std::vector<double>> newPhaseFits(popSize);
        std::vector<int>                 newClosed(popSize, 0);
        for (int e = 0; e < elitism && e < popSize; e++) {
            newFit[e]       = popFit[order[e]];
            newPhaseFits[e] = popPhaseFits[order[e]];
            newClosed[e]    = popClosed[order[e]];
        }

        pop       = std::move(newPop);
        popFit    = std::move(newFit);
        popPhaseFits = std::move(newPhaseFits);
        popClosed = std::move(newClosed);

        evalPop(elitism, popSize, "gen" + std::to_string(gen+1));

        // Update global best
        int bIdx = (int)(std::min_element(popFit.begin(), popFit.end()) - popFit.begin());
        if (popFit[bIdx] < bestFit) {
            bestFit = popFit[bIdx];
            std::lock_guard<std::mutex> lk(state->mu);
            state->bestFitness     = bestFit;
            state->bestAssignment  = pop[bIdx];
            state->bestPhaseFits   = popPhaseFits[bIdx];
            state->bestClosedCount = popClosed[bIdx];
            state->convergenceHistory.push_back({state->evalsDone, bestFit});
        }
        {
            std::lock_guard<std::mutex> lk(state->mu);
            state->itersDone = gen + 1;
        }
        doCheckpoint(state, candidates);
    }
    setDone(state, candidates);
}
