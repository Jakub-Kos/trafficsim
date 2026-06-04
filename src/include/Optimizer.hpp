#pragma once

#include "SimConfig.hpp"
#include "MetricsCollector.hpp"
#include "TripLog.hpp"
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <atomic>
#include <mutex>
#include <memory>

/**
 * @struct CandidateItem
 * @brief A road or specific lane that the optimizer may close.
 *
 * laneIdx == -1 means the entire road; >= 0 targets a single lane index.
 */
struct CandidateItem {
    int64_t wayId   = 0;
    int     laneIdx = -1;   ///< -1 = whole road; >= 0 = specific lane index

    [[nodiscard]] bool isLane() const { return laneIdx >= 0; }

    bool operator==(const CandidateItem& o) const {
        return wayId == o.wayId && laneIdx == o.laneIdx;
    }
    bool operator<(const CandidateItem& o) const {
        if (wayId != o.wayId) return wayId < o.wayId;
        return laneIdx < o.laneIdx;
    }
};

/**
 * @brief Optimizer scenario type.
 *
 * PhasedConstruction partitions the candidate set into K phases and minimises
 * average DANCE ratio. Pedestrianization finds the largest subset that can be
 * closed while keeping the DANCE ratio at or below a threshold.
 */
enum class ScenarioType {
    PhasedConstruction,
    Pedestrianization,
};

inline ScenarioType scenarioFromString(const std::string& s) {
    if (s == "pedestrianization") return ScenarioType::Pedestrianization;
    return ScenarioType::PhasedConstruction;
}
inline std::string scenarioToString(const ScenarioType t) {
    return t == ScenarioType::Pedestrianization ? "pedestrianization" : "phased_construction";
}

/**
 * @struct InlineSimResult
 * @brief Aggregated outcome of a single inline simulation run (no DB writes).
 */
struct InlineSimResult {
    int64_t spawned             = 0;
    int64_t completed           = 0;
    double  completedTravelS    = 0.0;  ///< Sum of actual travel times for completed agents.
    double  simDurS             = 0.0;
    int32_t excludedCount       = 0;    ///< Agents excluded due to closed spawn lane or unreachable destination.
};

/**
 * @struct EvalLogEntry
 * @brief One fitness evaluation record appended to OptimizerState::evalLog.
 */
struct EvalLogEntry {
    int         evalNum;
    int         phaseIdx;    ///< -1 = baseline, -2 = intermediate
    double      fitnessDANC;
    double      durationMs;
    std::string label;
};

/**
 * @struct OptimizerState
 * @brief Live state for one optimization session, shared between the HTTP thread and the algorithm thread.
 *
 * All mutable fields below the atomics are protected by mu.
 */
struct OptimizerState {
    std::string  sessionId;
    std::string  problemId;
    ScenarioType scenario    = ScenarioType::PhasedConstruction;
    int          phaseCount  = 2;
    double       threshold   = 1.15;  ///< Pedestrianization DANCE-ratio constraint.
    int          N           = 0;     ///< Number of candidate items.
    int          maxIter     = 50;
    int          maxParallel = 4;
    int          runsPerEval = 1;
    double       teMult      = 1.0;  ///< Te = avgBaselineTripS × teMult
    std::string  algorithm   = "hill_climbing";

    std::atomic<bool> paused{false};
    std::atomic<bool> cancelled{false};
    std::atomic<bool> allDone{false};

    std::mutex  mu;
    std::string statusStr  = "running";
    int         itersDone  = 0;
    int         evalsDone  = 0;
    int         totalEvals = 0;
    double      baselineDANC = 1.0;
    double      bestFitness  = 1e18;

    // assignment[i] ∈ {0..K-1} for phased; {0=open,1=closed} for pedestrianization
    std::vector<int>    bestAssignment;
    std::vector<double> bestPhaseFits;   ///< Per-phase DANCE ratio for the best solution found.

    double avgSimMs = 5000.0;
    std::deque<EvalLogEntry> evalLog;
    std::vector<std::pair<int,double>> convergenceHistory;

    int bestClosedCount = 0;        ///< Pedestrianization: number of items closed in the best solution.
    std::string candidatesJson;     ///< Serialised CandidateItem array for DB export.
    std::string hyperparamsJson;    ///< Serialised hyperparameters for DB export.
};

// Global session registry (defined in OptimizerRunner.cpp).
extern std::mutex gOptMu;
extern std::map<std::string, std::shared_ptr<OptimizerState>> gOptSessions;

// Globals from main.cpp — read-only by optimizer threads.
extern SimConfig        gCfg;
extern std::string      gCurrentOsmData;
extern std::string      gLoadedProjectName;
extern MetricsCollector gMetrics;

/**
 * @brief Run a single simulation and return aggregated metrics without writing to the DB.
 *
 * @param tripLogOut If non-null, records (spawnLane, destLane) for each successful spawn.
 * @param tripLogIn  If non-null, replays forced destinations and counts excluded agents.
 */
InlineSimResult runInlineSim(
    const std::string&             osmData,
    const SimConfig&               cfg,
    double                         trafficMult,
    const std::vector<CandidateItem>& closedItems,
    double                         targetDurS,
    uint64_t                       seed,
    const TripLog*                 tripLogIn  = nullptr,
    TripLog*                       tripLogOut = nullptr);

/**
 * @brief Compute the DANCE fitness score (DANC extended with an exclusion penalty).
 *
 * Returns average cost per agent: travel time + unserved penalty (Tp) + exclusion penalty (Te).
 * Excluded agents (r.excludedCount) are not also counted as unserved.
 *
 * @param S0 Baseline spawned agent count used to normalise the score.
 * @param Tp Penalty per unserved agent (seconds).
 * @param Te Penalty per excluded agent (typically avgBaselineTripS × teMult).
 */
double inlineDANCE(const InlineSimResult& r, int64_t S0, double Tp, double Te);

/** @brief Parse candidate items from JSON; supports the legacy plain-integer format and the current object format. */
std::vector<CandidateItem> parseCandidateItems(const std::string& jsonStr);

// Algorithm runners — each launches in its own detached thread.
void runHC(std::shared_ptr<OptimizerState> state,
           std::string osmData, SimConfig cfg,
           double trafficMult,
           std::vector<CandidateItem> candidates,
           double simDurS, uint64_t seed);

void runSA(std::shared_ptr<OptimizerState> state,
           std::string osmData, SimConfig cfg,
           double trafficMult,
           std::vector<CandidateItem> candidates,
           double simDurS, uint64_t seed,
           double T0, double Tmin);

void runGA(std::shared_ptr<OptimizerState> state,
           std::string osmData, SimConfig cfg,
           double trafficMult,
           std::vector<CandidateItem> candidates,
           double simDurS, uint64_t seed,
           int popSize, double mutProb, double crossProb);

// HTTP route registration — called once from main().
namespace httplib { class Server; }
void registerOptimizerRoutes(httplib::Server& svr);
void registerSensitivityRoutes(httplib::Server& svr);
