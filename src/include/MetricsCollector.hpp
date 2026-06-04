#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <cstdint>

// Forward-declare PGconn so callers don't need libpq-fe.h
struct pg_conn;
typedef struct pg_conn PGconn;

/**
 * @brief Pending trip record held in memory until batch-flushed.
 */
struct PendingTrip {
    std::string runId;
    int64_t     agentId;
    double      spawnTimeS;
    double      completeTimeS;
    double      actualTravelTimeS;
    double      freeflowEstimateS;
    double      excessTimeS;
    int         rerouteCount;
};

/**
 * @class MetricsCollector
 * @brief Writes per-run and per-trip analytics to PostgreSQL.
 *
 * All public methods are thread-safe (protected by an internal mutex).
 * The collector connects lazily and degrades gracefully: if Postgres is
 * unreachable, all methods become no-ops and the simulation continues normally.
 *
 * Connection string is read from the POSTGRES_URL environment variable.
 * Default: "postgresql://trafficsim:trafficsim@localhost:5432/trafficsim"
 */
class MetricsCollector {
public:
    MetricsCollector();
    ~MetricsCollector();

    // Non-copyable, non-movable
    MetricsCollector(const MetricsCollector&) = delete;
    MetricsCollector& operator=(const MetricsCollector&) = delete;

    /** @brief Returns true if the DB connection is open and usable. */
    [[nodiscard]] bool isConnected() const;

    /**
     * @brief Create a top-level Analysis group.
     * @return UUID string for the new analysis, or empty string on failure.
     *
     * @param seed  Shared RNG seed stored with the analysis; all simulations within
     *              this analysis use the same seed for reproducible demand.
     */
    std::string createAnalysis(const std::string& projectName,
                               const std::string& name,
                               uint64_t seed);

    /**
     * @brief Return all analyses for a project as a JSON array, newest first.
     * Each element contains nested "sessions" array with summary data.
     */
    std::string queryAnalysesForProject(const std::string& projectName);

    /**
     * @brief Delete an analysis and all nested sessions/runs/trips (manual cascade).
     */
    void deleteAnalysis(const std::string& analysisId);

    /**
     * @brief Return all sessions for an analysis as a JSON array (for comparison view).
     */
    std::string querySessionsForAnalysis(const std::string& analysisId);

    /**
     * @brief Return the RNG seed stored for an analysis, or 0 on failure.
     */
    uint64_t queryAnalysisSeed(const std::string& analysisId);

    /**
     * @brief Return top hotspot intersections for a session (aggregated from all runs).
     * Each element: { intersectionId, conflictCount }
     */
    std::string queryHotspotsForSession(const std::string& sessionId);

    /**
     * @brief Return a trip sample for histogram display (from all completed runs in session).
     * Each element: { actualTravelTimeSec, excessTimeSec, rerouteCount }
     */
    std::string queryTripsForSession(const std::string& sessionId);

    /**
     * @brief Create a new analysis_sessions row and return its UUID.
     *
     * @param projectName       Matches the project directory name.
     * @param name              Human-readable label for this simulation.
     * @param trafficMode       "low" | "medium" | "high"
     * @param runCount          Number of parallel runs to average.
     * @param simDurationS      Target sim duration per run (seconds).
     * @param closedRoadIds     JSON array string of closed OSM way IDs.
     * @param isBaseline        True if this is the reference simulation.
     * @param analysisId        UUID of parent analysis (empty = standalone).
     * @param baselineSessionId UUID of the baseline session for fitness comparison.
     * @return UUID string for the new session, or empty string on failure.
     */
    std::string createSession(const std::string& projectName,
                              const std::string& name,
                              const std::string& trafficMode,
                              int runCount,
                              double simDurationS,
                              const std::string& closedRoadIds,
                              bool isBaseline = false,
                              const std::string& analysisId = "",
                              const std::string& baselineSessionId = "");

    /**
     * @brief Average all completed runs in the session, write summary, mark completed.
     */
    void finalizeSession(const std::string& sessionId);

    /**
     * @brief Mark a session as failed.
     */
    void failSession(const std::string& sessionId);

    /**
     * @brief Delete a session and all linked runs/trips/snapshots (CASCADE).
     */
    void deleteSession(const std::string& sessionId);

    /**
     * @brief Mark all sessions and runs still in 'running' state as 'failed'.
     * Call once on server startup to clean up leftovers from a previous crash.
     */
    void cleanupStaleRuns();

    /**
     * @brief Return all sessions for a project as a JSON array string, newest first.
     */
    std::string querySessionsForProject(const std::string& projectName);

    /**
     * @brief Create a new simulation_runs row and return its UUID.
     *
     * @param projectName   Matches the project directory name.
     * @param runType       "baseline" | "scenario" | "optimization"
     * @param label         Human-readable label for this run.
     * @param closedRoadIds JSON array string of OSM way IDs, e.g. "[123, 456]"
     * @param baselineRunId UUID of the baseline run to compare against (empty = none).
     * @param sessionId     UUID of the parent analysis session (empty = standalone run).
     * @return UUID string for the new run, or empty string on failure.
     */
    std::string startRun(const std::string& projectName,
                         const std::string& runType,
                         const std::string& label,
                         const std::string& closedRoadIds,
                         const std::string& baselineRunId = "",
                         const std::string& sessionId = "");

    /**
     * @brief Queue one trip for batch insertion into agent_trips.
     *
     * Trips are flushed automatically every BATCH_SIZE entries.
     * Call flush() before finalizeRun() to ensure nothing is lost.
     */
    void recordTrip(const std::string& runId,
                    int64_t agentId,
                    double spawnTimeS,
                    double completeTimeS,
                    double actualTravelTimeS,
                    double freeflowEstimateS,
                    int rerouteCount);

    /**
     * @brief Record one intersection stress snapshot.
     *
     * Only call this when lockedConflicts >= 2 (caller is responsible for filtering).
     */
    void recordIntersectionSnapshot(const std::string& runId,
                                    double simTimeS,
                                    int64_t intersectionId,
                                    int lockedConflicts);

    /**
     * @brief Flush all pending trip records to the database.
     * Call this before finalizeRun().
     */
    void flush();

    /**
     * @brief Mark a run as completed and compute its DANC fitness score.
     *
     * DANC = (excess_s + unserved * T_penalty) / spawned  [seconds/agent]
     * where T_penalty = max(300, avgFreeflowAllSpawned * 2).
     * fitness_score = DANC_this / DANC_baseline  (only when baseline_run_id is set).
     *
     * @param runId                UUID of the run to finalize.
     * @param simDurationS         Total sim time that elapsed (seconds).
     * @param spawned              Total vehicles spawned during the run.
     * @param spawnedFreeflowSumS  Σ freeflow_estimate over ALL spawned agents (seconds).
     */
    void finalizeRun(const std::string& runId,
                     double simDurationS,
                     int64_t spawned,
                     double spawnedFreeflowSumS);

    /**
     * @brief Mark a run as failed (e.g. on engine error).
     */
    void failRun(const std::string& runId);

    /**
     * @brief Find the most recent completed baseline run for a project.
     * @return run_id UUID string, or empty string if none exists.
     */
    std::string findBaselineRun(const std::string& projectName);

    /**
     * @brief Find the most recent completed baseline session (no closed roads) for a project.
     * @return session_id UUID string, or empty string if none exists.
     */
    std::string findBaselineSession(const std::string& projectName);

    /**
     * @brief Return trip records for a run as a JSON array string.
     *
     * Each element: { agentId, spawnTimeSec, completeTimeSec,
     *                 actualTravelTimeSec, freeflowEstimateSec,
     *                 excessTimeSec, rerouteCount }
     * Returns "[]" if disconnected or no trips.
     */
    std::string queryTripsForRun(const std::string& runId);

    /**
     * @brief Return all runs for a project as a JSON array string.
     *
     * Each element: { runId, runType, label, status, createdAt,
     *                 fitnessScore, totalExcessVehicleHours,
     *                 avgExcessTimeSec, completionRate,
     *                 vehiclesSpawned, vehiclesCompleted,
     *                 closedRoadIds, baselineRunId }
     *
     * Returns "[]" if disconnected or no runs exist.
     */
    std::string queryRunsForProject(const std::string& projectName);

    // Optimizer persistence — problems define the search space (candidate roads,
    // phase count, scenario type); sessions record individual algorithm runs
    // against a problem and accumulate results across iterations.

    /** Persist a new optimization problem definition; returns the new problem UUID. */
    std::string createOptimizationProblem(const std::string& projectName,
                                           const std::string& name,
                                           const std::string& candidateRoadIds,
                                           int phaseCount,
                                           double simDurationS,
                                           const std::string& trafficMode,
                                           const std::string& scenarioType = "phased_construction",
                                           double constraintThreshold = 1.15);

    std::string queryOptimizationProblems(const std::string& projectName);    // all problems for a project (JSON array)
    std::string queryOptimizationProblemById(const std::string& problemId);   // single problem (JSON object)
    std::string queryOptimizationSessionById(const std::string& sessionId);   // single session (JSON object)
    void        deleteOptimizationProblem(const std::string& problemId);      // cascades to sessions

    /** Start a new optimization session for problemId; returns the new session UUID. */
    std::string createOptimizationSession(const std::string& problemId,
                                           const std::string& projectName,
                                           const std::string& algorithm,
                                           const std::string& hyperparamsJson);

    /** Mark a session as completed and store its final results. */
    void finalizeOptimizationSession(const std::string& sessionId,
                                      double bestFitness,
                                      const std::string& bestAssignmentJson,
                                      const std::string& phaseFitnessJson,
                                      int itersDone,
                                      int evalsDone);

    /** Upsert progress mid-run (called per iteration so results survive crashes). */
    void checkpointOptimizationSession(const std::string& sessionId,
                                        double bestFitness,
                                        const std::string& bestAssignmentJson,
                                        const std::string& phaseFitnessJson,
                                        int itersDone,
                                        int evalsDone,
                                        double baselineDANC,
                                        int totalEvals,
                                        const std::string& convergenceHistoryJson,
                                        const std::string& candidateRoadIdsJson);

    /** Returns JSON array of sessions for a problem, newest first. */
    std::string queryOptimizationSessionsForProblem(const std::string& problemId);

private:
    void connect();
    bool execSimple(const std::string& sql);

    PGconn*                  conn_  = nullptr;
    mutable std::mutex       mutex_;
    std::vector<PendingTrip> batch_;

    static constexpr int BATCH_SIZE = 100;
};
