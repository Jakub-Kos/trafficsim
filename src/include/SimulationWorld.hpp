#pragma once

#include "SimulationTypes.hpp"
#include "SimulationAgent.hpp"   // full type required — agents stored by value
#include "TripLog.hpp"
#include <memory>
#include <string>
#include <deque>
#include <unordered_map>
#include <vector>
class SimulationRoad;
class SimulationLane;
class SimulationIntersection;
class SimulationNode;
#include "InternalSpawnSystem.hpp"
#include "InternalRoutePlanner.hpp"
class ParkingSystem;
class MetricsCollector;

/**
 * @class SimulationWorld
 * @brief The core internal implementation of the simulation.
 *
 * This class owns all simulation state, including the road network,
 * all agents, and all helper systems (spawning, routing). It is a
 * direct, decoupled replacement for the original 'City' class.
 * It is managed by the 'SimulationEngine' and is not exposed publicly.
 */
class SimulationWorld {
public:
    SimulationWorld();
    ~SimulationWorld();

    // Non-copyable
    SimulationWorld(const SimulationWorld&) = delete;
    SimulationWorld& operator=(const SimulationWorld&) = delete;

    // --- API implementation (mirrors SimulationEngine's public surface) ---

    // Phase 1: Setup & Initialization
    void setGlobalMapParameters(const GlobalMapParameters& params);
    [[nodiscard]] GlobalMapParameters getGlobalMapParameters() const;
    void initialize(const std::string& mapJsonData);

    // Phase 2: Post-Init Map Tinkering
    void updateNodePosition(int64_t nodeId, const WorldPosition& newPosition);
    void updateWayProperties(int64_t wayId, const WayConfig& config);
    void updateIntersectionTurn(int64_t intersectionId,
                                int64_t fromWayId,
                                int64_t toWayId,
                                const TurnPathConfig& config);

    // Phase 3: Simulation Setup
    void addVehicleType(const VehicleTypeConfig& config);
    void defineRoute(const RouteDefinition& route);
    void addSpawnRegion(const SpawnRegionConfig& config);
    void scaleSpawnRates(double factor);
    void addPublicTransportLine(const PublicTransportLineConfig& config);

    // Phase 4: Simulation Control & Execution
    void step(double deltaTime);

    // Phase 5: Data-Out (Querying State)
    [[nodiscard]] std::string getMapDataJSON() const;
    /** @brief Lightweight: returns [{id,x,y}] for all intersections (no conflict/path data). */
    [[nodiscard]] std::string getIntersectionPositionsJSON() const;
    [[nodiscard]] SimulationMetrics getMetrics() const;
    [[nodiscard]] std::vector<VehicleState> getVehicleStatesInBounds(
        const BoundingBox& bounds) const;
    [[nodiscard]] std::optional<DetailedVehicleState> getDetailedVehicleState(
        int64_t vehicleId) const;
    [[nodiscard]] std::vector<HeatmapCell> getVehicleHeatmap(
        const BoundingBox& bounds,
        int gridResolutionX,
        int gridResolutionY) const;

    [[nodiscard]] std::vector<LaneTrafficStat> getLaneTrafficStats() const;

    [[nodiscard]] std::string getParkingSpotsJSON() const;

    // Phase 6: Runtime Interaction
    void setVehicleRoute(int64_t vehicleId, const std::string& routeId);
    void setRoadClosed(int64_t wayId, bool isClosed);
    void setLaneClosed(int64_t wayId, int laneIdx, bool isClosed);

    /** @brief Set deterministic RNG seed for spawn-timer staggering and route destination
     *  selection. Must be called before initialize() / addSpawnRegion(). */
    void setSeed(uint64_t seed);

    /** @brief Returns true if there is enough gap at the start of lane to place a new agent. */
    [[nodiscard]] bool canSpawn(SimulationLane* lane, double minGap) const;

    /** @brief Sum of freeflow estimates over ALL spawned agents (for DANC penalty). */
    [[nodiscard]] double getTotalSpawnedFreeflowSum() const { return totalSpawnedFreeflowSum_; }
    /** @brief Sum of excess travel time for all completed trips (for inline DANC). */
    [[nodiscard]] double getTotalExcessSeconds() const { return totalExcessSeconds_; }
    /** @brief Sum of actual travel times for all completed trips (for optimizer DANC). */
    [[nodiscard]] double getTotalCompletedTravelSum() const { return totalCompletedTravelSum_; }
    /** @brief Count destination roads reachable from any non-closed road (legacy Approach A). */
    [[nodiscard]] int getReachableDestinationCount() const {
        return routePlanner_ ? routePlanner_->countReachableDestinations() : 0;
    }

    // --- Trip-log recording / replay ---
    /** @brief Enable trip-log recording: every successful spawn appends {spawnLaneId, destLaneId}. */
    void setTripLogRecording(TripLog* log);
    /** @brief Enable trip-log replay: forced destinations; excluded agents counted in getExcludedCount(). */
    void setTripLogReplay(const TripLog* log);
    /** @brief Agents excluded during the current replay run (closed spawn lane or unreachable dest). */
    [[nodiscard]] int32_t getExcludedCount() const { return excludedCount_; }

    void setSpawningEnabled(bool e);
    [[nodiscard]] bool isSpawningEnabled() const;
    [[nodiscard]] std::vector<InternalSpawnSystem::RegionInfo> getSpawnRegionInfo() const;

    /**
     * @brief Attach a MetricsCollector and activate run recording.
     * Pass nullptr + empty string to detach (stops recording without clearing collector).
     */
    void setMetricsCollector(MetricsCollector* collector, const std::string& runId);

    /**
     * @brief Apply runtime-configurable tuning parameters.
     * agentPoolCapacity only takes effect if called before initialize().
     */
    void setSimulationTuning(const SimulationTuning& t);

    /** @brief Place a new agent at the start of startLane and register it in the pool. */
    void spawnNewAgent(SimulationLane* startLane, int64_t agentId);

    /** @brief Return all lanes whose start point falls inside bounds; used by InternalSpawnSystem. */
    [[nodiscard]] std::vector<SimulationLane*> getStartLanesInBounds(const BoundingBox& bounds) const;

    /** @brief Return conflict-point and traffic-light state for all intersections (debug API). */
    [[nodiscard]] std::vector<IntersectionStateData> getIntersectionStates() const;

private:
    // --- Simulation State ---

    // Map Components
    std::unordered_map<int64_t, std::unique_ptr<SimulationNode>> nodes_;
    std::unordered_map<int64_t, std::unique_ptr<SimulationRoad>> roads_;
    std::vector<std::unique_ptr<SimulationIntersection>> intersections_;
    std::vector<SpawnRegionConfig> spawnRegionConfigs_;
    std::string rawOsmDataString_;
    std::string buildingsDataString_;
    std::string waysDataString_;
    std::string nodesDataString_;

    // Dynamic Components — flat pool; agents stored by value for cache-friendly iteration.
    // Pre-reserved to agentPoolCapacity_ so addresses are stable and all raw pointers
    // (lane queues, intrusive linked list) remain valid for the lifetime of the simulation.
    std::size_t agentPoolCapacity_ = 16384;  ///< Set via setSimulationTuning() before initialize().
    std::vector<SimulationAgent> agents_;

    // Configuration & Systems
    GlobalMapParameters globalParameters_{};
    std::unordered_map<std::string, VehicleTypeConfig> vehicleTypes_;
    std::unordered_map<std::string, RouteDefinition> definedRoutes_;

    // Helper Systems
    std::unique_ptr<InternalSpawnSystem> spawner_;
    std::unique_ptr<InternalRoutePlanner> routePlanner_;
    std::unique_ptr<ParkingSystem> parkingSystem_;

    // --- Parking seeding (spread across multiple steps to avoid a Dijkstra spike on tick 1) ---
    bool hasSeededParking_   = false; ///< True once the entire seed queue has been drained.
    bool seedingStarted_     = false; ///< True after the seed queue has been built.
    std::vector<int>  seedQueue_;     ///< Spot indices (into ParkingSystem::getSpots()) to seed.
    size_t            seedCursor_  = 0; ///< Next index in seedQueue_ to process.
    static constexpr int SEED_BATCH_PER_STEP = 300; ///< Max agents seeded per step.

    // --- Road-speed tracking for congestion-aware routing ---
    double speedUpdateTimer_ = 0.0;  ///< Accumulates deltaTime; triggers speed update at interval.
    double speedUpdateInterval_ = 5.0; ///< Seconds between route-cost updates. Settable via setSimulationTuning().

    // --- Metrics Tracking ---
    double   simTime_                  = 0.0;
    int64_t  totalSpawned_             = 0;
    int64_t  totalCompleted_           = 0;
    double   totalSpawnedFreeflowSum_  = 0.0; ///< Σ freeflow estimate over all spawned agents.
    double   totalExcessSeconds_       = 0.0; ///< Σ max(0, actual − freeflow) for completed trips.
    double   totalCompletedTravelSum_  = 0.0; ///< Σ actual travel time for completed trips.
    std::deque<double> completionTimestamps_;  ///< Sim times of completions (rolling 60 s window).

    struct SpawnRecord {
        double spawnTime        = 0.0;
        double freeflowEstimate = 0.0; ///< Σ(lane_length / speed_limit) at spawn time.
    };
    std::unordered_map<int64_t, SpawnRecord> spawnTimes_; ///< agentId → spawn record.

    double   avgTravelTime_       = 0.0; ///< Welford running mean of completed trip durations.
    int64_t  completedWithTimes_  = 0;   ///< Sample count used by the running mean.

    // --- Trip-log state ---
    TripLog* tripLogOut_ = nullptr;  ///< Non-null during recording mode.
    std::unordered_map<int64_t, SimulationLane*> laneById_;  ///< Built once per replay setup.
    std::unordered_map<int64_t, std::deque<int64_t>> replayDestByLane_;  ///< Per-lane forced dests.
    int32_t excludedCount_ = 0;  ///< Agents excluded in current replay run.

    // --- Deterministic RNG seed ---
    uint64_t simSeed_ = 12345; ///< Seed for InternalRoutePlanner + InternalSpawnSystem

    // --- Analytics DB ---
    MetricsCollector* metricsCollector_ = nullptr;
    std::string       activeRunId_;
    double            isectSnapshotTimer_ = 0.0;   ///< Accumulates sim time; fires a DB snapshot at isectSnapshotInterval_.
    double isectSnapshotInterval_   = 60.0;  ///< Sim-seconds between DB intersection snapshots. Settable via setSimulationTuning().
    double routeCacheTtl_           = 120.0; ///< Route cache TTL in sim-seconds. Settable via setSimulationTuning().
    double parkingDestProbability_  = 0.35;  ///< Fraction of agents that target a parking destination. Settable via setSimulationTuning().

    // --- Post-init tinkering support ---
    /// turn:lanes restriction map preserved from buildGraph for incremental path rebuilds.
    std::unordered_map<int64_t, std::vector<TurnDirection>> laneTurnAllowed_;
    /// Live lane graph kept in sync with intersection paths (used by tinkering + route planner update).
    InternalRoutePlanner::LaneGraph laneGraph_;

    /**
     * @brief Re-computes all geometry for a road and its intersections.
     * Called by the "tinkering" API.
     */
    void rebuildRoadGeometry(SimulationRoad* road);

    /**
     * @brief Re-build all turning paths for one intersection using the stored laneTurnAllowed_ map.
     * Updates laneGraph_ in place and calls intersection->computeConflictPoints().
     * Safe to call with live agents on OTHER intersections; do NOT call if agents are currently
     * traversing this intersection's paths.
     */
    void rebuildIntersectionPaths(SimulationIntersection* intersection);

    // --- Agent spatial grid (lazy: rebuilt only when a bounded query actually needs it) ---
    static constexpr double AGENT_GRID_CELL = 50.0; ///< Grid cell side length in metres.
    static uint64_t agentCellKey(int cx, int cy) noexcept {
        return (uint64_t)(uint32_t)cx | ((uint64_t)(uint32_t)cy << 32);
    }
    static int agentToCell(double coord) noexcept {
        return (int)std::floor(coord / AGENT_GRID_CELL);
    }
    mutable std::unordered_map<uint64_t, std::vector<SimulationAgent*>> agentSpatialGrid_;
    void rebuildAgentGrid() const;

    /** @brief Return all agents whose position falls within bounds, using the spatial grid. */
    std::vector<SimulationAgent*> getAgentsInBounds(const BoundingBox& bounds) const;

    /**
     * @brief Removes agents from the simulation that have completed their route.
     * Uses swap-and-pop to keep the vector dense; fixes up all raw pointers after each swap.
     */
    void purgeCompletedAgents();

    /**
     * @brief Fix all raw pointers that pointed to oldPtr after a swap-and-pop.
     * Updates the intrusive linked-list neighbors and the owning queue's head/tail.
     */
    void fixMovedAgent(SimulationAgent* oldPtr, SimulationAgent* newPtr);

    /** @brief Sample a random fraction of parking spots into seedQueue_; called once at init. */
    void buildSeedQueue(float fraction);
    /** @brief Spawn up to batchSize pre-parked agents from seedQueue_; spread across multiple steps. */
    void drainSeedQueue(int batchSize);
};