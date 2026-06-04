#pragma once

#include "SimulationTypes.hpp"
#include "SimulationWorld.hpp"
#include "TripLog.hpp"
#include <string>
#include <vector>
#include <memory>
#include <optional>

class MetricsCollector;

/**
 * @class SimulationEngine
 * @brief The main decoupled simulation class (the "backend").
 *
 * Manages the entire lifecycle of the traffic simulation, completely
 * independent of any UI, rendering, or networking. This is the "core"
 * of this project. It maintains the simulation state and exposes
 * a powerful API for setup, modifying, execution, and data query.
 *
 * @par Workflow
 * 1. **Load:** `loadMapFromOSM(...)`
 * 2. **Configure:** `setGlobalMapParameters(...)`
 * 3. **Build:** `initialize()` (This builds the sim graph)
 * 4. **Query & Modify (Optional):**
 * - `getMapDataJSON()` (To get the built map for the UI)
 * - `updateNodePosition(...)`, `updateWayProperties(...)`, etc. (To modify)
 * 5. **Setup Sim:**
 * - `addVehicleType(...)`, `defineRoute(...)`, `addSpawnRegion(...)`
 * 6. **Run:** Call `step(dt)` in a loop.
 * 7. **Query State:** `getVehicleStatesInBounds(...)`, `getMetrics()`, etc.
 */
class SimulationEngine {
public:
    SimulationEngine();
    ~SimulationEngine() noexcept;

    // --- C++ Idioms: Prevent copying to manage pimpl_ pointer ---
    SimulationEngine(const SimulationEngine&) = delete;
    SimulationEngine& operator=(const SimulationEngine&) = delete;

    // --- C++ Idioms: Enable moving ---
    SimulationEngine(SimulationEngine&&) noexcept;
    SimulationEngine& operator=(SimulationEngine&&) noexcept;

    // --- Phase 1: Setup & Initialization ---

    /**
     * @brief Loads the raw map data from an OSM XML string.
     * This only loads the raw nodes and ways; it does *not* build the
     * simulation graph (lanes, intersections) yet.
     *
     * @param osmData A string containing the OSM XML data.
     */
    void loadMapFromOSM(const std::string& osmData);

    /**
     * @brief Sets the *default* parameters used by the `initialize()` step.
     * Call this *after* `loadMapFromOSM` but *before* `initialize`.
     *
     * @param params A GlobalMapParameters struct with default values.
     */
    void setGlobalMapParameters(const GlobalMapParameters& params);

    /**
     * @brief Override runtime-tunable parameters (broadcast rate, parking probability, etc.).
     * Must be called before loadMapFromOSM() for agentPoolCapacity to take effect.
     * Other parameters may be updated at any time.
     */
    void setSimulationTuning(const SimulationTuning& tuning);

    /**
     * @brief Gets the currently set global map parameters.
     *
     * @return The current GlobalMapParameters struct.
     */
    [[nodiscard]] GlobalMapParameters getGlobalMapParameters() const;

    /**
     * @brief Processes the loaded map data and builds the simulation graph.
     * It procedurally generates lanes, intersections, and turn paths using
     * the `GlobalMapParameters`. Must be called *after* `loadMapFromOSM`.
     */
    void initialize();

    // --- Phase 2: Post-Init Map Modifying ("Messy OSM" Fix) ---

    /**
     * @brief Updates the position of a single node *after* `initialize()`.
     * This is a heavy operation. The engine will find all roads, lanes, and
     * intersection paths attached to this node and re-compute their geometry.
     *
     * @param nodeId The OSM ID of the node to move.
     * @param newPosition The new `WorldPosition` for the node.
     */
    void updateNodePosition(int64_t nodeId, const WorldPosition& newPosition);

    /**
     * @brief Updates the properties of a single Way (Road) *after* `initialize()`.
     * This will rebuild the Road, its Lanes, and re-link any attached
     * intersection paths.
     *
     * @param wayId The OSM ID of the Way to update.
     * @param config A `WayConfig` struct. Use 'std::nullopt' for
     * properties you do not wish to change.
     */
    void updateWayProperties(int64_t wayId, const WayConfig& config);

    /**
     * @brief Updates the properties of a single turn path in an intersection.
     * This directly addresses custom "intersection curvature".
     *
     * @param intersectionId The OSM ID of the node for the intersection.
     * @param fromWayId The OSM ID of the "from" Way.
     * @param toWayId The OSM ID of the "to" Way.
     * @param config A `TurnPathConfig` struct. (e.g., to set a new curvature)
     */
    void updateIntersectionTurn(int64_t intersectionId,
                                int64_t fromWayId,
                                int64_t toWayId,
                                const TurnPathConfig& config);


    // --- Phase 3: Simulation Setup (Agents, Routes, Spawns) ---

    /**
     * @brief Defines a new type of vehicle the simulation can create.
     * @param config A `VehicleTypeConfig` struct.
     */
    void addVehicleType(const VehicleTypeConfig& config);

    /**
     * @brief Defines a named, ordered list of Way IDs for agents to follow.
     * @param route A `RouteDefinition` struct.
     */
    void defineRoute(const RouteDefinition& route);

    /**
     * @brief Defines a geographical area that spawns agents.
     * @param config A `SpawnRegionConfig` struct.
     */
    void addSpawnRegion(const SpawnRegionConfig& config);

    /**
     * @brief Defines a scheduled public transport line.
     * @param config A `PublicTransportLineConfig` struct.
     */
    void addPublicTransportLine(const PublicTransportLineConfig& config);


    // --- Phase 4: Simulation Control & Execution ---

    /**
     * @brief Runs one discrete tick of the simulation.
     * This advances all agent logic, car-following, and spawners.
     * Time control (pause/speedup) is handled by the *caller* by
     * modifying the `deltaTime` or skipping calls to `step`.
     *
     * @param deltaTime The amount of simulation time (in seconds) to advance.
     */
    void step(double deltaTime);


    // --- Phase 5: Data-Out (Querying State) ---

    /**
     * @brief Gets the static map data (roads, lanes, intersections) as a JSON.
     * Call this *after* `initialize()` (and after any modifying) to get
     * the final map geometry to send to the web frontend for rendering.
     *
     * @return A JSON string representing the full, static map graph.
     */
    [[nodiscard]] std::string getMapDataJSON() const;
    /** @brief Lightweight [{id,x,y}] array for all intersections. */
    [[nodiscard]] std::string getIntersectionPositionsJSON() const;

    /**
     * @brief Get aggregated metrics for headless analysis.
     * @return A `SimulationMetrics` struct.
     */
    [[nodiscard]] SimulationMetrics getMetrics() const;

    /**
     * @brief Get minimal state *only* for vehicles within a given area.
     * This is the recommended function for live web UI streaming.
     *
     * @param bounds The `BoundingBox` to query.
     * @return A `std::vector` of `VehicleState` structs.
     */
    [[nodiscard]] std::vector<VehicleState> getVehicleStatesInBounds(const BoundingBox& bounds) const;

    /**
     * @brief Get the rich, detailed state for a single tracked vehicle.
     *
     * @param vehicleId The ID of the vehicle to "follow".
     * @return A `std::optional<DetailedVehicleState>`.
     * Check `.has_value()` before accessing.
     */
    [[nodiscard]] std::optional<DetailedVehicleState> getDetailedVehicleState(int64_t vehicleId) const;

    /**
     * @brief Aggregates vehicle data into a heatmap grid.
     *
     * @param bounds The `BoundingBox` to heatmap.
     * @param gridResolutionX Number of cells in the X direction.
     * @param gridResolutionY Number of cells in the Y direction.
     * @return A `std::vector` of `HeatmapCell` structs.
     */
    [[nodiscard]] std::vector<HeatmapCell> getVehicleHeatmap(
        const BoundingBox& bounds,
        int gridResolutionX,
        int gridResolutionY) const;

    /**
     * @brief Per-lane vehicle count, stuck count, and average speed.
     * Useful for road-network congestion heatmaps in the frontend.
     * Only lanes that currently have at least one vehicle are returned.
     */
    [[nodiscard]] std::vector<LaneTrafficStat> getLaneTrafficStats() const;

    /**
     * @brief Returns all parking spot positions, headings, and occupancy as JSON.
     */
    [[nodiscard]] std::string getParkingSpotsJSON() const;


    // --- Phase 6: Runtime Interaction ---

    /**
     * @brief Dynamically tell a specific vehicle to change its route.
     *
     * @param vehicleId The ID of the vehicle to command.
     * @param routeId The ID of the new `RouteDefinition` to assign.
     */
    void setVehicleRoute(int64_t vehicleId, const std::string& routeId);

    /**
     * @brief Closes or opens a road, forcing agents to re-route.
     *
     * @param wayId The OSM ID of the Way to close/open.
     * @param isClosed `true` to close, `false` to re-open.
     */
    void setRoadClosed(int64_t wayId, bool isClosed);
    /** Close or reopen a single lane by wayId and lane index within the road. */
    void setLaneClosed(int64_t wayId, int laneIdx, bool isClosed);

    /**
     * @brief Set the deterministic RNG seed for spawn-timer staggering and route
     * destination selection. Must be called before loadMapFromOSM() and addSpawnRegion().
     */
    void setSeed(uint64_t seed);

    /**
     * @brief API Debug info about the reservations
     *
     * @return List of intersection Conflict point reservations and the reservee
     */
    [[nodiscard]] std::vector<IntersectionStateData> getIntersectionStates() const;

    void setSpawningEnabled(bool e);                                                      ///< Pause or resume all spawn regions at runtime.
    [[nodiscard]] bool isSpawningEnabled() const;
    [[nodiscard]] std::vector<InternalSpawnSystem::RegionInfo> getSpawnRegionInfo() const; ///< Returns per-region spawn stats for the API.
    void scaleSpawnRates(double factor); ///< Multiply every spawn-region rate by factor (benchmark use).

    /** @brief Attach a MetricsCollector and activate run recording. */
    void setMetricsCollector(MetricsCollector* collector, const std::string& runId);

    /** @brief Cumulative spawned vehicle count (for run finalization). */
    [[nodiscard]] int64_t getTotalSpawned() const;

    /** @brief Sum of freeflow estimates over ALL spawned agents (for DANC penalty). */
    [[nodiscard]] double getTotalSpawnedFreeflowSum() const;

    /** @brief Sum of excess travel time for all completed trips (for inline DANC). */
    [[nodiscard]] double getTotalExcessSeconds() const;

    /** @brief Sum of actual travel times for all completed trips (for optimizer DANC). */
    [[nodiscard]] double getTotalCompletedTravelSum() const;
    /** @brief Count destination roads reachable from any non-closed road (legacy Approach A). */
    [[nodiscard]] int getReachableDestinationCount() const;

    /** @brief Enable trip-log recording (baseline run). Pass nullptr to stop. */
    void setTripLogRecording(TripLog* log);
    /** @brief Enable trip-log replay (candidate run). Pass nullptr to stop. */
    void setTripLogReplay(const TripLog* log);
    /** @brief Agents excluded in the current replay run. */
    [[nodiscard]] int32_t getExcludedCount() const;

private:
    // This is the "PIMPL" (Pointer to Implementation) idiom.
    // This is where refactored "City" class will live,
    // hidden from this public header.

    std::unique_ptr<SimulationWorld> pimpl_;
};