#pragma once

#include "SimulationTypes.hpp"
#include "ParkingSystem.hpp"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <random>

// Forward declarations
class SimulationLane;
class SimulationRoad;

/**
 * @class InternalRoutePlanner
 * @brief Provides functionality to generate random routes through a lane graph.
 * Generates routes by walking the lane graph.
 *
 * The RoutePlanner is initialized with a directed graph where each Lane* maps to a
 * vector of adjacent Lane* (outgoing connections). It can then produce a random
 * walk (sequence of lanes) starting from a given lane, with configurable minimum
 * and maximum lengths.
 */
class InternalRoutePlanner {
public:
    using LaneGraph = std::unordered_map<SimulationLane*, std::vector<SimulationLane*>>;

    /**
     * @brief Construct a new RoutePlanner instance with a given lane connectivity graph.
     *
     * Stores the provided graph mapping each Lane pointer to its adjacent lanes.
     * After construction, randomRoute() may be called to generate random walks.
     *
     * @param graph An unordered_map where each key is a Lane* and the value is a vector
     *              of Lane* representing outgoing neighbors. The graph is moved into the planner.
     * @param seed The seed used by the std::mt19937 randomGenerator_
     */
    InternalRoutePlanner(LaneGraph graph, uint64_t seed);

    /**
     * @brief Pre-compute per-lane traversal costs from road type and physical length.
     *
     * Must be called once after construction (and after all lanes have their road type
     * set). Populates the internal cost map used by generateRoute() and also builds the
     * destination pool from all lanes reachable in the graph.
     */
    void buildCostGraph();

    /** @brief Attach a ParkingSystem so that some routes can target parking lanes. */
    void setParkingSystem(ParkingSystem* ps) { parkingSystem_ = ps; }

    /**
     * @brief Generate a route that terminates near a random building centroid.
     *
     * Picks a random building, finds the nearest routable lane to it, then routes
     * there via Dijkstra. Falls back to generateRoute() if routing fails.
     *
     * @param start           Lane the agent spawns/departs from.
     * @param outSpot         If non-null, receives a pre-claimed ParkingSpot near the building.
     * @param outBuildingPos  If non-null, receives the chosen building centroid.
     * @param outBuildingRadius If non-null, receives the recommended search radius (m).
     * @return Ordered lane sequence from start toward the building.
     */
    std::vector<SimulationLane*> generateParkingRoute(SimulationLane* start,
                                                       ParkingSpot** outSpot = nullptr,
                                                       WorldPosition* outBuildingPos = nullptr,
                                                       double* outBuildingRadius = nullptr);

    /**
     * @brief Generate a route from start to a specific target lane using Dijkstra.
     *
     * Unlike generateParkingRoute(), no building selection is performed — the target lane
     * is given directly. Used when an agent has already claimed a parking spot and needs to
     * navigate to its adjacent lane.  No dead-end constraint is applied at the last hop.
     *
     * @param start   The lane the agent is currently on.
     * @param target  The lane to route to.
     * @return Ordered lane sequence, or empty on failure.
     */
    std::vector<SimulationLane*> generateRouteTo(SimulationLane* start, SimulationLane* target);

    /**
     * @brief Find the nearest routable lane (has outgoing graph edges) within maxRadius of pos.
     *
     * Iterates all lanes that are keys in the graph (i.e. have successors — not dead-ends)
     * and returns the one whose midpoint is closest to pos and within maxRadius.
     * Returns nullptr if none found within the radius.
     */
    [[nodiscard]] SimulationLane* findNearestRoutableLane(WorldPosition pos, double maxRadius) const;

    /**
     * @brief Generate a weighted A→B route using Dijkstra.
     *
     * Picks a random destination from the pre-built destination pool, then runs
     * Dijkstra with road-type-weighted edge costs so the resulting path prefers
     * arterial roads (primary/secondary) over residential streets.
     *
     * Falls back to generateRandomRoute() if the cost graph has not been built,
     * the pool is empty, or the destination is unreachable from start.
     *
     * @param start  Lane the agent will spawn on.
     * @return Ordered vector of Lane* from start to destination.
     */
    std::vector<SimulationLane*> generateRoute(SimulationLane* start);

    /**
     * @brief Generate a random route (random walk) starting from a specified lane.
     *
     * Legacy fallback. Used when Dijkstra cannot find a route.
     *
     * @param start     Pointer to the starting Lane for the route.
     * @param minLength Minimum number of lanes in the generated route.
     * @param maxLength Maximum number of lanes in the generated route.
     * @return A vector of Lane* representing the sequence of lanes in the random route.
     */
    std::vector<SimulationLane*> generateRandomRoute(SimulationLane* start,
                                                     int minLength,
                                                     int maxLength);

    /**
     * @brief Update per-road traversal costs based on measured average agent speeds.
     *
     * Roads with average speed well below their speed limit receive higher costs so
     * the next Dijkstra reroute avoids them in favour of less-congested alternatives.
     * Roads absent from the map (no agents currently) retain their base cost.
     * Also flushes the route cache so the next generateRoute() call uses fresh costs.
     *
     * Called periodically by SimulationWorld (every SPEED_UPDATE_INTERVAL seconds).
     *
     * @param avgSpeedMps  Map of road pointer → measured average speed (m/s).
     */
    void updateRoadSpeeds(const std::unordered_map<SimulationRoad*, double>& avgSpeedMps);

    /**
     * @brief Mark a road as closed (infinite cost) or reopen it.
     *
     * Sets roadCost_ for the road to 1e18 when closed or restores the base cost
     * when reopened. Flushes the route path cache so the next generateRoute() call
     * will reroute around the closed road.
     *
     * @param road   Road to close or reopen.
     * @param closed True to close, false to reopen.
     */
    void setRoadClosed(SimulationRoad* road, bool closed);

    /**
     * @brief Count destination roads still reachable from any non-closed road.
     *
     * Runs a multi-source BFS from every non-closed road in the adjacency graph
     * and counts how many entries in roadDestPool_ are visited.  Used to compute
     * the Exclusion (E) term in DANCE: destinations that become unreachable after
     * closures represent demand that is silently rerouted to sub-optimal endpoints.
     *
     * Legacy implementation for Exclusion (E) term, it tends to only affect score of
     * end roads.
     */
    [[nodiscard]] int countReachableDestinations() const;

    /**
     * @brief Replace the entire lane graph and rebuild all derived data.
     * Called by SimulationWorld after tinkering operations that change road geometry
     * or intersection paths. Flushes the route cache.
     */
    void updateGraph(LaneGraph newGraph);

    /**
     * @brief Set the world-coordinate bounding box of the OSM download area.
     *
     * When set, buildCostGraph() will filter dead-end lanes so that only those whose
     * endpoint is within BOUNDARY_TOLERANCE metres of the map edge are kept as valid
     * spawn / destination points. Interior dead-ends (cul-de-sacs) are removed from
     * both pools entirely. Must be called before buildCostGraph().
     */
    void setMapBounds(const double minX, const double minY, const double maxX, const double maxY) {
        mapBoundsSet_ = true;
        mapBoundsMinX_ = minX; mapBoundsMinY_ = minY;
        mapBoundsMaxX_ = maxX; mapBoundsMaxY_ = maxY;
    }

    /**
     * @brief Flush the route path cache immediately.
     * Called after road closures or geometry changes to force Dijkstra re-runs.
     */
    void invalidateCache() { ++cacheGeneration_; }

    /** @brief Set how long (sim-seconds) a route cache entry stays valid before Dijkstra is re-run. */
    void setRouteCacheTtl(const double ttl) { routeCacheTtl_ = ttl; }

    /**
     * @brief Advance the planner's internal simulation clock.
     *
     * Must be called once per world step (with the same deltaTime passed to
     * SimulationWorld::step). Used to expire route cache entries after routeCacheTtl_.
     */
    void advanceSimTime(const double dt) { simTime_ += dt; }

private:
    /**
     * @brief Internal representation of the lane graph.
     *
     * Maps each Lane* to a vector of adjacent Lane* (outgoing edges).
     */
    LaneGraph graph_;
    std::mt19937 randomGenerator_;

    /// Minimum road hops in a Dijkstra path. Routes shorter than this fall back
    /// to a farther dead-end so agents always cross several intersections first.
    static constexpr int MIN_ROUTE_ROADS = 3;

    /// Pre-computed traversal cost per lane (road-type multiplier × physical length).
    std::unordered_map<SimulationLane*, double> laneCost_;

    // ---- Road-level routing data (populated by buildCostGraph) ----

    /// Road-level adjacency: road → all roads directly reachable via intersection paths.
    std::unordered_map<SimulationRoad*, std::vector<SimulationRoad*>> roadAdj_;

    /// Pre-computed traversal cost per road (road-type multiplier × lane length).
    /// This is the live cost used by Dijkstra and is adjusted by updateRoadSpeeds().
    std::unordered_map<SimulationRoad*, double> roadCost_;

    /// Base road cost before any congestion adjustment (road-type multiplier × lane length).
    /// Kept so updateRoadSpeeds() can always compute relative to the uncongested baseline.
    std::unordered_map<SimulationRoad*, double> roadBaseCost_;

    /// Dead-end lanes: reachable in graph but have no outgoing edges (map borders / cul-de-sacs).
    /// Agents route to one of these so they despawn naturally at the map edge.
    std::vector<SimulationLane*> deadEndPool_;

    /// Fast membership test for dead-end lanes.
    std::unordered_set<SimulationLane*> deadEndSet_;

    /// Roads that contain at least one dead-end lane — eligible Dijkstra destinations.
    std::vector<SimulationRoad*> roadDestPool_;

    /// Optional parking system (may be null if parking not configured).
    ParkingSystem* parkingSystem_ = nullptr;

    /// Probability [0,1] that a newly generated route targets a parking lane.
    float parkingDestProbability_ = 0.35f;

    // ---- Road-path cache (populated by generateRoute, flushed by updateRoadSpeeds) ----
    //
    // Keyed by SimulationRoad* (the start road), not SimulationLane*.
    // Dijkstra runs from start->getParentRoad(), so all lanes on the same road produce
    // the same shortest-path tree. Caching at road level means agents spawning from
    // different lanes on the same road all share one Dijkstra computation.
    //
    // Each entry stores up to ROAD_PATH_POOL_SIZE pre-computed road paths (sequences of
    // SimulationRoad*). convertRoadPathToLanes() is called per-spawn with the actual
    // start lane so the lateral position is still correct.

    /// Pre-computed road paths to keep per start road ("round-robined" across spawns).
    static constexpr int ROAD_PATH_POOL_SIZE = 10;
    /// Sim-seconds before a cache entry expires and Dijkstra is re-run. Settable via setRouteCacheTtl().
    double routeCacheTtl_ = 120.0;

    struct RoadPathCacheEntry {
        std::vector<std::vector<SimulationRoad*>> roadPaths; ///< Up to ROAD_PATH_POOL_SIZE paths.
        int      nextIdx    = 0;    ///< Round-robin cursor.
        double   expireAt   = 0.0;  ///< simTime_ at which this entry becomes stale.
        uint64_t generation = 0;    ///< Cache generation when this entry was written.
    };

    std::unordered_map<SimulationRoad*, RoadPathCacheEntry> roadPathCache_;

    /// Incremented by updateRoadSpeeds/setRoadClosed instead of clearing the cache map.
    /// Entries whose generation != cacheGeneration_ are treated as misses and lazily replaced,
    /// avoiding the "clear storm" where every spawn after a speed update triggers a Dijkstra.
    uint64_t cacheGeneration_ = 0;

    // ---- Pre-allocated Dijkstra scratch space (reused across calls to avoid malloc churn) ----
    // unordered_map::clear() retains bucket capacity so re-insertions after clear() are fast.
    mutable std::unordered_map<SimulationRoad*, double>          dijkstraDist_;
    mutable std::unordered_map<SimulationRoad*, SimulationRoad*> dijkstraPrev_;
    mutable std::unordered_map<SimulationRoad*, int>             dijkstraHops_;

    /// Planner's own sim-clock, advanced by advanceSimTime().
    double simTime_ = 0.0;

    // ---- Map boundary filtering (set via setMapBounds before buildCostGraph) ----
    bool   mapBoundsSet_  = false;
    double mapBoundsMinX_ = 0.0, mapBoundsMinY_ = 0.0;
    double mapBoundsMaxX_ = 0.0, mapBoundsMaxY_ = 0.0;

    // ---- Spatial grid for findNearestRoutableLane (built in buildCostGraph) ----
    /// Grid cell side length in world units (metres).
    static constexpr double GRID_CELL = 100.0;

    /// Key from integer cell coordinates.
    static uint64_t cellKey(const int cx, const int cy) noexcept {
        return static_cast<uint64_t>(static_cast<uint32_t>(cx)) | (static_cast<uint64_t>(static_cast<uint32_t>(cy)) << 32);
    }
    /// World coordinate → grid cell index.
    static int toCell(const double coord) noexcept {
        return static_cast<int>(std::floor(coord / GRID_CELL));
    }

    /// Spatial grid: cell key → routable lanes whose midpoint falls in that cell.
    std::unordered_map<uint64_t, std::vector<SimulationLane*>> spatialGrid_;
};