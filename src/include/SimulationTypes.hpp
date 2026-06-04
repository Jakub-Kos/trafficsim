#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>   // For int64_t
#include <limits>
#include <optional>  // For std::optional

// --- Basic data primitives ---

/** @brief 2D position in the simulation's world coordinate system (projected UTM). */
struct WorldPosition {
    double x;
    double y;
};

/** @brief Axis-aligned rectangular area used for viewport queries and spawn regions. */
struct BoundingBox {
    WorldPosition min;
    WorldPosition max;
};

// --- Configuration structs (data-in) ---

/** @brief Physics and sizing parameters for one vehicle type. */
struct VehicleTypeConfig {
    std::string typeId;     ///< e.g. "car_default", "bus_city" TODO: sim uses only one type of vehicle
    double length;          ///< metres
    double maxSpeed;        ///< m/s
    double acceleration;    ///< m/s²
    double deceleration;    ///< m/s²
};

/** @brief A named route as an ordered sequence of OSM way IDs. */
struct RouteDefinition {
    std::string          routeId; ///< e.g. "commute_a", "bus_line_5"
    std::vector<int64_t> wayIds;  ///< OSM way IDs in travel order.
};

/**
 * @brief A geographic box that spawns agents onto valid lanes at a given rate.
 */
struct SpawnRegionConfig {
    std::string regionId;
    BoundingBox bounds;
    double      spawnRatePerSec;   ///< Total vehicles/second across the whole region.
    std::string vehicleTypeId;
    std::string routePolicy;       ///< "random" or a specific routeId.
};

/** @brief A scheduled public-transport line with fixed departure times. */
// TODO: Public Transport is not implemented
struct PublicTransportLineConfig {
    std::string          lineId;
    std::string          vehicleTypeId;
    std::string          routeId;
    std::vector<double>  departureTimes; ///< Seconds from simulation start.
};

/** @brief Map-wide defaults applied during initialize(). */
struct GlobalMapParameters {
    double defaultLaneWidth;               ///< metres
    double defaultSpeedLimit;              ///< m/s
    double defaultIntersectionCurbRadius;  ///< metres
    double defaultTurnCurvatureWeight;     ///< Bézier handle scale [0.1 – 2.0]
};

/**
 * @brief Partial update for a road's properties; unset fields are left unchanged.
 * Used with SimulationEngine::updateWayProperties().
 */
struct WayConfig {
    std::optional<int>    numLanes;
    std::optional<double> speedLimit; ///< m/s
    std::optional<bool>   isOneWay;
};

/**
 * @brief Partial update for a single intersection turn path.
 * Used with SimulationEngine::updateIntersectionTurn().
 */
struct TurnPathConfig {
    std::optional<double> curvatureWeight; ///< Bézier handle scale [0.1 – 2.0]
};

enum class RightOfWay {
    PRIORITY, ///< Major road or straight-ahead movement.
    YIELD,    ///< Minor road or left turn (right-hand traffic).
    STOP      ///< Stop sign.
};

/** @brief A point where two intersection paths cross, used by the yielding logic. */
struct ConflictPoint {
    int           id;
    WorldPosition position;

    const void* pathA; ///< Cast to SimulationIntersectionPath* in .cpp
    const void* pathB;

    double distOnA; ///< Arc distance from the start of pathA to this point.
    double distOnB; ///< Arc distance from the start of pathB to this point.

    /// 0 = first-come-first-served; 1 = pathA has priority; 2 = pathB has priority.
    int priorityRule = 0;
};

// --- State & metric structs (data-out) ---

/** @brief Minimal dynamic state of one vehicle; used for high-frequency WebSocket streaming. */
struct VehicleState {
    int64_t      id;
    std::string  typeId;
    WorldPosition pos;
    double heading;                ///< radians
    double speed;                  ///< m/s
    double acceleration;
    int64_t leaderId   = -1;       ///< -1 = no leader
    double gapToLeader = std::numeric_limits<double>::infinity(); ///< metres
    int64_t laneId     = -1;       ///< -1 when inside an intersection
    bool parked        = false;
    double timeUntilDeparture = -1.0; ///< seconds; -1 if not parked
};

/** @brief A single recorded event in an agent's life history. */
struct AgentHistoryEvent {
    enum class Type {
        Spawned,
        StartedParkingSearch,
        TargetedSpot,
        Parked,
        Departed,
        RouteRegenerated
    };
    Type        type;
    double      simTime;
    std::string detail;
};

/** @brief Rich state for a single tracked vehicle; used by the "follow" panel in the frontend. */
struct DetailedVehicleState {
    int64_t      id;
    std::string  typeId;
    WorldPosition pos;
    double heading;
    double speed;
    double acceleration;

    // State Info
    std::string controllerState;
    std::string currentLaneId;
    int         routeIndex;

    int64_t leaderId   = -1;       ///< -1 = no leader
    double gapToLeader = std::numeric_limits<double>::infinity(); ///< metres
    double timeHeadway = std::numeric_limits<double>::infinity(); ///< gap / speed

    bool    isYielding       = false;
    int64_t yieldingToId     = -1;
    WorldPosition conflictPointPos = {0, 0};

    std::vector<WorldPosition> debugHitbox;

    std::string blockedReason;          ///< Diagnostic string from IntersectionEntryController.
    double pathProgress   = 0.0;        ///< Agent's current pathProgress() value.
    double laneLength     = 0.0;        ///< TrimmedCenterlineLength of current lane (0 if in intersection).
    bool   inIntersection = false;

    std::vector<WorldPosition> routePolyline;         ///< Remaining route as world points.
    bool          isParkingRoute      = false;         ///< True if agent is on a parking mission.
    WorldPosition parkingDest         = {0.0, 0.0};   ///< Centre of the parking search area.
    double        parkingSearchRadius = 0.0;           ///< Search radius (0 if no active search).

    bool   isParked           = false; ///< True if agent is occupying a parking spot.
    double timeUntilDeparture = 0.0;   ///< Seconds until departure (0 if not parked).

    std::vector<AgentHistoryEvent> history; ///< Rolling life-event log, capped at 20 entries.
};

/** @brief Aggregated snapshot metrics returned by SimulationEngine::getMetrics(). */
struct SimulationMetrics {
    int64_t totalVehicles          = 0;
    int64_t vehiclesInQueue        = 0;   ///< Vehicles with speed < 1.0 m/s.
    int64_t vehiclesInIntersection = 0;
    int64_t yieldingVehicles       = 0;
    double  avgSpeed               = 0.0; ///< m/s
    double  maxSpeed               = 0.0; ///< m/s
    double  avgGapToLeader         = 0.0; ///< Mean gap over vehicles that have a leader (m).

    int64_t totalSpawned        = 0;      ///< Cumulative since sim start.
    int64_t totalCompleted      = 0;      ///< Cumulative since sim start.
    double  throughputPerMinute = 0.0;    ///< Completions in last 60 sim-seconds, scaled to /min.
    double  avgTravelTime       = 0.0;    ///< Rolling mean of completed trip durations (s).
    double  totalExcessSeconds  = 0.0;    ///< Σ max(0, actual − freeflow) for completed trips.
};

/** @brief One cell in an aggregated vehicle-density heatmap. */
struct HeatmapCell {
    BoundingBox bounds;
    int    vehicleCount;
    double averageSpeed;
};

/** @brief Per-lane traffic statistics for the congestion overlay. */
struct LaneTrafficStat {
    int64_t laneId;
    int     vehicleCount;
    int     stuckCount;   ///< Vehicles with speed < 1 m/s.
    double  averageSpeed; ///< m/s; 0 if no vehicles.
};

/** @brief Conflict-point and traffic-light state for one intersection; used by the debug API. */
struct IntersectionStateData {
    int64_t id;
    std::vector<std::pair<int, int64_t>> lockedConflicts; ///< (conflictId, agentId) pairs.
    std::vector<std::pair<int, int>>     lightStates;     ///< (pathIndex, LightState int) pairs.
};

/** @brief Turn direction of an intersection path, used for routing cost and UI display. */
enum class TurnDirection {
    Straight,
    Left,
    Right,
    UTurn
};

// --- Traffic-light types ---
enum class LightState { Red, Yellow, Green };

/** @brief One phase in a traffic-light cycle. */
struct TrafficLightPhase {
    std::vector<int> greenPathIndices; ///< Path indices that receive a green light in this phase.
    double           duration;         ///< Phase duration in seconds.
};

enum class IntersectionType {
    None,
    TrafficLight,
    Stop,
    Yield,
    Crossing ///< Pedestrian crossing, with or without lights.
};

enum class RoadType {
    Motorway,
    Trunk,
    Primary,
    Secondary,
    Tertiary,
    Residential,
    Service,
    Unknown
};

/** @brief Raw OSM node data as parsed from XML, before SimulationNode objects are constructed. */
struct SimulationNodeData {
    int64_t          id;
    WorldPosition    pos;
    IntersectionType type = IntersectionType::None;
};

enum class ParkingOrientation {
    None,
    Parallel,
    Diagonal,     ///< 45 degrees.
    Perpendicular ///< 90 degrees.
};

/**
 * @brief Runtime-configurable simulation tuning parameters.
 * All values have sensible defaults matching the original hardcoded values.
 * Loaded from simulation.json and passed to the engine before initialization.
 */
struct SimulationTuning {
    int         broadcastFps                  = 30;     ///< WebSocket broadcast rate (fps)
    double      parkingDestinationProbability = 0.35;   ///< Fraction of spawned agents that target a building for parking
    double      speedUpdateIntervalS          = 5.0;    ///< Seconds between congestion-aware routing cost updates
    double      routeCacheTtlS               = 120.0;  ///< Seconds before a Dijkstra route cache entry expires
    double      isectSnapshotIntervalS       = 60.0;   ///< Seconds between intersection congestion DB snapshots
    std::size_t agentPoolCapacity            = 16384;  ///< Pre-allocated agent pool size (stable raw-pointer invariant)
    bool        debugBlockedReason           = true;   ///< Build per-agent blocked-reason diagnostic string (set false for perf)
};