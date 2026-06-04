#pragma once
#include <vector>
#include <cstdint>

/**
 * @struct AgentTripRecord
 * @brief Spawn/destination pair captured during a baseline run.
 *
 * Candidate runs replay these records to force identical destinations; agents
 * whose destination becomes unreachable are counted as excluded in the DANCE score.
 */
struct AgentTripRecord {
    int64_t spawnLaneId; ///< Lane ID (wayId × 1000 + laneIdx) where the agent spawned.
    int64_t destLaneId;  ///< Lane ID of the route's final lane.
};

/** @brief Ordered sequence of trip records from one baseline simulation run. */
using TripLog = std::vector<AgentTripRecord>;