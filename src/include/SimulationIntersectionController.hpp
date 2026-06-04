#pragma once

#include <cstdint>
#include <vector>

#include "SimulationTypes.hpp"
// Forward declaration
class SimulationIntersectionPath;
class SimulationIntersection;

/**
 * @class SimulationIntersectionController
 * @brief Manages conflict-point reservations and traffic-light state for one intersection.
 *
 * Agents call isConflictLocked() before entering, lockConflict() to claim a point,
 * and unlockConflict() (or releaseAllForAgent()) when they exit.
 */
class SimulationIntersectionController {
public:
    SimulationIntersectionController() = default;

    void setParent(SimulationIntersection* parent) { parent_ = parent; }

    // --- Conflict-point API ---

    /** @brief Returns true if conflictId is currently held by any agent other than requestingAgentId. */
    [[nodiscard]] bool isConflictLocked(int conflictId, int64_t requestingAgentId) const;

    void lockConflict(int conflictId, int64_t agentId);    ///< Claim a conflict point on behalf of agentId.
    void unlockConflict(int conflictId, int64_t agentId);  ///< Release a previously claimed conflict point.

    /** @brief Returns the flat (conflictId, agentId) lock list for the debug API. */
    [[nodiscard]] const std::vector<std::pair<int, int64_t>>& getLocks() const { return locks_; }

    /** @brief Release all conflict points held by agentId — called on agent removal or crash recovery. */
    void releaseAllForAgent(int64_t agentId);

    // --- Traffic-light API ---

    /** @brief Advance the signal phase timer by dt seconds. */
    void update(double dt);
    /** @brief Replace the phase schedule; resets the timer to the first phase. */
    void setPhases(const std::vector<TrafficLightPhase>& phases);
    /** @brief Returns the current light state (green/yellow/red) for the given path index. */
    [[nodiscard]] LightState getLightState(int pathIndex) const;

    /** @brief Returns the cached (pathIndex → stateInt) pairs for the debug API. */
    [[nodiscard]] const std::vector<std::pair<int, int>>& getLightDebugState() const { return currentLightStates_; }

private:
    // Flat (conflictId, agentId) pair list. Intersections have at most ~25 conflict IDs;
    // linear scan over a contiguous array beats unordered_map for N < ~50 due to cache locality.
    std::vector<std::pair<int, int64_t>> locks_;
    SimulationIntersection* parent_ = nullptr;  ///< Non-owning back-pointer used to look up path indices.

    // ---- Traffic-light state ----
    std::vector<TrafficLightPhase> phases_;
    int    currentPhaseIdx_ = 0;
    double phaseTimer_      = 0.0;
    bool   isYellow_        = false;
    double yellowDuration_  = 3.0;

    std::vector<std::pair<int, int>> currentLightStates_;  ///< Cached pathIndex → LightState int, rebuilt each phase change.
};