#pragma once

#include "SimulationTypes.hpp"
#include "SimulationAgentQueue.hpp"
#include <vector>


// Forward declarations
class SimulationLane;

/**
 * @class SimulationIntersectionPath
 * @brief A cubic Bézier turning path connecting two lanes inside an intersection.
 *
 * Holds the source and destination lanes, the sampled curve points agents follow,
 * and the conflict-point cache used by the yielding logic every tick.
 */
class SimulationIntersectionPath {
public:
    /**
     * @brief Construct a path between two lanes.
     *
     * Only sets fromLane_, toLane_, and direction_. Call setupPathCurve() afterward
     * to generate the sampled curve points.
     *
     * @param fromLane Pointer to the lane from which the path originates.
     * @param toLane   Pointer to the lane where the path ends.
     */
    SimulationIntersectionPath(SimulationLane* fromLane, SimulationLane* toLane, TurnDirection dir);

    /**
     * @brief Sample a cubic Bézier curve and store the result in pathPoints_.
     *
     * Builds a G2-continuous cubic Bézier from p0 to p2 using entry tangent v_in and
     * exit tangent v_out. Handle lengths scale with endpoint separation so the curve
     * remains proportional regardless of intersection size.
     *
     * @param p0    End of the from-lane centreline (curve start).
     * @param v_in  Unit tangent of the from-lane at p0.
     * @param p2    Start of the to-lane centreline (curve end).
     * @param v_out Unit tangent of the to-lane at p2.
     * @param steps Number of points to sample along the curve.
     */
    void setupPathCurve(const WorldPosition& p0,
                        const WorldPosition& v_in,
                        const WorldPosition& p2,
                        const WorldPosition& v_out,
                        int steps);

    // --- Getters ---
    [[nodiscard]] double                            getPathLength()   const { return pathLength_; }
    [[nodiscard]] SimulationLane*                   getFromLane()     const { return fromLane_; }
    [[nodiscard]] SimulationLane*                   getToLane()       const { return toLane_; }
    [[nodiscard]] const std::vector<WorldPosition>& getPathPoints()   const { return pathPoints_; }
    [[nodiscard]] TurnDirection                     getDirection()    const { return direction_; }

    /** @brief The agent queue for this path — agents register here while traversing. */
    SimulationAgentQueue& getAgentQueue() const { return agentQueue_; }

    // --- Cached conflict data (populated once by SimulationIntersection::computeConflictPoints) ---
    [[nodiscard]] const std::vector<ConflictPoint>& getCachedConflicts() const { return cachedConflicts_; }
    [[nodiscard]] int                               getPathIndex()        const { return pathIndex_; }

    void setCachedConflicts(std::vector<ConflictPoint> v) { cachedConflicts_ = std::move(v); }
    void setPathIndex(int idx) { pathIndex_ = idx; }

private:
    SimulationLane*            fromLane_;        ///< Non-owning; source lane.
    SimulationLane*            toLane_;          ///< Non-owning; destination lane.
    std::vector<WorldPosition> pathPoints_;      ///< Sampled curve points agents follow through the intersection.
    mutable SimulationAgentQueue agentQueue_;    ///< Agents currently traversing this path.
    TurnDirection              direction_;
    double                     pathLength_ = 0.0; ///< Cached arc length of pathPoints_.

    std::vector<ConflictPoint> cachedConflicts_; ///< Conflict points involving this path; populated once, read every tick.
    int                        pathIndex_ = -1;  ///< Index of this path in the parent intersection's paths_ vector.
};