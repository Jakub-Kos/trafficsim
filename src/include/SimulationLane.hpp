#pragma once

#include "SimulationTypes.hpp"
#include "SimulationAgentQueue.hpp"
#include <vector>
#include <string>

// Forward declarations
class SimulationRoad;
class SimulationNode;
class SimulationIntersection;
class SimulationAgent;

/**
 * @class SimulationLane
 * @brief A single one-directional lane: owns its geometry, agent queue, and intersection links.
 *
 * Lanes are offset from the parent road's centreline by lateralOffset and trimmed at
 * each endpoint by curbRadius so agents stop cleanly before the intersection box.
 * The agent queue is ordered back-to-front (tail = last agent, head = first).
 */
class SimulationLane {
public:
    /**
     * @brief Construct a lane and compute its untrimmed centreline.
     *
     * @param id            Unique identifier for this lane.
     * @param parent        Parent road (non-owning).
     * @param isForward     True if the lane travels in the road's node order.
     * @param lateralOffset Offset in metres from the road centreline.
     * @param laneIndex     Position within the road (0 = rightmost).
     * @param curbRadius    Trim radius around the exit intersection (metres).
     */
    SimulationLane(int64_t id,
                   SimulationRoad* parent,
                   bool isForward,
                   double lateralOffset,
                   int laneIndex,
                   double curbRadius);

    // --- Core geometry ---

    /** @brief Offset the parent road's centreline by lateralOffset to build the full (untrimmed) centreline. */
    void computeCenterline();

    /**
     * @brief Trim the centreline to curbRadius from the exit intersection.
     *
     * Must be called after computeCenterline() and after exitIntersection_ is assigned.
     */
    void computeTrimmedCenterline();

    /** @brief Override the natural stop point at the lane exit (used by the road editor). */
    void setCustomStopPosition(WorldPosition pos) { customStopPos_ = pos; }

    // --- Agent management ---
    SimulationAgentQueue& getAgentQueue() { return agentQueue_; }

    /** @brief Insert agent into the queue when it enters this lane. */
    void registerAgent(SimulationAgent* agent);
    /** @brief Remove agent from the queue when it leaves this lane. */
    void deregisterAgent(SimulationAgent* agent);

    // --- Getters ---
    [[nodiscard]] int64_t        getId()                    const { return id_; }
    [[nodiscard]] SimulationRoad* getParentRoad()           const { return parentRoad_; }
    [[nodiscard]] bool           isForward()                const { return isForward_; }
    [[nodiscard]] int            getLaneIndex()             const { return laneIndex_; }
    [[nodiscard]] double         getTrimmedCenterlineLength() const { return trimmedLength_; }
    [[nodiscard]] const std::vector<WorldPosition>& getTrimmedCenterline() const { return trimmedCenterline_; }

    /**
     * @brief Return the neighbouring lane at indexOffset (+1 = left, -1 = right), or nullptr.
     *
     * Only returns a lane that travels in the same direction as this one.
     */
    [[nodiscard]] SimulationLane* getAdjacentLane(int indexOffset) const;

    // --- Graph building ---
    void setStartIntersection(SimulationIntersection* intersection);
    void setExitIntersection(SimulationIntersection* intersection);
    [[nodiscard]] SimulationIntersection* getStartIntersection() const { return startIntersection_; }
    [[nodiscard]] SimulationIntersection* getExitIntersection()  const { return exitIntersection_; }

    // --- Lane closure (optimizer / road editor) ---
    [[nodiscard]] bool isClosed()       const { return isClosed_; }
    void setIsClosed(bool c) { isClosed_ = c; }

private:
    int64_t         id_;             ///<  Unique identifier.
    SimulationRoad* parentRoad_;     ///<  Non-owning; parent road.
    bool            isForward_;      ///<  True if lane travels in the road's node order.
    double          lateralOffset_;  ///<  Offset from road centreline in metres.
    int             laneIndex_;      ///<  Position within the road (0 = rightmost).
    double          curbRadius_;     ///<  Trim radius around the exit intersection in metres.

    std::optional<WorldPosition> customStopPos_;  ///<  Overrides the natural lane-end stop point when set.
    double trimmedLength_ = 0.0;                  ///<  Cached arc length of trimmedCenterline_.

    SimulationIntersection* startIntersection_ = nullptr;  ///<  Non-owning; intersection at the lane start (may be null).
    SimulationIntersection* exitIntersection_  = nullptr;  ///<  Non-owning; intersection the lane leads into (may be null).

    SimulationAgentQueue agentQueue_;  ///<  Agents on this lane, ordered back-to-front.
    bool isClosed_ = false;            ///<  When true, the lane is excluded from spawning and routing.

    // Computed geometry
    std::vector<WorldPosition> centerline_;         ///<  Full (untrimmed) centreline in world coordinates.
    std::vector<WorldPosition> trimmedCenterline_;  ///<  Centreline trimmed to curbRadius; what agents actually follow.
};