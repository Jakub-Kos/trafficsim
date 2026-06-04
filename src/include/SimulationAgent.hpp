#pragma once

#include "SimulationTypes.hpp"
#include "AgentControllers.hpp"
#include <vector>
#include <string>
#include <limits>
#include <memory>

class SimulationLane;
class SimulationIntersection;
class SimulationIntersectionPath;
class InternalRoutePlanner;
class SimulationIntersectionController;
struct ParkingSpot;
class ParkingSystem;

/// Sequence of lane pointers representing a route
using SimulationRoute = std::vector<SimulationLane*>;

/**
 * @class SimulationAgent
 * @brief Represents a moving element (vehicle/agent) within the simulation.
 *
 * This class follows the Strategy Pattern. It holds the physical state
 * (position, speed) and executes physics updates (IDM car-following).
 * High-level decision making is delegated to the `IAgentController` strategy.
 */
class SimulationAgent {
public:
    /**
     * @brief Construct a new Agent with specified parameters.
     *
     * @param id            Unique 64-bit identifier for this Agent.
     * @param typeId        String identifier for the agent type (e.g., "car").
     * @param route         Sequence of SimulationLane* representing the route.
     * @param desiredSpeed  Target cruise speed (meters per second).
     * @param initialSpeed  Starting speed (meters per second).
     * @param planner       Pointer to a RoutePlanner instance (may be nullptr).
     */
    SimulationAgent(int64_t id,
                    std::string  typeId,
                    const SimulationRoute& route,
                    double desiredSpeed,
                    double initialSpeed,
                    InternalRoutePlanner* planner);

    //--- Core Update ---

    /**
     * @brief Update the Agent's state over a timestep.
     *
     * 1. Calls controller_->updateDecisions() to decide actions.
     * 2. Calculates acceleration using the Intelligent Driver Model (IDM).
     * 3. Integrates physics (position, speed, heading).
     *
     * @param dt Time delta in seconds.
     */
    void update(double dt);

    //--- Controller Interface (API for the "Brain") ---

    /** @brief Replace the current decision controller with a new strategy. */
    void setController(std::unique_ptr<IAgentController> newController);

    /** @brief Sets the waypoints the agent should physically follow. */
    void setWaypoints(const std::vector<WorldPosition>& pts);

    /** @brief Forces the agent to brake to a stop this frame (overrides IDM). */
    void stopVehicle();

    /** @brief Transition logic: enter intersection path (updates queues/state). */
    void enterIntersection(const SimulationIntersectionPath* path);

    /** @brief Transition logic: exit intersection to next lane (updates queues/state). */
    void exitIntersection();

    /** @brief Marks the agent as finished with its route. */
    void markCompleted();

    /**
     * @brief Complete a lane change: swap queues, update route entry and currentLane_.
     *
     * Called by LaneChangingController when the lateral transition is done.
     * Removes the ghost from targetLane, deregisters from fromLane, and inserts
     * this agent at the ghost's queue position on targetLane.
     *
     * @param fromLane   Lane the agent is leaving.
     * @param targetLane Lane the agent is joining.
     * @param ghost      Ghost placeholder currently sitting in targetLane's queue (may be nullptr).
     */
    void completeLaneChange(SimulationLane* fromLane, SimulationLane* targetLane,
                            SimulationAgent* ghost);

    /**
     * @brief Store a privately-owned waypoint buffer and follow it.
     *
     * Used by LaneChangingController to set a lateral-transition path that is
     * owned by the agent itself (not by a lane or intersection path).
     * Resets waypointIndex_ to 0.
     *
     * @param pts Waypoints to store and follow (moved in).
     */
    void setOwnedWaypoints(std::vector<WorldPosition> pts);

    /**
     * @brief Point to an external waypoint buffer, finding the correct start index.
     *
     * Sets waypoints_ to &pts and projects nearPos onto the polyline to find
     * the correct waypointIndex_, so pathProgress() returns the accurate distance
     * from the start of the polyline after a mid-lane handoff (e.g. after a lane
     * change completes).
     *
     * @param pts     External polyline to follow (must outlive this call).
     * @param nearPos World position to project onto the polyline.
     */
    void setWaypointsFrom(const std::vector<WorldPosition>& pts, const WorldPosition& nearPos);

    /**
     * @brief Turn this agent into a passive ghost sentinel.
     *
     * A ghost occupies a slot in a lane queue so other agents respect its space.
     * update() becomes a no-op and pathProgress() returns ghostProgress_.
     *
     * @param ownerId  ID of the real agent that owns this ghost.
     * @param progress Initial path-progress value to report.
     */
    void makeGhost(int64_t ownerId, double progress);

    /** @brief Update the progress value reported by a ghost agent. */
    void setGhostProgress(const double p) { ghostProgress_ = p; }

    /**
     * @brief Fix self-referential waypoints_ after this agent has been move-assigned
     *        from movedFrom (e.g. after a swap-and-pop in the agent pool).
     *
     * If waypoints_ pointed to movedFrom's ownedWaypoints_ buffer, the move left it
     * pointing at the old address. This reseats it to this->ownedWaypoints_.
     * Must be called immediately after move-assignment, before any other use.
     */
    void fixSelfWaypointsAfterMove(const SimulationAgent* movedFrom) {
        if (waypoints_ == &movedFrom->ownedWaypoints_)
            waypoints_ = &ownedWaypoints_;
    }

    /** @brief True if this is a passive ghost sentinel (not a real vehicle). */
    [[nodiscard]] bool isGhost() const { return isGhost_; }

    //--- Predicates (Queries for the "Brain") ---

    /** @brief True if the agent has reached the last waypoint of its current path. */
    [[nodiscard]] bool hasReachedEndOfPath() const;

    /** @brief True if there is another lane in the route after the current one. */
    [[nodiscard]] bool hasNextLaneInRoute() const;

    /** @brief Checks if the *next* lane has physical space for the agent to enter. */
    [[nodiscard]] bool canEnterNextLane() const;

    /** @brief Returns route_[routeIndex_+1], or nullptr if at end of route. */
    [[nodiscard]] SimulationLane* getNextRouteLane() const;

    /** @brief Finds the intersection path connecting current lane to the next route lane.
     *  Falls back to same-road fuzzy match and patches route_[routeIndex_+1] in place. */
    [[nodiscard]] const SimulationIntersectionPath* findPathToNextLane();

    /** @brief Gets the controller for the intersection ahead (requires valid currentLane). */
    SimulationIntersectionController& getIntersectionController();


    //--- Public Getters (Frontend/Simulation) ---

    [[nodiscard]] WorldPosition getPosition() const { return position_; }

    /**
     * @brief Get the Agent's current heading angle.
     * @return The heading angle in radians.
     */
    [[nodiscard]] double getHeading() const { return heading_; }

    /**
     * @brief Get the Agent's current speed in meters per second.
     */
    [[nodiscard]] double getSpeed() const { return speed_; }

    /**
     * @brief Get the Agent's type identifier (e.g., "car").
     */
    [[nodiscard]] const std::string& getTypeId() const { return typeId_; }

    /**
     * @brief Get the Lane currently being followed by the Agent.
     * @return Pointer to the current SimulationLane, or nullptr if none.
     */
    [[nodiscard]] SimulationLane* getCurrentLane() const { return currentLane_; }

    /**
     * @brief Get the IntersectionPath the Agent is currently traversing.
     * @return Pointer to the current SimulationIntersectionPath, or nullptr.
     */
    [[nodiscard]] const SimulationIntersectionPath* getActivePath() const { return activePath_; }

    /**
     * @brief Get the visual length of the Agent.
     * @return Agent's length in meters.
     */
    [[nodiscard]] double getLength() const { return length_; }

    [[nodiscard]] double getWidth() const { return width_; }

    /**
     * @brief Get the unique identifier for this Agent.
     * @return The 64-bit ID.
     */
    [[nodiscard]] int64_t getId() const { return id_; }

    /**
     * @brief Get the pointer to the next Agent in the same lane (behind).
     * @return Pointer to the next Agent, or nullptr if none.
     */
    [[nodiscard]] SimulationAgent* getNextInLane() const { return nextInLane_; }

    /**
     * @brief Get the pointer to the previous Agent in the same lane (in front).
     * @return Pointer to the previous Agent, or nullptr if none.
     */
    [[nodiscard]] SimulationAgent* getPrevInLane() const { return prevInLane_; }

    /**
     * @brief Check if the Agent is currently within an intersection.
     * @return true if inside an intersection; false otherwise.
     */
    [[nodiscard]] bool isInIntersection() const { return inIntersection_; }

    /**
     * @brief Check if the Agent has completed its route.
     * @return true if the route is completed; false otherwise.
     */
    [[nodiscard]] bool isCompleted() const { return completed_; }

    /**
     * @brief Compute the Agent's progress (distance) along the current path.
     * @return Progress distance along the current path (in meters).
     */
    [[nodiscard]] double pathProgress() const;

    // --- Helpers for frontend/diagnostics ---
    /** @return The ID of the vehicle directly ahead in this lane, or -1 if none. */
    [[nodiscard]] int64_t getLeaderId() const;

    /** @return Distance to the vehicle ahead (meters). +inf if no leader. */
    [[nodiscard]] double getGapToLeader() const;

    /** @return The current acceleration value of the agent */
    [[nodiscard]] double getAcceleration() const { return acceleration_; }

    /** @return The index of the current route being followed */
    [[nodiscard]] int getRouteIndex() const { return routeIndex_; }

    /** @return The full route lane sequence. */
    [[nodiscard]] const SimulationRoute& getRoute() const { return route_; }

    // Allows the controller to command a stop at a specific distance relative to the agent
    void setVirtualStopTarget(double dist) { virtualStopGap_ = dist; }

    /** @brief Desired cruise speed, readable by controllers for lane-change decisions. */
    [[nodiscard]] double getDesiredSpeed() const { return desiredSpeed_; }

    //--- API Debug ---
    /** @brief Debug information for the API */
    [[nodiscard]] int isYielding() const { return isYielding_; }
    /** @brief Debug information for the API */
    [[nodiscard]] int64_t getYieldingToId() const { return yieldingToId_; }
    /** @brief Debug information for the API */
    [[nodiscard]] WorldPosition getConflictPointPos() const { return conflictPt_; }

    [[nodiscard]] const std::vector<WorldPosition>& getDebugHitbox() const { return debugHitbox_; }
    void setDebugHitbox(const std::vector<WorldPosition>& poly) { debugHitbox_ = poly; }


    /** @return The state of the Agent, that is the name of controller being used */
    [[nodiscard]] std::string getControllerState() const {
        return controller_ ? controller_->getName() : "None";
    }
    /** @return Raw pointer to the current controller (may be null). */
    [[nodiscard]] const IAgentController* getController() const { return controller_.get(); }
    // --- Parking ---

    /** @brief Returns true if the agent is currently parked. */
    [[nodiscard]] bool isParked() const { return isParked_; }

    /** @brief Returns true if the agent is parked and its departure time has passed. */
    [[nodiscard]] bool isReadyToDepart(double simTime) const {
        return isParked_ && simTime >= departureSimTime_;
    }

    /** @brief Move agent to a parking spot: deregisters from lane, sets position. */
    void park(ParkingSpot* spot, double departureSimTime);

    /**
     * @brief Complete joining the target lane after departing from a parking spot.
     *
     * If ghost is non-null, swaps it out in the queue and links this agent in its place.
     * Otherwise simply registers normally.
     *
     * @param targetLane  Lane to join.
     * @param ghost       Ghost sentinel currently holding space (may be nullptr).
     */
    void unpark(SimulationLane* targetLane, SimulationAgent* ghost);

    /**
     * @brief Instantly teleport the agent from its parked position into the lane queue.
     *
     * Clears parking state, snaps position to lanePoint, joins the lane queue in
     * place of the ghost sentinel, and sets heading to the lane direction.
     * Call regenerateRoute() and setController(LDC) immediately after.
     *
     * @param targetLane  Lane to join.
     * @param ghost       Ghost sentinel to replace (may be nullptr).
     * @param lanePoint   World position on the lane centerline to snap to.
     */
    void teleportUnpark(SimulationLane* targetLane, SimulationAgent* ghost, const WorldPosition& lanePoint);

    [[nodiscard]] double getDepartureSimTime()  const { return departureSimTime_; }  ///< Absolute sim time to leave the parking spot.
    [[nodiscard]] ParkingSpot* getParkedSpot() const { return parkedSpot_; }          ///< Currently occupied spot; nullptr if not parked.
    [[nodiscard]] ParkingSpot* getTargetSpot() const { return targetSpot_; }          ///< Claimed approach target; nullptr if none.
    void setTargetSpot(ParkingSpot* spot) { targetSpot_ = spot; }

    void setAgentSimTime(double t) { agentSimTime_ = t; }  ///< Updated each tick so parking controllers can compare against departure time.
    [[nodiscard]] double getAgentSimTime() const { return agentSimTime_; }

    [[nodiscard]] ParkingSystem* getParkingSystem() const { return parkingSystem_; }
    void setParkingSystem(ParkingSystem* ps) { parkingSystem_ = ps; }

    // Parking search mode: agent is cruising along a parking lane looking for a free spot.
    [[nodiscard]] bool isParkingSearchMode() const { return parkingSearchMode_; }
    void setParkingSearchMode(bool b) { parkingSearchMode_ = b; }
    [[nodiscard]] int getParkingSearchAttempts() const { return parkingSearchAttempts_; }
    void incrementParkingSearchAttempts() { ++parkingSearchAttempts_; }
    /** @brief Expose the route planner to controllers that need to extend the route. */
    [[nodiscard]] InternalRoutePlanner* getPlanner() const { return planner_; }

    // Parking building destination: the building centroid the agent wants to park near.
    [[nodiscard]] WorldPosition getParkingDestBuilding() const { return parkingDestBuilding_; }
    void setParkingDestBuilding(WorldPosition pos) { parkingDestBuilding_ = pos; }
    [[nodiscard]] double getParkingBuildingRadius() const { return parkingBuildingRadius_; }
    void setParkingBuildingRadius(double r) { parkingBuildingRadius_ = r; }

    /** @brief Replace the current route (used when departing from parking). */
    void setRoute(const SimulationRoute& route);

    /**
     * @brief Generate a fresh route from startLane via the route planner.
     *
     * Replaces route_ and resets routeIndex_ to 0. No-op if planner_ is null
     * or startLane is null. Used by ParkingDepartureController after the agent
     * joins the lane so it has a valid destination to drive to.
     */
    void regenerateRoute(SimulationLane* startLane);

    void setCurrentLane(SimulationLane* lane) { currentLane_ = lane; }  ///< Used by ParkingDepartureController after the agent merges onto a lane.

    //--- Setters ---

    /**
     * @brief Set the next Agent in the lane behind this Agent.
     * @param next Pointer to the next Agent (behind). May be nullptr.
     */
    void setNextInLane(SimulationAgent* next) { nextInLane_ = next; }

    /**
    * @brief Set the previous Agent in the lane in front of this Agent.
    * @param prev Pointer to the previous Agent (in front). May be nullptr.
    */
    void setPrevInLane(SimulationAgent* prev) { prevInLane_ = prev; }

    void setParallelSafe(bool v) { parallelSafe_ = v; }  ///< Tagged by SimulationWorld each step before the parallel controller pass.
    bool isParallelSafe() const  { return parallelSafe_; }

    /** @brief Set a free-form debug note (e.g. why IntersectionEntryController is blocked).
     *  Uses a fixed char buffer and plain character copies — no heap alloc, no snprintf. */
    void setBlockedReason(const char* reason, const char* diag = "") {
        char* p   = blockedReasonBuf_;
        char* end = blockedReasonBuf_ + sizeof(blockedReasonBuf_) - 1;
        while (*reason && p < end) *p++ = *reason++;
        while (*diag   && p < end) *p++ = *diag++;
        *p = '\0';
    }
    [[nodiscard]] const char* getBlockedReason() const { return blockedReasonBuf_; }

    // --- Agent history ---
    /** @brief Append a life event; the log is capped at 20 entries (oldest dropped). */
    void addHistoryEvent(AgentHistoryEvent::Type type, double simTime, std::string detail = "") {
        if (history_.size() >= 20) history_.erase(history_.begin());
        history_.push_back({type, simTime, std::move(detail)});
    }
    [[nodiscard]] const std::vector<AgentHistoryEvent>& getHistory() const { return history_; }

    /** @brief Record the current yield state for the debug overlay. */
    void setDebugYieldState(bool isYielding, int64_t toId, WorldPosition pt) {
        isYielding_ = isYielding;
        yieldingToId_ = toId;
        conflictPt_ = pt;
    }
private:
    //--- Constants and initial values ---
    static constexpr double DEFAULT_DESIRED_SPEED = 13.8; ///< Default cruise speed (m/s) (~50 km/h)
    static constexpr double DEFAULT_SPEED = 0.0;          ///< Default initial speed (m/s)
    static constexpr double DEFAULT_ACCELERATION = 0.0;   ///< Default starting acceleration (m/s²)
    static constexpr double DEFAULT_HEADING = 0.0;        ///< Default heading (radians)
    static constexpr double DEFAULT_LENGTH = 4.5;         ///< Default vehicle length (meters)
    static constexpr double DEFAULT_WIDTH = 1.8;          ///< Default vehicle width (meters)

    //--- Constants for basic movement ---
    static constexpr double MIN_GAP = 5.0;              ///< Minimum gap to next agent (meters)
    static constexpr double MAX_ACCEL = 3.0;            ///< Maximum acceleration (m/s²)
    static constexpr double MAX_DECEL = 4.5;            ///< Maximum deceleration (m/s²)
    static constexpr double DESIRED_TIME_HEADWAY = 1.5; ///< Desired time headway (seconds)
    static constexpr double LOOKAHEAD_DIST = 100.0;     ///< Max distance to scan ahead across lanes

    //--- Internal Methods ---
    /** @brief Calculates acceleration using the Intelligent Driver Model (IDM). */
    [[nodiscard]] double calculateCarFollowingAccel();

    /** @brief Moves the agent along the current waypoints_ vector. */
    void moveAlongWaypoints(double dt);

    /**
     * @brief Recursively checks future lanes/paths for a leader if none exists in current lane.
     * @param accumulatedGap Distance from agent to the end of the current component.
     * @param distRemaining  How much further we should scan (e.g. 100m - accumulatedGap).
     * @return The gap to the first found vehicle, or Infinity if path is clear.
     */
    [[nodiscard]] double lookAheadForGap(double accumulatedGap, double distRemaining);

    /** @brief Helper to get total length of current waypoints vector. */
    [[nodiscard]] double getCurrentPathLength() const;

    /** @brief Recompute pathProgress_ and currentPathLength_ from scratch (call when waypoints change). */
    void recomputePathProgress();

    /** @brief Returns a virtual IDM gap distance to enforce yielding at conflict points; infinity if no conflict. */
    double checkIntersectionConflicts(double currentSpeed);

    //--- Basic properties ---
    int64_t id_;            ///< Unique identifier for this Agent
    std::string typeId_;    ///< String identifier for the agent type
    WorldPosition position_;///< Current world coordinates (x, y)

    //--- Movement state ---
    double speed_;         ///< Current speed in meters per second
    double acceleration_;  ///< Current acceleration (negative for deceleration)
    double heading_;       ///< Current heading direction in radians

    //--- Strategy ---
    std::unique_ptr<IAgentController> controller_; ///< The current decision strategy.
    bool forceStop_ = false;                       ///< Flag set by controller to override physics.

    //--- Navigation ---
    int routeIndex_;                      ///< Index into route_ for the next target lane
    SimulationRoute route_;               ///< Sequence of lanes that the Agent will follow
    int waypointIndex_;                   ///< Index into waypoints_ for the next target point
    const std::vector<WorldPosition>* waypoints_; ///< Pointer to a sequence of waypoints (positions)
    SimulationLane* currentLane_;         ///< Pointer to the lane currently being followed
    InternalRoutePlanner* planner_;       ///< Pointer to a global RoutePlanner (may be nullptr)
    bool completed_;                      ///< True if the Agent has reached end of its route

    //--- Intersection state ---
    bool inIntersection_;                     ///< True if the Agent is currently inside an intersection
    SimulationIntersection* activeInt_;       ///< Pointer to the intersection being traversed (if any)
    const SimulationIntersectionPath* activePath_;  ///< Pointer to the current path through an intersection
    std::vector<WorldPosition> debugHitbox_;     ///< Corner points of the vehicle's oriented bounding box, used by the debug overlay.

    //--- Cached progress (updated incrementally in moveAlongWaypoints) ---
    double pathProgress_       = 0.0; ///< Cached distance from path start to current position
    double currentPathLength_  = 0.0; ///< Cached total arc length of current waypoints

    //--- Car-Following Rule ---
    double distanceToNextAgent_;  ///< Distance to the next Agent ahead (meters)
    double desiredSpeed_;         ///< Desired cruise speed (m/s)
    SimulationAgent* nextInLane_; ///< Pointer to the next Agent behind (in the same lane)
    SimulationAgent* prevInLane_; ///< Pointer to the previous Agent ahead (in the same lane)

    //--- Visuals ---
    double length_;  ///< Vehicle length in meters
    double width_;   ///< Vehicle width in meters

    //--- Debug info ---
    bool isYielding_ = false;
    int64_t yieldingToId_ = -1;
    WorldPosition conflictPt_ = {0,0};  ///< Position in the world of the conflict point it waits for

    double virtualStopGap_ = std::numeric_limits<double>::infinity(); ///< IDM virtual obstacle distance set by controllers.

    //--- Debug ---
    char blockedReasonBuf_[256] = {}; ///< Set by IntersectionEntryController — fixed buffer, no heap alloc.

    //--- Parallel update ---
    bool parallelSafe_ = false; ///< Tagged each step by SimulationWorld before the parallel pass.

    //--- Ghost sentinel support ---
    bool   isGhost_       = false; ///< True if this is a passive queue sentinel (no IDM, no movement).
    double ghostProgress_ = 0.0;   ///< Progress value reported by ghost agents instead of computing from waypoints.
    int64_t ghostOwnerId_ = -1;    ///< ID of the real agent that placed this ghost.

    //--- Owned waypoints buffer (used for lane-change transition paths) ---
    std::vector<WorldPosition> ownedWaypoints_; ///< Waypoint buffer owned by this agent (e.g. lane-change path).

    //--- Parking state ---
    bool         isParked_            = false;   ///< True when agent is occupying a parking spot.
    double       departureSimTime_    = 0.0;     ///< Absolute sim time to depart from spot.
    ParkingSpot* parkedSpot_          = nullptr; ///< Non-owning; nullptr if not parked.
    ParkingSpot* targetSpot_          = nullptr; ///< Claimed spot for approach; nullptr if none.
    double       agentSimTime_        = 0.0;     ///< Current absolute sim time, updated each tick.
    ParkingSystem* parkingSystem_     = nullptr; ///< Non-owning pointer to the parking system.
    bool parkingSearchMode_     = false; ///< True while agent is cruising for a free parking spot.
    int  parkingSearchAttempts_ = 0;     ///< How many times the agent has extended its route searching.
    WorldPosition parkingDestBuilding_  = {0.0, 0.0}; ///< Building centroid the agent is targeting.
    double        parkingBuildingRadius_ = 0.0;        ///< Search radius around the building (m).

    std::vector<AgentHistoryEvent> history_; ///< Rolling life-event log (capped at 20).
};