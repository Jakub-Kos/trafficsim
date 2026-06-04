#pragma once

#include "SimulationTypes.hpp"
#include <memory>
#include <vector>

// Forward declarations to avoid circular dependencies
class SimulationAgent;
class SimulationIntersectionPath;
class SimulationLane;
struct ParkingSpot;
class ParkingSystem;

/// Set once at startup from simulation.json "debugBlockedReason".
/// When false, all setBlockedReason / appendInt calls in controllers are skipped.
void setGlobalDebugBlockedReason(bool enabled);

/**
 * @interface IAgentController
 * @brief Abstract interface for the agent's decision-making logic (the "Brain").
 *
 * The Controller is responsible for high-level decisions:
 * - Detecting when to switch from a lane to an intersection.
 * - Checking if an intersection path is free.
 * - Deciding when to exit an intersection.
 * - Commanding the Agent to stop or change waypoints.
 */
class IAgentController {
public:
    virtual ~IAgentController() = default;

    /**
     * @brief Update the agent's high-level decisions for this frame.
     *
     * This method is called once per tick by the SimulationAgent. It checks
     * the agent's progress and surroundings, then calls methods on the Agent
     * to change state (e.g., stopVehicle(), enterIntersection()).
     *
     * @param agent Pointer to the SimulationAgent being controlled.
     */
    virtual void updateDecisions(SimulationAgent* agent) = 0;

    /**
     * Defines the name of the controller for the API.
     *
     * @return user-readable name of the controller being used
     */
    virtual std::string getName() const = 0;

    /**
     * Returns true if updateDecisions() is safe to call concurrently with other
     * thread-safe controllers.  A controller is thread-safe when it only reads
     * shared lane/intersection structure (immutable after map load) and only
     * writes to its own agent's fields — no queue mutations, no lock map writes.
     */
    virtual bool isThreadSafe() const { return false; }
};

/**
 * @class LaneDrivingController
 * @brief Strategy for driving normally along a lane.
 *
 * Monitors the agent's progress along the lane. When the end is reached,
 * it determines if the agent should finish or transition to an intersection.
 */
class LaneDrivingController : public IAgentController {
public:
    void updateDecisions(SimulationAgent* agent) override;
    std::string getName() const override { return "Lane Driving"; }
    // Only reads shared lane structure (immutable) and writes to its own agent.
    bool isThreadSafe() const override { return true; }
};

/**
 * @class IntersectionEntryController
 * @brief Strategy for waiting at an intersection entrance.
 *
 * Identifies the correct path through the intersection. Checks with the
 * IntersectionController if the path is available. If so, transitions the
 * agent into the intersection; otherwise, commands the agent to stop.
 *
 * If the agent remains blocked for REROUTE_THRESHOLD_FRAMES consecutive frames
 * (~45 s at 60 Hz), it triggers a full Dijkstra reroute so it can seek an
 * alternative path around the congestion.
 */
class IntersectionEntryController : public IAgentController {
public:
    void updateDecisions(SimulationAgent* agent) override;
    [[nodiscard]] std::string getName() const override { return "Intersection Entry"; }

private:
    /// Frames the agent has been continuously blocked at this entry (cannot enter).
    int stuckFrames_ = 0;
    /// Reroute when continuously blocked for this many frames (~45 s at 60 Hz).
    static constexpr int REROUTE_THRESHOLD_FRAMES = 2700;
    /// Cached result of findPathToNextLane() — resolved once, reused every tick.
    const SimulationIntersectionPath* cachedPath_ = nullptr;
};

/**
 * @class IntersectionCrossingController
 * @brief Strategy for traversing an intersection path.
 *
 * Monitors progress along the curved path. When the end is reached, checks
 * if there is space on the destination lane. If so, transitions the agent
 * out of the intersection; otherwise, commands the agent to stop inside.
 */
class IntersectionCrossingController : public IAgentController {
public:
    /**
     * @brief Construct a new IntersectionCrossingController.
     * @param path The path currently being traversed.
     */
    explicit IntersectionCrossingController(const SimulationIntersectionPath* path);

    void updateDecisions(SimulationAgent* agent) override;

    [[nodiscard]] std::string getName() const override { return "Intersection Crossing"; }

private:
    const SimulationIntersectionPath* activePath_; ///< The path being traversed.

    /// Frames the agent has been stopped at the exit of this path waiting for space on the
    /// next lane. If this reaches FORCE_EXIT_FRAMES the agent exits anyway to break a deadlock.
    int stuckAtExitFrames_ = 0;
    static constexpr int FORCE_EXIT_FRAMES = 90; ///< ~1.5 s at 60 Hz

    /// Frames the agent has been stopped mid-intersection (mustYield or locked conflict).
    /// If this reaches YIELD_DEADLOCK_FRAMES the agent forces through to break deadlock.
    int stuckYieldingFrames_ = 0;
    static constexpr int YIELD_DEADLOCK_FRAMES = 180; ///< ~3 s at 60 Hz
};

// ---- Lane-change tuning constants -------------------------------------------
inline constexpr double LANE_CHANGE_LENGTH   = 35.0;  ///< Metres of forward travel during a lane change.
inline constexpr double LC_TRIGGER_GAP       = 20.0;  ///< Bumper-to-bumper gap (m) that may trigger a lane change.
inline constexpr double LC_LEADER_SPEED_FRAC = 0.55;  ///< Leader must be slower than this fraction of desired speed.
inline constexpr double LC_MIN_DIST_TO_INT   = 55.0;  ///< Agent must be this far from the exit intersection (m).
inline constexpr double MIN_GAP_LC           = 5.0;   ///< Safety buffer used for space checks during lane change.
inline constexpr double LC_ABORT_SPEED       = 1.5;   ///< Speed (m/s) below which a mid-change agent commits or aborts.
inline constexpr double LC_COMMIT_FRACTION   = 0.5;   ///< Progress fraction past which the agent commits to the target lane.

/**
 * @class ParkingApproachController
 * @brief Strategy for an agent approaching its pre-claimed parking spot.
 *
 * Monitors progress along the parking lane. When close enough to the spot,
 * parks the agent and hands off to ParkingDepartureController.
 */
class ParkingApproachController : public IAgentController {
public:
    /**
     * @param claimedSpot    The pre-claimed spot this agent is heading to.
     * @param departDuration How long (sim seconds) to stay parked before departing.
     */
    explicit ParkingApproachController(ParkingSpot* claimedSpot,
                                       double departDuration = 14400.0);
    void updateDecisions(SimulationAgent* agent) override;
    [[nodiscard]] std::string getName() const override { return "Parking Approach"; }

private:
    ParkingSpot* claimedSpot_;
    double departDuration_;

    enum class State { Approaching, Parking };
    State state_ = State::Approaching;

    static constexpr double SPOT_ARRIVE_DIST = 8.0; ///< Park when within 8m of spot (IDM stops ~5m from virtual car).
};

/**
 * @class ParkingSearchController
 * @brief Strategy for an agent cruising along a parking lane to find a free spot.
 *
 * Installed by LaneDrivingController when the agent arrives on its final
 * parking-destination lane with no pre-claimed spot. Each frame it scans
 * for free spots on the current lane ahead of the agent. When a spot is
 * found it claims it and hands off to ParkingApproachController. When the
 * end of the lane is reached without a spot it extends the route to another
 * nearby parking lane (up to MAX_ATTEMPTS times), then gives up.
 */
class ParkingSearchController : public IAgentController {
public:
    ParkingSearchController() = default;
    void updateDecisions(SimulationAgent* agent) override;
    [[nodiscard]] std::string getName() const override { return "Parking Search"; }
private:
    static constexpr int    MAX_ATTEMPTS   = 3;    ///< Max route extensions before giving up.
    static constexpr double SCAN_RADIUS    = 60.0; ///< Spot search radius around agent (m).
    static constexpr int    ROUTE_COOLDOWN = 30;   ///< Frames between route-search retries (~0.5s at 60Hz).
    int routeSearchCooldown_ = 0; ///< Frames remaining before the next route-search attempt.
};

/**
 * @class ParkingDepartureController
 * @brief Strategy for an agent waiting in a parking spot to re-enter traffic.
 *
 * Waits for departure time, then places a ghost sentinel in the adjacent lane
 * to hold space. When a sufficient gap exists, the agent physically moves from
 * the spot to the lane entry point and joins the queue.
 */
class ParkingDepartureController : public IAgentController {
public:
    /**
     * @param targetLane    Lane the agent will merge into.
     * @param laneProgress  Insertion point (meters from lane start).
     * @param spotPos       Parked position (world coordinates).
     * @param spotHeading   Parked heading (radians).
     */
    ParkingDepartureController(SimulationLane* targetLane,
                                double laneProgress,
                                WorldPosition spotPos,
                                double spotHeading);
    ~ParkingDepartureController() override;

    void updateDecisions(SimulationAgent* agent) override;
    [[nodiscard]] std::string getName() const override { return "Parking Departure"; }

private:
    enum class State { WaitingForDepartureTime, WaitingForGap };
    State state_ = State::WaitingForDepartureTime;

    SimulationLane* targetLane_;
    double          targetProgress_;
    WorldPosition   spotPos_;
    double          spotHeading_;

    std::unique_ptr<SimulationAgent> ghost_;
    bool ghostRegistered_ = false;

    static constexpr double CLEAR_AHEAD  = 10.0; ///< Metres clear ahead of insertion point.
};

/**
 * @class LaneChangingController
 * @brief Strategy for executing a lateral lane change to an adjacent parallel lane.
 *
 * On the first update it:
 *   - Creates a ghost sentinel on the target lane to reserve space.
 *   - Generates a short diagonal transition path and hands it to the agent.
 *
 * Each subsequent update it keeps the ghost's reported progress in sync with
 * the real agent. When the transition waypoints are exhausted it completes the
 * swap (queue membership, route entry, currentLane) and hands off to a fresh
 * LaneDrivingController. setWaypointsFrom() is used so pathProgress() remains
 * accurate relative to the full target-lane centerline.
 *
 * FIXME: Usage of this controller was heavily reduced due to the aggressive lane changes
 *  making the sim behave a lot worse. The throughput was damaged by the agents making
 *  unnecessary lane changes.
 */
class LaneChangingController : public IAgentController {
public:
    /**
     * @param fromLane   Lane the agent is currently on.
     * @param targetLane Adjacent lane the agent is moving into.
     */
    LaneChangingController(SimulationLane* fromLane, SimulationLane* targetLane);

    /// Safety net: if the controller is destroyed before completion, deregister ghost.
    ~LaneChangingController() override;

    void updateDecisions(SimulationAgent* agent) override;
    [[nodiscard]] std::string getName() const override { return "Lane Changing"; }

private:
    SimulationLane* fromLane_;
    SimulationLane* targetLane_;

    std::unique_ptr<SimulationAgent> ghost_; ///< Passive sentinel placed in targetLane's queue.
    bool   ghostRegistered_ = false;         ///< True while ghost is live in the target queue.
    double startProgress_   = -1.0;          ///< Agent's pathProgress at start; < 0 = not yet initialised.
};