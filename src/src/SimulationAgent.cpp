/*
 * Note: Gemini (Gemini 3.1 Pro) was used to assist with commenting this file —
 * clarifying what the code physically does and documenting non-obvious decisions.
 * The implementation logic was written by the author.
 */
#include "../include/SimulationAgent.hpp"
#include "../include/SimulationLane.hpp"
#include "../include/SimulationIntersection.hpp"
#include "../include/SimulationIntersectionPath.hpp"
#include "../include/InternalRoutePlanner.hpp"
#include "../include/MathHelpers.hpp"
#include "../include/ParkingSystem.hpp"

#include <cmath>
#include <algorithm>
#include <cstdio> // Added for safety logging

SimulationAgent::SimulationAgent(const int64_t id,
                                 std::string  typeId,
                                 const SimulationRoute& route,
                                 const double desiredSpeed,
                                 const double initialSpeed,
                                 InternalRoutePlanner* planner)
    : id_(id),
      typeId_(std::move(typeId)),
      position_{}, // Will be set below
      speed_(initialSpeed),
      acceleration_(DEFAULT_ACCELERATION),
      heading_(DEFAULT_HEADING),
      routeIndex_(0),
      route_(route),
      waypointIndex_(0),
      waypoints_(nullptr),
      currentLane_(nullptr),
      planner_(planner),
      completed_(false),
      inIntersection_(false),
      activeInt_(nullptr),
      activePath_(nullptr),
      distanceToNextAgent_(std::numeric_limits<double>::infinity()),
      desiredSpeed_(desiredSpeed),
      nextInLane_(nullptr),
      prevInLane_(nullptr),
      length_(DEFAULT_LENGTH), // Using default, could be from a VehicleTypeConfig
      width_(DEFAULT_WIDTH)    // Using default
{
    // Initialize on first lane
    if (!route_.empty()) {
        currentLane_ = route_[0];
        setWaypoints(currentLane_->getTrimmedCenterline());
        if (!waypoints_->empty()) {
            position_ = (*waypoints_)[0];
        }
        currentLane_->registerAgent(this);

        // Start with the LaneDriving logic
        controller_ = std::make_unique<LaneDrivingController>();
    } else {
        completed_ = true;
    }
}

// --- Core Update Loop ---

void SimulationAgent::update(const double dt) {
    if (isGhost_) return;  // Ghosts are passive sentinels — no movement, no IDM.
    if (completed_) return;

    // Parked agents: only run the controller (handles departure timing/gap checks).
    if (isParked_) {
        if (controller_) controller_->updateDecisions(this);
        return;
    }

    if (!waypoints_ || waypoints_->empty()) return;

    // 1. Decision Step (The Brain)
    if (controller_) {
        controller_->updateDecisions(this);
    }

    // 2. Physics Step (The Body)
    if (forceStop_) {
        // Controller commanded a stop (e.g. red light, blocked intersection)
        // We apply max braking to stop quickly but smoothly-ish
        acceleration_ = -MAX_DECEL;
    } else {
        // Run IDM Car-Following
        acceleration_ = calculateCarFollowingAccel();
    }

    // 3. Integration
    speed_ += acceleration_ * dt;
    if (speed_ < 0.0) speed_ = 0.0; // No reversing

    moveAlongWaypoints(dt);

    // Reset flag for next frame
    forceStop_ = false;
}

// --- Intelligent Driver Model (IDM) ---

double SimulationAgent::calculateCarFollowingAccel() {
    // 1. Gather IDM Inputs
    double s = std::numeric_limits<double>::infinity(); // Gap
    double leaderSpeed = desiredSpeed_; // Default to free flow

    // A. Check immediate leader in same segment (Lane or IntersectionPath)
    if (prevInLane_) {
        // Calculate dynamic gap based on path progress
        // (LeaderProgress - LeaderLength) - MyProgress
        double rawDist = prevInLane_->pathProgress() - pathProgress();
        s = rawDist - prevInLane_->getLength();
        leaderSpeed = prevInLane_->getSpeed();
    }
    // B. If no immediate leader, scan ahead (Iterative Lookahead)
    else {
        double myProgress = pathProgress();
        double currentTotalLen = getCurrentPathLength();
        double distToEnd = currentTotalLen - myProgress;

        // Check if we are close enough to the end to care about the next segment
        if (distToEnd < LOOKAHEAD_DIST) {
            double gapAhead = lookAheadForGap(distToEnd, LOOKAHEAD_DIST - distToEnd);
            if (gapAhead != std::numeric_limits<double>::infinity()) {
                s = gapAhead;
                // Treat cross-boundary leader as slow/stopped for safety since we don't have its pointer
                leaderSpeed = 0.0;
            }
        }
    }

    if (s < 0.1) s = 0.1; // Prevent division by zero / negative gaps
    distanceToNextAgent_ = s; // Store for diagnostics

    // 2. Intersection Conflict Check
    double distToConflict = std::numeric_limits<double>::infinity();

    // 3. Combine: Determine the "Effective" Leader
    double finalGap = s;
    double finalLeaderSpeed = leaderSpeed;

    if (virtualStopGap_ < finalGap) {
        finalGap = virtualStopGap_;
        finalLeaderSpeed = 0.0;
    }
    // Reset for next frame
    virtualStopGap_ = std::numeric_limits<double>::infinity();

    if (distToConflict < s) {
        finalGap = distToConflict;
        finalLeaderSpeed = 0.0; // Conflict points are stationary targets
    }

    // Ensure we don't divide by zero if the conflict point is literally inside us
    if (finalGap < 0.1) finalGap = 0.1;

    // 4. Apply Formula using the FINAL values
    double dV = speed_ - finalLeaderSpeed; // Use finalLeaderSpeed here!

    const double v = speed_;
    const double v0 = desiredSpeed_;

    const double s0 = MIN_GAP;

    // Precomputed: 2*sqrt(a*b)
    static const double TWO_SQRT_AB = 2.0 * std::sqrt(MAX_ACCEL * MAX_DECEL);

    // Desired dynamic gap s*
    // s* = s0 + v*T + (v*dV) / (2*sqrt(a*b))
    double s_star = s0 + (v * DESIRED_TIME_HEADWAY) + (v * dV) / TWO_SQRT_AB;

    // Acceleration calculation: 1 - (v/v0)^4
    const double r = v / v0;
    const double r2 = r * r;
    double accelTerm = 1.0 - r2 * r2;

    // Use finalGap here!
    double brakeTerm = 0.0;
    if (finalGap < 10000.0) {
        const double t = s_star / finalGap;
        brakeTerm = t * t;
    }

    return MAX_ACCEL * (accelTerm - brakeTerm);
}

double SimulationAgent::checkIntersectionConflicts(double currentSpeed) {
    isYielding_ = false;
    yieldingToId_ = -1;

    const SimulationIntersectionPath* pathToCheck = nullptr;
    double myDistOnPath = 0.0;

    // Case A: Approaching Intersection
    if (!inIntersection_ && currentLane_ && currentLane_->getExitIntersection()) {
        pathToCheck = findPathToNextLane();
        if(!pathToCheck) return std::numeric_limits<double>::infinity();

        myDistOnPath = -(currentLane_->getTrimmedCenterlineLength() - pathProgress());
    }
    // Case B: Inside Intersection
    else if (inIntersection_ && activePath_) {
        pathToCheck = activePath_;
        myDistOnPath = pathProgress();
    }

    if (!pathToCheck) return std::numeric_limits<double>::infinity();

    // Find intersection
    SimulationIntersection* intersection = nullptr;
    if (currentLane_) intersection = currentLane_->getExitIntersection();
    else if (activePath_) intersection = activePath_->getFromLane()->getExitIntersection();

    if (!intersection) return std::numeric_limits<double>::infinity();

    double minStopDist = std::numeric_limits<double>::infinity();

    // Iterate conflicts
    for (const auto& conf : intersection->getConflictPoints()) {
        const SimulationIntersectionPath* otherPath = nullptr;
        double myConfDist = 0;
        double otherConfDist = 0;

        if (conf.pathA == pathToCheck) {
            otherPath = (const SimulationIntersectionPath*)conf.pathB;
            myConfDist = conf.distOnA;
            otherConfDist = conf.distOnB;
        } else if (conf.pathB == pathToCheck) {
            otherPath = (const SimulationIntersectionPath*)conf.pathA;
            myConfDist = conf.distOnB;
            otherConfDist = conf.distOnA;
        } else {
            continue; // Not my conflict
        }

        // Is the conflict ahead of me?
        if (myConfDist < myDistOnPath) continue; // Passed it

        // CHECK OTHER PATH
        SimulationAgent* otherAgent = otherPath->getAgentQueue().getHead(); // Simplification: check head

        if (otherAgent) {
            double otherProgress = otherAgent->pathProgress();
            double distToConfForHim = otherConfDist - otherProgress;

            // "Time to Collision" / Proximity check
            // He is approaching (positive dist) or ON it (negative dist but not past vehicle length)
            bool heIsClose = (distToConfForHim < 15.0 && distToConfForHim > -otherAgent->getLength());

            if (heIsClose) {
                // Simplified Yield Rule: If he is closer to the point than me, I yield.
                if (distToConfForHim < (myConfDist - myDistOnPath)) {
                    double stopDist = (myConfDist - myDistOnPath) - MIN_GAP; // Stop before point
                    if (stopDist < minStopDist) {
                        minStopDist = stopDist;
                        isYielding_ = true;
                        yieldingToId_ = otherAgent->getId();
                        conflictPt_ = conf.position;
                    }
                }
            }
        }
    }

    return minStopDist;
}

// --- Safe Iterative Lookahead Logic ---
double SimulationAgent::lookAheadForGap(double accumulatedGap, double distRemaining) {
    double currentGap = accumulatedGap;
    double range = distRemaining;

    // Safety check on route index
    if (routeIndex_ < 0 || routeIndex_ >= static_cast<int>(route_.size()))
        return std::numeric_limits<double>::infinity();

    size_t nextLaneIdx = static_cast<size_t>(routeIndex_) + 1;
    bool checkIntersectionFirst = (currentLane_ != nullptr);

    // Iteratively check future segments without recursion (Prevents Stack Overflow)
    for (size_t i = nextLaneIdx; i < route_.size(); ++i) {
        if (range <= 0) break;

        // --- STEP A: Check the Intersection Path leading TO this lane ---
        if (checkIntersectionFirst) {
            if (i == 0) break;

            SimulationLane* prevLane = route_[i - 1];
            SimulationLane* nextLane = route_[i];

            if (!prevLane || !nextLane) continue;

            const SimulationIntersectionPath* path = nullptr;
            SimulationIntersection* exitInt = prevLane->getExitIntersection();

            if (exitInt) {
                const auto& paths = exitInt->getPaths();
                for (const auto& p : paths) {
                    if (p.getFromLane() == prevLane && p.getToLane() == nextLane) {
                        path = &p;
                        break;
                    }
                }
            }

            if (path) {
                // 1. Check for tail agent on path
                SimulationAgent* tail = path->getAgentQueue().getTail();
                if (tail) {
                    return currentGap + tail->pathProgress() - tail->getLength();
                }

                // 2. No car, add cached path length
                double pLen = path->getPathLength();
                currentGap += pLen;
                range -= pLen;
            }
        }

        if (range <= 0) break;

        // --- STEP B: Check the Lane itself ---
        SimulationLane* lane = route_[i];
        if (!lane) continue;

        SimulationAgent* tail = lane->getAgentQueue().getTail();
        if (tail) {
            return currentGap + tail->pathProgress() - tail->getLength();
        }

        // Add cached lane length
        double lLen = lane->getTrimmedCenterlineLength();
        currentGap += lLen;
        range -= lLen;

        checkIntersectionFirst = true;
    }

    return std::numeric_limits<double>::infinity();
}

double SimulationAgent::getCurrentPathLength() const {
    return currentPathLength_;
}

void SimulationAgent::moveAlongWaypoints(double dt) {
    if (!waypoints_) return;

    double remainingDist = speed_ * dt;
    const double totalRequested = remainingDist;
    int N = static_cast<int>(waypoints_->size());

    while (remainingDist > 0.0 && waypointIndex_ + 1 < N) {
        WorldPosition nextPt = (*waypoints_)[waypointIndex_ + 1];
        WorldPosition delta = nextPt - position_;
        double dist = length(delta);

        if (dist <= remainingDist) {
            // Reached waypoint
            position_ = nextPt;
            waypointIndex_++;
            remainingDist -= dist;
        } else {
            // Move partially
            WorldPosition dir = delta / dist;
            position_ += dir * remainingDist;
            remainingDist = 0.0;
        }
    }

    // Increment cached progress by the distance actually travelled
    pathProgress_ += (totalRequested - remainingDist);

    // Update heading
    if (speed_ > 0.01 && waypointIndex_ + 1 < N) {
        WorldPosition delta = (*waypoints_)[waypointIndex_ + 1] - position_;
        heading_ = std::atan2(delta.y, delta.x);
    }
}

// --- Controller API Implementation ---

void SimulationAgent::setController(std::unique_ptr<IAgentController> newController) {
    controller_ = std::move(newController);
}

void SimulationAgent::setWaypoints(const std::vector<WorldPosition>& pts) {
    waypoints_ = &pts;
    waypointIndex_ = 0;
    // Agent always starts at pts[0] when following a new path segment (enterIntersection,
    // exitIntersection, constructor).  Progress resets to 0; just recompute path length.
    pathProgress_ = 0.0;
    currentPathLength_ = 0.0;
    for (size_t i = 1; i < pts.size(); ++i)
        currentPathLength_ += length(pts[i] - pts[i - 1]);
}

void SimulationAgent::stopVehicle() {
    forceStop_ = true;
}

void SimulationAgent::enterIntersection(const SimulationIntersectionPath* path) {
    if (!path) return;

    // Leave lane
    if (currentLane_) {
        currentLane_->deregisterAgent(this);
        // Store the intersection pointer BEFORE setting currentLane to null
        activeInt_ = currentLane_->getExitIntersection();
    }

    // Enter path
    activePath_ = path;
    inIntersection_ = true;

    // Explicitly clear currentLane_ so we don't think we are still on it.
    // This is required for lookAheadForGap to work correctly.
    currentLane_ = nullptr;

    activePath_->getAgentQueue().enqueueTail(this);
    setWaypoints(activePath_->getPathPoints());
}

void SimulationAgent::exitIntersection() {
    // Leave path
    if (activePath_) {
        activePath_->getAgentQueue().remove(this);
    }

    // Enter next lane
    routeIndex_++;
    if (routeIndex_ < static_cast<int>(route_.size())) {
        SimulationLane* nextLane = route_[routeIndex_];
        if (nextLane) {
            nextLane->registerAgent(this);
            currentLane_ = nextLane;
            setWaypoints(nextLane->getTrimmedCenterline());
        }
    } else {
        completed_ = true;
    }

    // Reset intersection state
    inIntersection_ = false;
    activePath_ = nullptr;
    activeInt_ = nullptr;
}

void SimulationAgent::markCompleted() {
    if (currentLane_) currentLane_->deregisterAgent(this);
    currentLane_ = nullptr;
    completed_ = true;
}

void SimulationAgent::makeGhost(int64_t ownerId, double progress) {
    isGhost_       = true;
    ghostOwnerId_  = ownerId;
    ghostProgress_ = progress;
    speed_         = 0.0;
}

void SimulationAgent::park(ParkingSpot* spot, double departureSimTime) {
    // Deregister from current lane
    if (currentLane_) {
        currentLane_->deregisterAgent(this);
        currentLane_ = nullptr;
    }
    parkedSpot_         = spot;
    departureSimTime_   = departureSimTime;
    isParked_           = true;
    speed_              = 0.0;
    acceleration_       = 0.0;
    waypoints_          = nullptr;
    // Snap agent to the parking spot position
    if (spot) {
        position_ = spot->position;
        heading_  = spot->heading;
    }
    int depMin = static_cast<int>((departureSimTime - agentSimTime_) / 60.0);
    addHistoryEvent(AgentHistoryEvent::Type::Parked, agentSimTime_,
                    "departs in ~" + std::to_string(depMin) + "min");
}

void SimulationAgent::unpark(SimulationLane* targetLane, SimulationAgent* ghost) {
    if (!targetLane) return;

    if (ghost) {
        // Replace ghost in the queue with this agent, preserving queue order
        SimulationAgent* prev = ghost->getPrevInLane();
        SimulationAgent* next = ghost->getNextInLane();

        targetLane->deregisterAgent(ghost);

        // Insert this agent at the same position the ghost occupied
        targetLane->getAgentQueue().insertAfter(this, prev);
        nextInLane_ = next;
        prevInLane_ = prev;
        if (next) next->setPrevInLane(this);
        if (prev) prev->setNextInLane(this);
    } else {
        targetLane->registerAgent(this);
    }

    currentLane_ = targetLane;
    setWaypointsFrom(targetLane->getTrimmedCenterline(), position_);
}

void SimulationAgent::teleportUnpark(SimulationLane* targetLane, SimulationAgent* ghost,
                                     const WorldPosition& lanePoint) {
    isParked_   = false;
    parkedSpot_ = nullptr;
    speed_      = 0.0;
    position_   = lanePoint;

    // Join the lane queue (replaces ghost, sets currentLane_ and waypoints)
    unpark(targetLane, ghost);

    // Snap heading to the lane direction at the insertion point
    if (waypoints_ && waypointIndex_ + 1 < static_cast<int>(waypoints_->size())) {
        const WorldPosition delta = (*waypoints_)[waypointIndex_ + 1] - (*waypoints_)[waypointIndex_];
        const double d = length(delta);
        if (d > 1e-9) heading_ = std::atan2(delta.y, delta.x);
    }
}

void SimulationAgent::setRoute(const SimulationRoute& route) {
    route_      = route;
    routeIndex_ = 0;
}

void SimulationAgent::regenerateRoute(SimulationLane* startLane) {
    if (!planner_ || !startLane) return;
    auto newRoute = planner_->generateRoute(startLane);
    if (!newRoute.empty()) {
        route_      = std::move(newRoute);
        routeIndex_ = 0;
        addHistoryEvent(AgentHistoryEvent::Type::RouteRegenerated, agentSimTime_,
                        std::to_string(route_.size()) + " lanes");
    }
}

void SimulationAgent::setOwnedWaypoints(std::vector<WorldPosition> pts) {
    ownedWaypoints_ = std::move(pts);
    waypoints_      = &ownedWaypoints_;
    waypointIndex_  = 0;
    recomputePathProgress();
}

void SimulationAgent::setWaypointsFrom(const std::vector<WorldPosition>& pts,
                                        const WorldPosition& nearPos) {
    waypoints_ = &pts;
    int N = static_cast<int>(pts.size());
    if (N < 2) { waypointIndex_ = 0; return; }

    // Project nearPos onto each segment; keep the nearest.
    double bestDist2 = std::numeric_limits<double>::max();
    int    bestIdx   = 0;
    double bestT     = 0.0;
    for (int i = 0; i < N - 1; ++i) {
        WorldPosition seg = pts[i + 1] - pts[i];
        double len2 = dot(seg, seg);
        double t    = (len2 > 1e-10) ?
            std::max(0.0, std::min(1.0, dot(nearPos - pts[i], seg) / len2)) : 0.0;
        WorldPosition proj = pts[i] + seg * t;
        WorldPosition diff = proj - nearPos;
        double d2 = dot(diff, diff);
        if (d2 < bestDist2) { bestDist2 = d2; bestIdx = i; bestT = t; }
    }
    waypointIndex_ = bestIdx;
    // Snap position to the projected point so pathProgress() is exact from the start.
    // Without this, the lateral distance between old lane and new lane inflates pathProgress().
    position_ = pts[bestIdx] + (pts[bestIdx + 1] - pts[bestIdx]) * bestT;
    recomputePathProgress();
}

void SimulationAgent::completeLaneChange(SimulationLane* fromLane,
                                          SimulationLane* targetLane,
                                          SimulationAgent* ghost) {
    // Capture ghost's predecessor (the agent ahead of it) before removing.
    SimulationAgent* insertAfterThis = ghost ? ghost->getPrevInLane() : nullptr;

    // Remove ghost sentinel from the target lane's queue.
    if (ghost) targetLane->deregisterAgent(ghost);

    // Leave the current lane queue.
    if (fromLane) fromLane->deregisterAgent(this);

    // Insert at the position the ghost occupied so ordering is preserved.
    targetLane->getAgentQueue().insertAfter(this, insertAfterThis);

    // Update navigation state.
    currentLane_ = targetLane;
    if (routeIndex_ < static_cast<int>(route_.size()))
        route_[routeIndex_] = targetLane;

    // Also fix up the next route entry: targetLane may exit to a different
    // lane than fromLane did (parallel lanes → parallel exit lanes).
    // Find a path from targetLane whose toLane is on the same road as the
    // current route_[routeIndex_ + 1] and update it.
    if (routeIndex_ + 1 < static_cast<int>(route_.size())) {
        SimulationIntersection* exitInt = targetLane->getExitIntersection();
        SimulationRoad* nextRoad = route_[routeIndex_ + 1]->getParentRoad();
        if (exitInt && nextRoad) {
            for (const auto& path : exitInt->getPaths()) {
                if (path.getFromLane() == targetLane &&
                    path.getToLane()->getParentRoad() == nextRoad) {
                    route_[routeIndex_ + 1] = path.getToLane();
                    break;
                }
            }
        }
    }
}

// --- Predicates ---

bool SimulationAgent::hasReachedEndOfPath() const {
    if (!waypoints_) return true;
    return (waypointIndex_ + 1 >= static_cast<int>(waypoints_->size()));
}

bool SimulationAgent::hasNextLaneInRoute() const {
    return (routeIndex_ + 1 < static_cast<int>(route_.size()));
}

SimulationLane* SimulationAgent::getNextRouteLane() const {
    if (routeIndex_ + 1 < static_cast<int>(route_.size()))
        return route_[routeIndex_ + 1];
    return nullptr;
}

const SimulationIntersectionPath* SimulationAgent::findPathToNextLane() {
    if (!currentLane_ || !hasNextLaneInRoute()) return nullptr;

    SimulationIntersection* exitInt = currentLane_->getExitIntersection();
    if (!exitInt) return nullptr;

    if (routeIndex_ + 1 >= static_cast<int>(route_.size())) return nullptr;
    SimulationLane* nextLane = route_[routeIndex_ + 1];

    // Exact lane match
    for (const auto& path : exitInt->getPaths()) {
        if (path.getFromLane() == currentLane_ && path.getToLane() == nextLane) {
            return &path;
        }
    }

    // Fuzzy match: same parent road (handles lane offsets from route builder or lane changes)
    SimulationRoad* nextRoad = nextLane->getParentRoad();
    if (nextRoad) {
        for (const auto& path : exitInt->getPaths()) {
            if (path.getFromLane() == currentLane_ &&
                path.getToLane()->getParentRoad() == nextRoad) {
                route_[routeIndex_ + 1] = path.getToLane(); // patch route in place
                return &path;
            }
        }
    }

    // Last resort: route is unresolvable (e.g. lane changed into turn-only lane).
    // Prefer Straight, then any path, so the agent moves rather than freezing.
    const SimulationIntersectionPath* anyPath = nullptr;
    for (const auto& path : exitInt->getPaths()) {
        if (path.getFromLane() != currentLane_) continue;
        if (path.getDirection() == TurnDirection::Straight) {
            route_[routeIndex_ + 1] = path.getToLane();
            return &path;
        }
        if (!anyPath) anyPath = &path;
    }
    if (anyPath) {
        route_[routeIndex_ + 1] = anyPath->getToLane();
        return anyPath;
    }

    return nullptr;
}

SimulationIntersectionController& SimulationAgent::getIntersectionController() {
    // When inside an intersection, currentLane_ is nullptr.
    // We must use the stored activeInt_ pointer instead.
    if (inIntersection_ && activeInt_) {
        return activeInt_->getController();
    }

    // Fallback for normal lane driving
    if (currentLane_ && currentLane_->getExitIntersection()) {
        return currentLane_->getExitIntersection()->getController();
    }

    // Should not happen if logic is correct, but returning *activeInt_
    // (even if null) allows the crash to be strictly about the null pointer if it occurs.
    // FIXME: In a production engine, this would throw or log error.
    return activeInt_->getController();
}

bool SimulationAgent::canEnterNextLane() const {
    if (!hasNextLaneInRoute()) return true; // Exit to void is always fine

    if (routeIndex_ + 1 >= static_cast<int>(route_.size())) return true;
    SimulationLane* nextLane = route_[routeIndex_ + 1];

    if (!nextLane) return true;

    SimulationAgent* tail = nextLane->getAgentQueue().getTail();

    if (!tail) return true; // Lane is empty

    // Check space at start of lane
    double tailDist = tail->pathProgress(); // Distance from start
    double required = tail->getLength() + MIN_GAP;

    return (tailDist > required);
}

// --- Diagnostics / Getters ---
double SimulationAgent::pathProgress() const {
    if (isGhost_) return ghostProgress_;
    return pathProgress_;
}

void SimulationAgent::recomputePathProgress() {
    if (!waypoints_ || waypoints_->empty()) {
        pathProgress_      = 0.0;
        currentPathLength_ = 0.0;
        return;
    }
    const auto& pts = *waypoints_;
    int N = static_cast<int>(pts.size());

    // Current path length
    currentPathLength_ = 0.0;
    for (int i = 1; i < N; ++i)
        currentPathLength_ += length(pts[i] - pts[i - 1]);

    // Progress up to current waypoint index + partial segment
    double d = 0.0;
    for (int i = 1; i <= waypointIndex_ && i < N; ++i)
        d += length(pts[i] - pts[i - 1]);
    if (waypointIndex_ < N)
        d += length(position_ - pts[waypointIndex_]);
    pathProgress_ = d;
}

int64_t SimulationAgent::getLeaderId() const {
    return prevInLane_ ? prevInLane_->getId() : -1;
}

double SimulationAgent::getGapToLeader() const {
    if (prevInLane_) return distanceToNextAgent_;
    return std::numeric_limits<double>::infinity();
}