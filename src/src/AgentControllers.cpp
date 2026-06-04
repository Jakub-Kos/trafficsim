/*
 * Note: Claude (Anthropic) was used to assist with commenting this file —
 * clarifying what the code physically does and documenting non-obvious decisions.
 * The implementation logic was written by the author (unless stated otherwise).
 */
#include "../include/AgentControllers.hpp"
#include "../include/SimulationAgent.hpp"
#include "../include/SimulationLane.hpp"
#include "../include/SimulationRoad.hpp"
#include "../include/SimulationIntersection.hpp"
#include "../include/SimulationIntersectionController.hpp"
#include "../include/SimulationIntersectionPath.hpp"
#include "../include/ParkingSystem.hpp"
#include "../include/InternalRoutePlanner.hpp"
#include "../include/MathHelpers.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <cstdio>
#include <cstring>

// Global flag — set once at startup. When false, all blocked-reason string building is skipped.
static bool sDebugBlockedReason = true;
void setGlobalDebugBlockedReason(bool enabled) { sDebugBlockedReason = enabled; }

// Writes decimal |val| into buf, returns pointer past last char. Avoids snprintf overhead.
static char* appendInt(char* p, char* end, int val) {
    if (p >= end) return p;
    if (val < 0) { *p++ = '-'; val = -val; }
    char tmp[12]; int len = 0;
    if (val == 0) { tmp[len++] = '0'; }
    else { for (int n = val; n > 0; n /= 10) tmp[len++] = '0' + (n % 10); }
    for (int i = len - 1; i >= 0 && p < end; --i) *p++ = tmp[i];
    return p;
}

// ---- Shared geometry helpers ----------------------------------------

/* [AI-generated: Claude] */
/// Arc-distance of the nearest projected point on polyline `pts` from world position `pos`.
static double progressOnPolyline(const std::vector<WorldPosition>& pts, const WorldPosition& pos) {
    int N = static_cast<int>(pts.size());
    if (N < 2) return 0.0;
    double bestDist2 = std::numeric_limits<double>::max();
    int    bestIdx   = 0;
    double bestT     = 0.0;
    for (int i = 0; i < N - 1; ++i) {
        WorldPosition seg = pts[i + 1] - pts[i];
        double len2 = dot(seg, seg);
        double t    = (len2 > 1e-10) ? std::max(0.0, std::min(1.0, dot(pos - pts[i], seg) / len2)) : 0.0;
        WorldPosition proj = pts[i] + seg * t;
        WorldPosition diff = proj - pos;
        double d2 = dot(diff, diff);
        if (d2 < bestDist2) { bestDist2 = d2; bestIdx = i; bestT = t; }
    }
    double d = 0.0;
    for (int i = 1; i <= bestIdx; ++i)
        d += length(pts[i] - pts[i - 1]);
    d += bestT * length(pts[bestIdx + 1] - pts[bestIdx]);
    return d;
}
/* [end AI-generated] */

// --- LaneDrivingController ---
void LaneDrivingController::updateDecisions(SimulationAgent* agent) {
    SimulationLane* lane = agent->getCurrentLane();

    // Hand off to ParkingSearchController only on the LAST route lane.
    // Checking proximity here (mid-route) caused an infinite loop: PSC would set a
    // new multi-lane route, reinstall LDC, then LDC would immediately reinstall PSC
    // before the agent moved — preventing it from ever following the new route.
    if (agent->isParkingSearchMode() && agent->getParkingSystem() &&
        !agent->hasNextLaneInRoute())
    {
        double bRadius = agent->getParkingBuildingRadius();
        if (bRadius > 0.0) {
            // Only search when actually near the target building.
            WorldPosition bPos = agent->getParkingDestBuilding();
            WorldPosition aPos = agent->getPosition();
            double dx = aPos.x - bPos.x;
            double dy = aPos.y - bPos.y;
            if (dx*dx + dy*dy < bRadius * bRadius) {
                agent->setController(std::make_unique<ParkingSearchController>());
                return;
            }
            // Last lane but not near building — let the route end normally;
            // PSC step 4 will extend toward the building when the route expires.
        } else {
            // No building radius set — trigger on last route lane regardless.
            agent->setController(std::make_unique<ParkingSearchController>());
            return;
        }
    }

    // --- Lane-discipline check (only on straight sections far from intersection) ---
    // A lane change is triggered ONLY when the current lane cannot reach the next
    // route road (e.g. the agent is in a turn-only lane but needs to go straight).
    // FIXME: Overtaking is intentionally disabled: it caused agents to oscillate between
    //  adjacent lanes on every road, halving effective throughput on multi-lane roads.
    if (lane && agent->hasNextLaneInRoute()) {
        double distToEnd = lane->getTrimmedCenterlineLength() - agent->pathProgress();

        if (distToEnd > LC_MIN_DIST_TO_INT) {
            SimulationLane* nextRoute = agent->getNextRouteLane();
            if (nextRoute && lane->getExitIntersection()) {
                SimulationRoad* nextRoad = nextRoute->getParentRoad();

                // Check whether the current lane already reaches the next route road.
                bool currentCanReach = false;
                for (const auto& p : lane->getExitIntersection()->getPaths()) {
                    if (p.getFromLane() == lane &&
                        p.getToLane()->getParentRoad() == nextRoad) {
                        currentCanReach = true;
                        break;
                    }
                }

                if (!currentCanReach) {
                    // Current lane is a dead-end for this route — must change lanes.
                    for (int offset : {1, -1}) {
                        SimulationLane* target = lane->getAdjacentLane(offset);
                        if (!target) continue;
                        if (target->getExitIntersection() != lane->getExitIntersection()) continue;

                        // Target must reach the next route road.
                        bool compatible = false;
                        for (const auto& p : target->getExitIntersection()->getPaths()) {
                            if (p.getFromLane() == target &&
                                p.getToLane()->getParentRoad() == nextRoad) {
                                compatible = true;
                                break;
                            }
                        }
                        if (!compatible) continue;

                        // Target lane must be clear in the merge window.
                        double myProg  = agent->pathProgress();
                        bool   spaceOk = true;
                        SimulationAgent* it = target->getAgentQueue().getHead();
                        while (it) {
                            double relProg = it->pathProgress() - myProg;
                            if (relProg > -(agent->getLength() + MIN_GAP_LC) &&
                                relProg <  (LANE_CHANGE_LENGTH   + MIN_GAP_LC)) {
                                spaceOk = false;
                                break;
                            }
                            it = it->getNextInLane();
                        }

                        if (spaceOk) {
                            agent->setController(
                                std::make_unique<LaneChangingController>(lane, target));
                            return;
                        }
                    }
                }
            }
        }
    }

    // --- Switch to intersection entry logic when close ---
    if (agent->hasNextLaneInRoute() && lane) {
        double distToEnd = lane->getTrimmedCenterlineLength() - agent->pathProgress();
        if (distToEnd < 40.0) {
            agent->setController(std::make_unique<IntersectionEntryController>());
            return;
        }
    }

    if (agent->hasReachedEndOfPath()) {
        if (!agent->hasNextLaneInRoute()) {
            agent->markCompleted();
        }
    }
}

// --- IntersectionEntryController helpers ---

/**
 * @brief Decide whether the "don't block the box" rule must be enforced.
 *
 * The rule prevents an agent from entering an intersection it cannot exit,
 * avoiding gridlock when roads of equal importance meet.  However it is
 * counter-productive in two situations:
 *
 *   (a) MERGE / LANE-COUNT CHANGE — The path has zero conflict points,
 *       meaning no other traffic stream can ever cross it.  An agent backed
 *       up here cannot block anyone else, so the rule is skipped.
 *
 *   (b) PRIORITY ROAD — The entry road is strictly more important than every
 *       other road feeding into this intersection (e.g. Primary through a
 *       Residential side-road junction).  The right-of-way agent must be able
 *       to keep flowing; forcing it to stop for a backed-up destination would
 *       starve the arterial and cause cascading jams.
 *
 * The rule is ALWAYS enforced when the intersection is signal-controlled:
 * traffic lights give equal green phases to all approaches, so a blocked box
 * would hold up the next phase for everyone.
 *
 * RoadType enum ordering: lower value = higher priority (Motorway=0 … Unknown=7).
 */
static bool shouldEnforceBoxRule(SimulationLane*                   fromLane,
                                 const SimulationIntersectionPath* path,
                                 SimulationIntersection*           exitInt)
{
    // Signal-controlled → always enforce (symmetric phases, box = blocked phase).
    if (exitInt->isSignalControlled()) return true;

    // (a) No conflicts on this path → merge/lane-count change, zero cross-traffic risk.
    if (exitInt->getConflictsForPath(path).empty()) return false;

    // (b) Check if every other entry road is strictly lower priority than ours.
    //     If so, the agent has absolute right of way and can flow through.
    int myPriority = static_cast<int>(fromLane->getParentRoad()->getRoadType());
    for (SimulationLane* entry : exitInt->getEntryLanes()) {
        if (entry->getParentRoad() == fromLane->getParentRoad()) continue;
        int otherPriority = static_cast<int>(entry->getParentRoad()->getRoadType());
        if (otherPriority <= myPriority) {
            // Another road of equal or higher importance feeds in → enforce the rule.
            return true;
        }
    }
    // All cross-traffic is on lower-priority roads → skip the rule.
    return false;
}

// --- IntersectionEntryController ---

void IntersectionEntryController::updateDecisions(SimulationAgent* agent) {
    SimulationLane* lane = agent->getCurrentLane();
    if (!lane) { agent->setBlockedReason("no_lane"); return; }

    double laneLen   = lane->getTrimmedCenterlineLength();
    double prog      = agent->pathProgress();
    double stopPos   = laneLen - (agent->getLength() / 2.0 + 0.5);
    double distToStop = stopPos - prog;

    // Cache the path pointer — findPathToNextLane() scans the intersection path list and
    // the result doesn't change while the agent is waiting at the same entry.
    if (!cachedPath_) cachedPath_ = agent->findPathToNextLane();
    const SimulationIntersectionPath* path = cachedPath_;

    agent->setDebugYieldState(false, -1, {0,0});

    if (!path) {
        SimulationIntersection* exitInt = lane->getExitIntersection();
        int numPaths = 0;
        if (exitInt) {
            for (const auto& p : exitInt->getPaths())
                if (p.getFromLane() == lane) ++numPaths;
        }
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "no_path|prog=%d|laneLen=%d|dist=%d|exitInt=%d|pathsFromLane=%d",
            (int)prog, (int)laneLen, (int)distToStop, exitInt ? 1 : 0, numPaths);
        agent->setBlockedReason(buf);
        agent->stopVehicle();
        return;
    }

    SimulationIntersectionController& controller = agent->getIntersectionController();
    const auto& conflicts = lane->getExitIntersection()->getConflictsForPath(path);

    int pathIdx = lane->getExitIntersection()->getPathIndex(path);
    LightState signal = controller.getLightState(pathIdx);
    const char* sigChar = (signal == LightState::Red) ? "R"
                        : (signal == LightState::Yellow) ? "Y" : "G";

    double gap = agent->getGapToLeader();
    bool   reached = agent->hasReachedEndOfPath();

    bool blocked = false;
    const char* reason = "";

    // Traffic light check.
    if (signal == LightState::Red) {
        blocked = true; reason = "red_light";
    } else if (signal == LightState::Yellow) {
        double timeToStop = distToStop / (std::max(1.0, agent->getSpeed()));
        if (timeToStop > 2.0) {
            blocked = true; reason = "yellow_too_far";
        }
    }

    // 1. Check cross-traffic physically at conflict points.
    // For each conflict on our path, scan agents currently on the OPPOSING path
    // inside the intersection and block entry if one is physically near the crossing
    // point. Agents on the SAME path (same-direction convoy) are never in the
    // opposing queue, so this check does not prevent convoy following.
    if (!blocked) {
        for (const auto& c : conflicts) {
            bool iAmA = (c.pathA == path);
            const auto* oppPath = iAmA
                ? static_cast<const SimulationIntersectionPath*>(c.pathB)
                : static_cast<const SimulationIntersectionPath*>(c.pathA);
            double oppConfDist = iAmA ? c.distOnB : c.distOnA;

            SimulationAgent* opp = oppPath->getAgentQueue().getHead();
            while (opp) {
                double d = oppConfDist - opp->pathProgress();
                // Opponent front is within agentLength+2m before the point,
                // or up to agentLength+1m past it — physically occupying it.
                if (d > -(opp->getLength() + 1.0) && d < opp->getLength() + 2.0) {
                    blocked = true;
                    reason = "conflict_occupied";
                    agent->setDebugYieldState(true, opp->getId(), c.position);
                    break;
                }
                opp = opp->getNextInLane();
            }
            if (blocked) break;
        }
    }

    // 2. Check Physical Leader
    if (!blocked && gap < 5.0) {
        blocked = true;
        reason = "leader_gap";
    }

    // 3. Check Spillback (Path) + straight-through box-blocking
    if (!blocked) {
        bool isStraight = (path->getDirection() == TurnDirection::Straight);
        if (isStraight) {
            SimulationAgent* pathHead = path->getAgentQueue().getHead();
            if (pathHead != nullptr) {
                blocked = true;
                reason = "straight_occupied";
            }
        } else {
            SimulationAgent* pathTail = path->getAgentQueue().getTail();
            if (pathTail) {
                double tailEnd = pathTail->pathProgress() - pathTail->getLength();
                if (tailEnd < 0.5) {
                    blocked = true;
                    reason = "path_spill";
                }
            }
        }
    }

    // 4. "Don't block the box" — refuse entry if the exit lane is backed up.
    // canEnterNextLane() returns false when the tail of the destination lane hasn't cleared enough space
    // from the entrance (tailProgress <= tailLength + MIN_GAP).
    // This prevents the agent from committing to a crossing it can't complete, which would leave it stopped mid-intersection and cause deadlocks.
    //
    // The check is selectively skipped (via shouldEnforceBoxRule) for:
    // A. Merge / lane-count-change nodes (no conflict points → no cross-traffic risk).
    // B. Higher-priority roads passing through a lower-priority junction (e.g. a Primary street crossing a Residential side road).
    // It is always applied at signal-controlled intersections.
    if (!blocked
        && shouldEnforceBoxRule(lane, path, lane->getExitIntersection())
        && !agent->canEnterNextLane())
    {
        SimulationLane* toLane = path->getToLane();
        SimulationAgent* laneTail = toLane ? toLane->getAgentQueue().getTail() : nullptr;
        blocked = true;
        reason = "dest_blocked";
    }

    bool canEnter = !blocked;

    if (!blocked) {
        stuckFrames_ = 0;
        if (reached || distToStop < 0.5) {
            reason = "entering";
        } else {
            reason = "approaching";
        }
    } else {
        ++stuckFrames_;
        // Deadlock prevention: reroute if stuck at entry for too long
        // After ~45 s of being continuously blocked, ask the route planner for a new
        // route using the current (congestion-adjusted) road costs.  This lets the agent
        // seek an alternative path around the bottleneck rather than waiting forever.
        if (stuckFrames_ >= REROUTE_THRESHOLD_FRAMES && agent->getPlanner()) {
            SimulationLane* curLane = agent->getCurrentLane();
            if (curLane) {
                agent->regenerateRoute(curLane);
                cachedPath_ = nullptr; // route changed — invalidate cached path
                // Restart from LaneDrivingController so the new route is evaluated cleanly.
                agent->setController(std::make_unique<LaneDrivingController>());
                return;
            }
        }
    }

    // Append compact diagnostic context — only when debug output is enabled.
    if (sDebugBlockedReason) {
        char diagBuf[128];
        char* dp  = diagBuf;
        char* dend = diagBuf + sizeof(diagBuf) - 1;
        for (const char* s = "|sig="; *s; ) *dp++ = *s++;
        for (const char* s = sigChar; *s && dp < dend; ) *dp++ = *s++;
        for (const char* s = "|dist="; *s; ) *dp++ = *s++;
        dp = appendInt(dp, dend, (int)(distToStop * 10) / 10);
        for (const char* s = "|prog="; *s; ) *dp++ = *s++;
        dp = appendInt(dp, dend, (int)prog);
        for (const char* s = "|lane="; *s; ) *dp++ = *s++;
        dp = appendInt(dp, dend, (int)laneLen);
        for (const char* s = "|end="; *s; ) *dp++ = *s++;
        *dp++ = reached ? '1' : '0';
        for (const char* s = "|gap="; *s; ) *dp++ = *s++;
        dp = appendInt(dp, dend, std::isinf(gap) ? 9999 : (int)gap);
        *dp = '\0';
        agent->setBlockedReason(reason, diagBuf);
    }

    // Act
    if (canEnter) {
        if (reached || distToStop < 0.5) {
            agent->enterIntersection(path);
            agent->setController(std::make_unique<IntersectionCrossingController>(path));
        }
    } else {
        if (distToStop > 0.0 && distToStop < 100.0) {
            agent->setVirtualStopTarget(distToStop);
        } else {
            agent->stopVehicle();
        }
    }
}

// --- IntersectionCrossingController ---

// --- LaneChangingController ---

LaneChangingController::LaneChangingController(SimulationLane* fromLane,
                                               SimulationLane* targetLane)
    : fromLane_(fromLane), targetLane_(targetLane) {}

LaneChangingController::~LaneChangingController() {
    // Safety net: if the controller is discarded mid-transition, clean up the ghost.
    if (ghostRegistered_ && ghost_) {
        targetLane_->deregisterAgent(ghost_.get());
    }
}

void LaneChangingController::updateDecisions(SimulationAgent* agent) {
    // First-frame initialisation
    // The agent stays on its original lane waypoints throughout the transition.
    // The ghost acts as a "space reservation" on the target lane so other agents
    // on that lane see the incoming vehicle and maintain following distance.
    if (startProgress_ < 0.0) {
        startProgress_ = agent->pathProgress();
        double ghostProgress = progressOnPolyline(
            targetLane_->getTrimmedCenterline(), agent->getPosition());

        ghost_ = std::make_unique<SimulationAgent>(
            ~agent->getId(),     // unique ghost ID
            agent->getTypeId(),
            SimulationRoute{},   // empty route → completed = true
            0.0, 0.0, nullptr);
        ghost_->makeGhost(agent->getId(), ghostProgress);

        // Insert ghost at the position matching the agent's current equivalent progress.
        SimulationAgent* insertAfterThis = nullptr;
        SimulationAgent* it = targetLane_->getAgentQueue().getHead();
        while (it && it->pathProgress() > ghostProgress) {
            insertAfterThis = it;
            it = it->getNextInLane();
        }
        targetLane_->getAgentQueue().insertAfter(ghost_.get(), insertAfterThis);
        ghostRegistered_ = true;
        return;
    }

    // Update ghost position to match agent.
    if (ghostRegistered_ && ghost_) {
        double prog = progressOnPolyline(
            targetLane_->getTrimmedCenterline(), agent->getPosition());
        ghost_->setGhostProgress(prog);
    }

    // Safety: commit-or-abort if the agent gets blocked mid-lane-change
    // Without this, a stopped leader on the original lane can halt the agent
    // mid-transition, leaving it blocking both the original lane and the target
    // lane (via the ghost) indefinitely.
    if (agent->getSpeed() < LC_ABORT_SPEED) {
        double progressInChange = agent->pathProgress() - startProgress_;
        if (progressInChange >= LANE_CHANGE_LENGTH * LC_COMMIT_FRACTION) {
            // Past the point of no return — force-complete into the target lane.
            ghostRegistered_ = false;
            agent->completeLaneChange(fromLane_, targetLane_, ghost_.get());
            agent->setWaypointsFrom(targetLane_->getTrimmedCenterline(), agent->getPosition());
            agent->setController(std::make_unique<LaneDrivingController>());
        } else {
            // Too early to commit — abort; the destructor will remove the ghost.
            agent->setController(std::make_unique<LaneDrivingController>());
        }
        return;
    }

    // Completion: agent has advanced LANE_CHANGE_LENGTH on original lane
    bool doneByDistance = agent->pathProgress() >= startProgress_ + LANE_CHANGE_LENGTH;
    bool doneByEndOfPath = agent->hasReachedEndOfPath();

    if (doneByDistance || doneByEndOfPath) {
        ghostRegistered_ = false;  // prevent double-deregister in destructor

        // Swap queues: deregister from original lane, insert at ghost's slot on target lane.
        agent->completeLaneChange(fromLane_, targetLane_, ghost_.get());

        // Re-anchor waypoints to the full target-lane centerline from current position.
        agent->setWaypointsFrom(targetLane_->getTrimmedCenterline(), agent->getPosition());

        // Hand off to normal lane driving — this destroys *this.
        agent->setController(std::make_unique<LaneDrivingController>());
    }
}

// --- ParkingApproachController ---

ParkingApproachController::ParkingApproachController(ParkingSpot* claimedSpot,
                                                     double departDuration)
    : claimedSpot_(claimedSpot), departDuration_(departDuration) {}

void ParkingApproachController::updateDecisions(SimulationAgent* agent) {
    if (state_ != State::Approaching) return;

    ParkingSpot* spot = claimedSpot_;
    if (!spot) {
        // Spot gone — fall back to normal driving
        agent->setController(std::make_unique<LaneDrivingController>());
        return;
    }

    double progress = agent->pathProgress();
    double distToSpot = spot->laneProgress - progress;

    if (distToSpot <= SPOT_ARRIVE_DIST || agent->hasReachedEndOfPath()) {
        // Arrived — park the agent: deregisters from lane queue, teleports to spot position.
        state_ = State::Parking;
        double departTime = agent->getAgentSimTime() + departDuration_;
        agent->park(spot, departTime);

        // Set departure controller to handle re-entry when timer fires
        agent->setController(std::make_unique<ParkingDepartureController>(
            spot->adjacentLane,
            spot->laneProgress,
            spot->position,
            spot->heading));

    } else if (distToSpot > 0.0 && distToSpot < 30.0) {
        // Virtual "obstacle" placed at the spot: IDM brakes to its min-gap (~5 m)
        // behind the virtual car, which puts the agent within SPOT_ARRIVE_DIST.
        agent->setVirtualStopTarget(distToSpot);

    } else if (distToSpot < 0.0) {
        // Passed the spot — release claim and resume normal driving
        ParkingSystem* ps = agent->getParkingSystem();
        if (ps) ps->releaseSpot(spot->id);
        agent->setTargetSpot(nullptr);
        agent->setController(std::make_unique<LaneDrivingController>());
    }
}

// --- ParkingDepartureController ---

ParkingDepartureController::ParkingDepartureController(SimulationLane* targetLane,
                                                       double laneProgress,
                                                       WorldPosition spotPos,
                                                       double spotHeading)
    : targetLane_(targetLane), targetProgress_(laneProgress),
      spotPos_(spotPos), spotHeading_(spotHeading) {}

ParkingDepartureController::~ParkingDepartureController() {
    if (ghostRegistered_ && ghost_ && targetLane_) {
        targetLane_->deregisterAgent(ghost_.get());
    }
}

void ParkingDepartureController::updateDecisions(SimulationAgent* agent) {
    // Phase 0: Wait until departure time
    if (state_ == State::WaitingForDepartureTime) {
        if (!agent->isReadyToDepart(agent->getAgentSimTime())) {
            return; // Still waiting
        }
        state_ = State::WaitingForGap;
    }

    // Phase 1: Place ghost and wait for a gap in traffic
    if (state_ == State::WaitingForGap) {
        // Place ghost sentinel on the target lane if not yet placed
        if (!ghostRegistered_ && targetLane_) {
            ghost_ = std::make_unique<SimulationAgent>(
                ~agent->getId(),
                agent->getTypeId(),
                SimulationRoute{},
                0.0, 0.0, nullptr);
            ghost_->makeGhost(agent->getId(), targetProgress_);

            // Insert ghost at the correct queue position (ordered by progress, head=front)
            SimulationAgent* insertAfterThis = nullptr;
            SimulationAgent* it = targetLane_->getAgentQueue().getHead();
            while (it && it->pathProgress() > targetProgress_) {
                insertAfterThis = it;
                it = it->getNextInLane();
            }
            targetLane_->getAgentQueue().insertAfter(ghost_.get(), insertAfterThis);
            ghostRegistered_ = true;
        }

        if (!targetLane_) {
            // No valid target lane — just unpark and do normal driving
            agent->park(nullptr, 0.0); // clear parked state safely
            agent->setController(std::make_unique<LaneDrivingController>());
            return;
        }

        // Check for clearance AHEAD only.
        // The ghost sentinel already holds the space behind — traffic has stopped for it,
        // so requiring CLEAR_BEHIND would never be satisfied (IDM stops ~2m behind the ghost,
        // far less than any useful clearance constant).
        bool gapOk = true;
        SimulationAgent* it = targetLane_->getAgentQueue().getHead();
        while (it) {
            if (it == ghost_.get()) { it = it->getNextInLane(); continue; }
            double rel = it->pathProgress() - targetProgress_;
            if (rel > 0.0 && rel < agent->getLength() + CLEAR_AHEAD) { gapOk = false; break; }
            it = it->getNextInLane();
        }

        if (!gapOk) return; // Keep waiting

        // Gap found — compute the exact world position on the lane centerline
        WorldPosition lanePoint = spotPos_; // fallback
        const auto& pts = targetLane_->getTrimmedCenterline();
        if (!pts.empty()) {
            double acc = 0.0;
            for (size_t i = 0; i + 1 < pts.size(); ++i) {
                WorldPosition seg = pts[i+1] - pts[i];
                double segLen = length(seg);
                if (acc + segLen >= targetProgress_) {
                    double t = (targetProgress_ - acc) / segLen;
                    lanePoint = pts[i] + seg * t;
                    break;
                }
                acc += segLen;
                if (i + 2 == pts.size()) lanePoint = pts.back();
            }
        }

        // Teleport: instantly snap agent to the lane, replacing the ghost
        ghostRegistered_ = false;
        agent->teleportUnpark(targetLane_, ghost_.get(), lanePoint);
        ghost_.reset();

        // Clear parking search mode so LDC doesn't reinstall PSC
        agent->setParkingSearchMode(false);
        agent->addHistoryEvent(AgentHistoryEvent::Type::Departed,
                               agent->getAgentSimTime(), "rejoined traffic");

        // Generate a fresh route from the joined lane
        agent->regenerateRoute(targetLane_);

        agent->setController(std::make_unique<LaneDrivingController>());
    }
}

// --- ParkingSearchController ---
void ParkingSearchController::updateDecisions(SimulationAgent* agent) {
    SimulationLane* lane = agent->getCurrentLane();
    ParkingSystem*  ps   = agent->getParkingSystem();
    if (!lane || !ps) {
        agent->setParkingSearchMode(false);
        agent->markCompleted();
        return;
    }

    double agentProg = agent->pathProgress();

    // Step 1: claim a spot on the current lane that is still ahead.
    // Search around the agent position, not the building centroid, to avoid missing nearby spots.
    auto spots = ps->findFreeSpots(agent->getPosition(), SCAN_RADIUS);
    for (ParkingSpot* s : spots) {
        if (s->adjacentLane != lane) continue;
        if (s->laneProgress - agentProg < 2.0) continue; // passed or too close
        if (ps->claimSpot(s->id, agent->getId())) {
            agent->setTargetSpot(s);
            agent->setParkingSearchMode(false);
            agent->addHistoryEvent(AgentHistoryEvent::Type::TargetedSpot,
                                   agent->getAgentSimTime(), "spot id " + std::to_string(s->id));
            agent->setController(std::make_unique<ParkingApproachController>(s));
            return;
        }
    }

    InternalRoutePlanner* planner = agent->getPlanner();

    // Step 2: no spot on this lane — route to a reachable parking lane.
    // We don't pre-claim; PSC will claim on arrival. Rate-limited: Dijkstra is expensive.
    if (routeSearchCooldown_ > 0) {
        --routeSearchCooldown_;
    } else if (planner) {
        routeSearchCooldown_ = ROUTE_COOLDOWN;
        WorldPosition buildingPos    = agent->getParkingDestBuilding();
        double        buildingRadius = agent->getParkingBuildingRadius();
        WorldPosition searchPos = (buildingRadius > 0.0) ? buildingPos : agent->getPosition();
        double        searchRadius = (buildingRadius > 0.0) ? (buildingRadius + 40.0) : SCAN_RADIUS;

        auto bldgSpots = ps->findFreeSpots(searchPos, searchRadius);
        for (ParkingSpot* s : bldgSpots) {
            if (s->adjacentLane == lane || !s->adjacentLane) continue;
            auto newRoute = planner->generateRouteTo(lane, s->adjacentLane);
            if (newRoute.size() >= 2) {
                // Route to the parking lane; PSC will be reinstalled on arrival.
                agent->setRoute(newRoute);
                // parkingSearchMode stays true — LDC reinstalls PSC when near building
                agent->setController(std::make_unique<LaneDrivingController>());
                return;
            }
        }
    }

    // Step 3: no reachable lane nearby — cross the next intersection and keep searching.
    double distToEnd = lane->getTrimmedCenterlineLength() - agentProg;
    if (agent->hasNextLaneInRoute() && distToEnd < 40.0) {
        agent->setController(std::make_unique<IntersectionEntryController>());
        return;
    }
    if (!agent->hasReachedEndOfPath()) return;

    // Step 4: route exhausted — pick a new building and extend toward it.
    int attempts = agent->getParkingSearchAttempts();
    if (attempts < MAX_ATTEMPTS) {
        agent->incrementParkingSearchAttempts();
        if (planner) {
            WorldPosition newBldg;
            double        newRad = 0.0;
            auto newRoute = planner->generateParkingRoute(lane, nullptr, &newBldg, &newRad);
            if (newRoute.size() >= 2) {
                agent->setRoute(newRoute);
                agent->setParkingDestBuilding(newBldg);
                agent->setParkingBuildingRadius(newRad);
                agent->setController(std::make_unique<LaneDrivingController>());
                return;
            }
        }
    }

    // Give up — despawn.
    agent->setParkingSearchMode(false);
    agent->markCompleted();
}

// --- IntersectionCrossingController ---

IntersectionCrossingController::IntersectionCrossingController(const SimulationIntersectionPath* path)
    : activePath_(path) {}

void IntersectionCrossingController::updateDecisions(SimulationAgent* agent) {
    if (!activePath_) return;
    agent->setDebugYieldState(false, -1, {0,0}); // Reset

    SimulationIntersectionController& controller = agent->getIntersectionController();

    const double speed = agent->getSpeed();
    const double len = agent->getLength();
    const double myProgress = agent->pathProgress();

    const double rawBraking = (speed * speed) / (2.0 * 2.5);
    const double brakingDist = std::min(rawBraking, 12.0);
    constexpr double frontBuffer = 6.0;

    const double reserveBack = -(len / 2.0) - 0.5;
    const double reserveFront = (len / 2.0) + brakingDist + frontBuffer;

    const double releaseBack = -(len / 2.0) - 3.0;
    const double releaseFront = reserveFront;

    agent->setDebugHitbox({});

    const auto conflicts = activePath_->getFromLane()->getExitIntersection()->getConflictsForPath(activePath_);

    double nearestStopDist = std::numeric_limits<double>::infinity();

    for (const auto& c : conflicts) {
        // Skip points that are far away or already cleared along the path curve.
        const double distAlongPath = c.distOnA - myProgress;

        if (distAlongPath < -15.0) {
            controller.unlockConflict(c.id, agent->getId());
            continue;
        }
        if (distAlongPath > 25.0) continue;

        bool inReserve = (distAlongPath > reserveBack && distAlongPath < reserveFront);
        const bool inRelease = (distAlongPath > releaseBack && distAlongPath < releaseFront);

        // Anti-hoarding: if stopped, release locks on points we can't yet reach.
        if (speed < 0.1 && distAlongPath > 3.0) {
            controller.unlockConflict(c.id, agent->getId());
            inReserve = false;
        }

        bool mustYield = false;
        bool occupying = (distAlongPath < 3.5);

        if (inReserve && !controller.isConflictLocked(c.id, agent->getId()) && !occupying) {

            const bool iAmPathA = (c.pathA == activePath_);
            bool opponentHasPriority = (iAmPathA && c.priorityRule == 2) || (!iAmPathA && c.priorityRule == 1);

            if (opponentHasPriority) {
                const auto* oppPath = iAmPathA ? (const SimulationIntersectionPath*)c.pathB
                                               : (const SimulationIntersectionPath*)c.pathA;
                double oppConfDist = iAmPathA ? c.distOnB : c.distOnA;

                SimulationAgent* threat = oppPath->getAgentQueue().getHead();
                while (threat) {
                    double distToPoint = oppConfDist - threat->pathProgress();

                    if (distToPoint < -2.0) { // already passed the conflict point
                        threat = threat->getNextInLane();
                        continue;
                    }

                    if (distToPoint < 60.0) {
                        double realSpeed = threat->getSpeed();

                        if (realSpeed < 0.2) {
                            // Case A: He is stopped ON the point (or very close).
                            // This is a physical blockage. We MUST yield/stop.
                            if (distToPoint < 4.0) {
                                mustYield = true;
                                agent->setDebugYieldState(true, threat->getId(), c.position);
                                break;
                            }
                            // Case B: He is stopped far away (e.g. waiting at start line).
                            // We IGNORE him and take the gap.
                            threat = threat->getNextInLane();
                            continue;
                        }

                        double threatSpeed = std::max(5.0, realSpeed);
                        double tta = distToPoint / threatSpeed;

                        double distToClear = (distAlongPath + len + 2.0);
                        double myEstSpeed = std::max(3.0, speed);
                        double ttc = distToClear / myEstSpeed;

                        if (tta < (ttc + 2.0)) {
                            mustYield = true;
                            agent->setDebugYieldState(true, threat->getId(), c.position);
                            break;
                        }
                    }
                    threat = threat->getNextInLane();
                }
            }
        }

        if (mustYield) {
            controller.unlockConflict(c.id, agent->getId());
            double distToContact = distAlongPath - (len / 2.0 + 1.5);
            if (distToContact < nearestStopDist) nearestStopDist = distToContact;
            continue;
        }

        if (inReserve) {
            if (!controller.isConflictLocked(c.id, agent->getId())) {
                controller.lockConflict(c.id, agent->getId());
            }

            // Someone else holds this lock — calculate our stop point.
            if (controller.isConflictLocked(c.id, agent->getId())) {
                double distToContact = distAlongPath - (len / 2.0 + 1.0);
                if (distToContact < nearestStopDist) {
                    nearestStopDist = distToContact;
                    agent->setDebugYieldState(true, -1, c.position);
                }
            }
        }

        // Unlock once we've moved past the hysteresis window.
        if (!inRelease) {
             controller.unlockConflict(c.id, agent->getId());
        }
    }

    bool isStoppedMidPath = false;
    if (nearestStopDist != std::numeric_limits<double>::infinity()) {
        agent->setVirtualStopTarget(nearestStopDist);
        if (agent->getSpeed() < 0.1) {
            ++stuckYieldingFrames_;
            isStoppedMidPath = true;
        }
    }
    if (!isStoppedMidPath) stuckYieldingFrames_ = 0;

    // Mid-intersection deadlock breaker: if the agent has been stopped by a yield
    // or conflict lock for too long, force it through. This handles the case where
    // the opposing agent is itself stuck (e.g. a chain of yielding agents) and
    // hasReachedEndOfPath() never fires so FORCE_EXIT_FRAMES can't help.
    if (stuckYieldingFrames_ >= YIELD_DEADLOCK_FRAMES) {
        stuckYieldingFrames_ = 0;
        for (const auto& c : conflicts) {
            controller.unlockConflict(c.id, agent->getId());
        }
        // Don't exit yet — just clear the stop target and let physics resume.
        // The agent will naturally reach the end of the path and exit normally.
    }

    // Update debug reason so the user can see the ICC state, not a stale IEC string.
    if (sDebugBlockedReason) {
        // Reuse myProgress (already read at top of function — no second pathProgress() call).
        int    nPts  = activePath_ ? static_cast<int>(activePath_->getPathPoints().size()) : 0;
        const double gapVal = agent->getGapToLeader();
        char iccBuf[128];
        char* ip  = iccBuf;
        char* iend = iccBuf + sizeof(iccBuf) - 1;
        for (const char* s = "crossing|prog="; *s; ) *ip++ = *s++;
        ip = appendInt(ip, iend, (int)myProgress);
        for (const char* s = "|pts="; *s; ) *ip++ = *s++;
        ip = appendInt(ip, iend, nPts);
        for (const char* s = "|yf="; *s; ) *ip++ = *s++;
        ip = appendInt(ip, iend, stuckYieldingFrames_);
        for (const char* s = "|ef="; *s; ) *ip++ = *s++;
        ip = appendInt(ip, iend, stuckAtExitFrames_);
        for (const char* s = "|gap="; *s; ) *ip++ = *s++;
        ip = appendInt(ip, iend, std::isinf(gapVal) ? 99999 : (int)gapVal);
        *ip = '\0';
        agent->setBlockedReason(iccBuf);
    }

    if (agent->hasReachedEndOfPath()) {
        if (agent->canEnterNextLane()) {
            stuckAtExitFrames_ = 0;
            for (const auto& c : conflicts) {
                controller.unlockConflict(c.id, agent->getId());
            }
            agent->exitIntersection();
            agent->setDebugHitbox({});
            agent->setController(std::make_unique<LaneDrivingController>());
        } else {
            ++stuckAtExitFrames_;
            if (stuckAtExitFrames_ >= FORCE_EXIT_FRAMES) {
                // Deadlock breaker: force-exit after being stuck too long.
                // The IDM will re-establish spacing once the agent is on the lane.
                stuckAtExitFrames_ = 0;
                for (const auto& c : conflicts) {
                    controller.unlockConflict(c.id, agent->getId());
                }
                agent->exitIntersection();
                agent->setDebugHitbox({});
                agent->setController(std::make_unique<LaneDrivingController>());
            } else {
                agent->stopVehicle();
            }
        }
    }
}