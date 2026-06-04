/*
 * Note: Claude (Anthropic) was used to assist with commenting this file —
 * clarifying what the code physically does and documenting non-obvious decisions.
 * The implementation logic was written by the author (unless stated otherwise).
 */
#include "../include/InternalRoutePlanner.hpp"
#include "../include/SimulationLane.hpp"
#include "../include/SimulationRoad.hpp"
#include "../include/ParkingSystem.hpp"
#include <queue>
#include <random>
#include <unordered_set>
#include <algorithm>
#include <queue>
#include <limits>
#include <iostream>
#include <unordered_set>

// ---- Road-type cost multipliers ----------------------------------------
// Lower = faster/preferred. Agents naturally take arterials to minimise cost.
static double roadTypeCostMultiplier(RoadType rt) {
    switch (rt) {
        case RoadType::Motorway:    return 0.4;
        case RoadType::Trunk:       return 0.5;
        case RoadType::Primary:     return 0.7;
        case RoadType::Secondary:   return 1.0;
        case RoadType::Tertiary:    return 1.5;
        case RoadType::Residential: return 3.0;
        case RoadType::Service:     return 5.0;
        default:                    return 1.5;
    }
}

// Constructor, moves the graph and initializes the random generator with a fixed seed
InternalRoutePlanner::InternalRoutePlanner(LaneGraph graph, uint64_t seed)
    : graph_(std::move(graph)), randomGenerator_(seed) {
}

void InternalRoutePlanner::buildCostGraph() {
    laneCost_.clear();
    roadCost_.clear();
    roadBaseCost_.clear();
    roadAdj_.clear();
    deadEndPool_.clear();
    deadEndSet_.clear();
    roadDestPool_.clear();

    // Deduplicate road adjacency using sets, then convert.
    std::unordered_map<SimulationRoad*, std::unordered_set<SimulationRoad*>> roadAdjSet;

    // Collect every lane that appears as a successor (i.e. is reachable from some lane).
    std::unordered_set<SimulationLane*> reachableLanes;

    for (auto& [lane, succs] : graph_) {
        double len = lane->getTrimmedCenterlineLength();
        if (len < 1.0) len = 1.0;
        SimulationRoad* road = lane->getParentRoad();
        double mult = roadTypeCostMultiplier(road->getRoadType());

        laneCost_[lane] = mult * len;

        // Road cost = minimum across all lanes of this road (consistent with Dijkstra).
        double rc = mult * len;
        auto rcIt = roadCost_.find(road);
        if (rcIt == roadCost_.end() || rc < rcIt->second) {
            roadCost_[road]     = rc;
            roadBaseCost_[road] = rc;
        }

        for (SimulationLane* s : succs) {
            reachableLanes.insert(s);
            SimulationRoad* toRoad = s->getParentRoad();
            if (road != toRoad)
                roadAdjSet[road].insert(toRoad);
        }
    }

    for (auto& [road, adjSet] : roadAdjSet)
        roadAdj_[road] = std::vector<SimulationRoad*>(adjSet.begin(), adjSet.end());

    // Dead-end lanes: reachable (appear as a graph_ value) but NOT a key —
    // meaning there is nowhere to go from them. These are map borders / cul-de-sacs.
    // Routes always terminate on one of these so agents despawn on the open road,
    // never inside an intersection.
    std::unordered_set<SimulationRoad*> deadEndRoads;
    for (SimulationLane* lane : reachableLanes) {
        if (graph_.find(lane) == graph_.end()) {
            deadEndPool_.push_back(lane);
            deadEndSet_.insert(lane);
            deadEndRoads.insert(lane->getParentRoad());
        }
    }

    // Filter dead-ends to boundary-only when map bounds are known.
    // A dead-end whose endpoint is strictly inside the bounds box is a cul-de-sac —
    // remove it from both spawn and destination pools. Only lanes whose endpoint lies
    // on or outside the bounds edge are real map-boundary stubs.
    if (mapBoundsSet_ && !deadEndPool_.empty()) {
        auto isBoundaryLane = [&](SimulationLane* lane) -> bool {
            const auto& cl = lane->getTrimmedCenterline();
            if (cl.empty()) return false;
            // Check both endpoints — the lane is a boundary stub if either point
            // is outside (or exactly on) the OSM download bounds.
            for (const WorldPosition* pt : {&cl.front(), &cl.back()}) {
                if (pt->x <= mapBoundsMinX_ || pt->x >= mapBoundsMaxX_ ||
                    pt->y <= mapBoundsMinY_ || pt->y >= mapBoundsMaxY_) {
                    return true;
                }
            }
            return false;
        };

        deadEndPool_.erase(
            std::remove_if(deadEndPool_.begin(), deadEndPool_.end(),
                           [&](SimulationLane* l) { return !isBoundaryLane(l); }),
            deadEndPool_.end());
        deadEndSet_.clear();
        for (SimulationLane* l : deadEndPool_) deadEndSet_.insert(l);

        // Rebuild road destination pool from surviving boundary lanes only.
        deadEndRoads.clear();
        for (SimulationLane* l : deadEndPool_) deadEndRoads.insert(l->getParentRoad());

        std::cout << "[RoutePlanner] Boundary filter: " << deadEndPool_.size()
                  << " boundary dead-ends kept as spawn/destination points.\n";
    }

    roadDestPool_.assign(deadEndRoads.begin(), deadEndRoads.end());

    // Build spatial grid for findNearestRoutableLane (O(1) amortised lookup).
    // Only routable lanes (non-empty successors) are inserted.
    spatialGrid_.clear();
    for (const auto& [lane, succs] : graph_) {
        if (succs.empty()) continue;
        const auto& cl = lane->getTrimmedCenterline();
        if (cl.empty()) continue;
        const WorldPosition& mid = cl[cl.size() / 2];
        spatialGrid_[cellKey(toCell(mid.x), toCell(mid.y))].push_back(lane);
    }
}

// Normalized lateral position of a lane within its direction group (0=right, 1=left).
// Used to match lane positions across road transitions so agents stay in the same
// relative lane when going straight through an intersection.
static double normalizedLanePos(SimulationLane* lane) {
    const auto& lanes = lane->getParentRoad()->getLanes();
    int maxIdx = 0;
    for (SimulationLane* l : lanes) {
        if (l->isForward() == lane->isForward())
            maxIdx = std::max(maxIdx, l->getLaneIndex());
    }
    return (maxIdx > 0) ? static_cast<double>(lane->getLaneIndex()) / maxIdx : 0.5;
}

// Among a list of candidate lanes, return the one whose normalized lateral position
// is closest to `targetU`. When multiple candidates tie within a small tolerance
// (e.g. a road widens from 3 to 4 lanes and the centre agent is equidistant to two
// middle lanes), one is chosen at random so both middle lanes get traffic.
static SimulationLane* pickClosestLane(const std::vector<SimulationLane*>& candidates,
                                        double targetU,
                                        std::mt19937& rng)
{
    // Find minimum distance first.
    double minDist = std::numeric_limits<double>::max();
    for (SimulationLane* c : candidates) {
        double d = std::abs(normalizedLanePos(c) - targetU);
        if (d < minDist) minDist = d;
    }

    // Collect all lanes within a small tolerance of the minimum.
    // On even splits (3→4 lanes) this gives two tied candidates; on exact matches
    // it gives exactly one, so behaviour is unchanged for non-widening transitions.
    constexpr double TIE_TOLERANCE = 0.05;
    std::vector<SimulationLane*> tied;
    for (SimulationLane* c : candidates) {
        if (std::abs(normalizedLanePos(c) - targetU) <= minDist + TIE_TOLERANCE)
            tied.push_back(c);
    }

    if (tied.size() == 1) return tied[0];
    std::uniform_int_distribution<size_t> pick(0, tied.size() - 1);
    return tied[pick(rng)];
}

// Count lanes in the same travel direction as `lane` on its road.
static int countSameDirectionLanes(SimulationLane* lane) {
    const auto& lanes = lane->getParentRoad()->getLanes();
    int maxIdx = 0;
    for (SimulationLane* l : lanes)
        if (l->isForward() == lane->isForward())
            maxIdx = std::max(maxIdx, l->getLaneIndex());
    return maxIdx + 1;
}

/* [AI-generated: Claude] */
// Pick a lane when transitioning to a wider road (nextCount > prevCount).
// Window size = max(prevCount, ceil(nextCount/prevCount)), sorted by normalised
// lateral position and centered on the agent's current position.
// Examples: 1→3 window=3 (fully random), 2→3 window=2, 3→4 window=3.
static SimulationLane* pickWideningLane(const std::vector<SimulationLane*>& candidates,
                                         double targetU, int prevCount, std::mt19937& rng)
{
    std::vector<SimulationLane*> sorted = candidates;
    std::sort(sorted.begin(), sorted.end(), [](SimulationLane* a, SimulationLane* b) {
        return normalizedLanePos(a) < normalizedLanePos(b);
    });
    const int M = static_cast<int>(sorted.size());
    const int N = std::max(1, prevCount);
    int w = std::max(N, (M + N - 1) / N); // max(N, ceil(M/N))
    w = std::min(w, M);

    int closest = 0;
    double minD = std::numeric_limits<double>::max();
    for (int i = 0; i < M; ++i) {
        double d = std::abs(normalizedLanePos(sorted[i]) - targetU);
        if (d < minD) { minD = d; closest = i; }
    }

    int start = std::clamp(closest - w / 2, 0, M - w);
    std::uniform_int_distribution<int> pick(start, start + w - 1);
    return sorted[pick(rng)];
}
/* [end AI-generated] */

// Try to convert a Dijkstra road path into a lane sequence.
// Preserves the agent's relative lateral position (leftmost stays leftmost, etc.)
// When requireDeadEndAtLastHop=true (default), the last lane must be a dead-end lane.
// When false, any lane on the last road is accepted (used for parking routes).
static std::vector<SimulationLane*> convertRoadPathToLanes(
    SimulationLane* start,
    const std::vector<SimulationRoad*>& roadPath,
    const InternalRoutePlanner::LaneGraph& graph,
    const std::unordered_set<SimulationLane*>& deadEndSet,
    std::mt19937& rng,
    bool requireDeadEndAtLastHop = true)
{
    std::vector<SimulationLane*> route;
    route.push_back(start);

    for (size_t i = 1; i < roadPath.size(); ++i) {
        SimulationRoad* nextRoad  = roadPath[i];
        SimulationLane* prevLane  = route.back();
        bool            isLastHop = (i == roadPath.size() - 1);

        auto graphIt = graph.find(prevLane);
        if (graphIt == graph.end()) return {}; // prevLane is a dead-end — can't continue

        // All lanes on nextRoad reachable from prevLane.
        std::vector<SimulationLane*> candidates;
        for (SimulationLane* succ : graphIt->second) {
            if (succ->getParentRoad() == nextRoad)
                candidates.push_back(succ);
        }
        if (candidates.empty()) return {}; // lane-level gap

        double targetU   = normalizedLanePos(prevLane);
        int    prevCount = countSameDirectionLanes(prevLane);

        // Dispatch: widen when the next road has more lanes, otherwise keep closest.
        auto pickLane = [&](const std::vector<SimulationLane*>& pool) -> SimulationLane* {
            int nextCount = countSameDirectionLanes(pool[0]);
            if (nextCount > prevCount)
                return pickWideningLane(pool, targetU, prevCount, rng);
            return pickClosestLane(pool, targetU, rng);
        };

        if (isLastHop) {
            if (requireDeadEndAtLastHop) {
                std::vector<SimulationLane*> deadEnds;
                for (SimulationLane* c : candidates) {
                    if (deadEndSet.count(c)) deadEnds.push_back(c);
                }
                if (deadEnds.empty()) return {};
                route.push_back(pickLane(deadEnds));
            } else {
                route.push_back(pickLane(candidates));
            }
        } else if (i == roadPath.size() - 2) {
            // Second-to-last hop: prefer lanes that lead to a dead-end on the final
            // road so the last hop is guaranteed to succeed.
            SimulationRoad* lastRoad = roadPath.back();
            std::vector<SimulationLane*> guided;
            for (SimulationLane* c : candidates) {
                auto cIt = graph.find(c);
                if (cIt == graph.end()) continue;
                for (SimulationLane* s : cIt->second) {
                    if (s->getParentRoad() == lastRoad && deadEndSet.count(s)) {
                        guided.push_back(c);
                        break;
                    }
                }
            }
            auto& pool = guided.empty() ? candidates : guided;
            route.push_back(pickLane(pool));
        } else {
            route.push_back(pickLane(candidates));
        }
    }

    return route;
}

// Helper: reconstruct a road path from startRoad to destRoad using prev map.
// Returns empty vector on failure.
static std::vector<SimulationRoad*> reconstructRoadPath(
    SimulationRoad* startRoad,
    SimulationRoad* destRoad,
    const std::unordered_map<SimulationRoad*, SimulationRoad*>& prev)
{
    std::vector<SimulationRoad*> path;
    for (SimulationRoad* at = destRoad; at != nullptr; ) {
        path.push_back(at);
        if (at == startRoad) break;
        auto it = prev.find(at);
        if (it == prev.end()) { path.clear(); break; }
        at = it->second;
    }
    if (path.empty() || path.back() != startRoad) return {};
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<SimulationLane*> InternalRoutePlanner::generateRoute(SimulationLane* start) {
    if (deadEndPool_.empty())
        return generateRandomRoute(start, 10, 20);

    SimulationRoad* startRoad = start->getParentRoad();

    // ---- Road-path cache lookup (keyed by road, shared across all lanes on it) ----
    {
        auto it = roadPathCache_.find(startRoad);
        if (it != roadPathCache_.end()
            && simTime_ < it->second.expireAt
            && it->second.generation == cacheGeneration_) {
            RoadPathCacheEntry& entry = it->second;
            // Try each road path in round-robin order until one converts successfully.
            const int n = static_cast<int>(entry.roadPaths.size());
            for (int attempt = 0; attempt < n; ++attempt) {
                const std::vector<SimulationRoad*>& roadPath =
                    entry.roadPaths[entry.nextIdx % n];
                ++entry.nextIdx;
                std::vector<SimulationLane*> route =
                    convertRoadPathToLanes(start, roadPath, graph_, deadEndSet_, randomGenerator_);
                if (route.size() >= 2 && deadEndSet_.count(route.back()))
                    return route;
            }
            // All cached paths failed for this start lane — fall through to Dijkstra.
        }
    }

    // ---- Full road-level Dijkstra (no early exit — visits all reachable roads) ----
    // Track both cost and hop count. Cost drives shortest-path preference
    // (arterials over residential). Hops let us filter out very close destinations.
    // Reuse pre-allocated maps (clear() retains bucket capacity → no rehashing).
    using Entry = std::pair<double, SimulationRoad*>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    dijkstraDist_.clear();
    dijkstraPrev_.clear();
    dijkstraHops_.clear();

    dijkstraDist_[startRoad] = 0.0;
    dijkstraHops_[startRoad] = 0;
    pq.push({0.0, startRoad});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dijkstraDist_[u]) continue;

        auto adjIt = roadAdj_.find(u);
        if (adjIt == roadAdj_.end()) continue;

        for (SimulationRoad* v : adjIt->second) {
            if (v->isClosed()) continue; // hard block — closed roads are invisible to Dijkstra
            auto costIt = roadCost_.find(v);
            double edgeCost = (costIt != roadCost_.end()) ? costIt->second : 1.0;
            double nd = d + edgeCost;
            auto dIt = dijkstraDist_.find(v);
            if (dIt == dijkstraDist_.end() || nd < dIt->second) {
                dijkstraDist_[v] = nd;
                dijkstraPrev_[v] = u;
                dijkstraHops_[v] = dijkstraHops_[u] + 1;
                pq.push({nd, v});
            }
        }
    }

    // ---- Collect reachable dead-end roads, split by distance ---------------
    // Prefer destinations >= MIN_ROUTE_ROADS road-hops away so the agent
    // crosses several intersections before reaching the terminal lane.
    std::vector<SimulationRoad*> farPool, nearPool;
    for (SimulationRoad* dr : roadDestPool_) {
        if (dr == startRoad || dijkstraPrev_.find(dr) == dijkstraPrev_.end()) continue;
        // Skip destinations only reachable via a closed road (path cost would be ≥ 1e18)
        auto dIt = dijkstraDist_.find(dr);
        if (dIt == dijkstraDist_.end() || dIt->second >= 1e10) continue;
        (dijkstraHops_.at(dr) >= MIN_ROUTE_ROADS ? farPool : nearPool).push_back(dr);
    }
    // Prefer arterial exits (non-residential/service) so agents stay on major roads.
    auto& basePool = farPool.empty() ? nearPool : farPool;
    auto isArterial = [](SimulationRoad* r) -> bool {
        RoadType rt = r->getRoadType();
        return rt != RoadType::Residential && rt != RoadType::Service && rt != RoadType::Unknown;
    };
    std::vector<SimulationRoad*> arterialFiltered;
    for (SimulationRoad* r : basePool)
        if (isArterial(r)) arterialFiltered.push_back(r);
    std::vector<SimulationRoad*>& candidateRoads =
        arterialFiltered.empty() ? basePool : arterialFiltered;
    if (candidateRoads.empty())
        return generateRandomRoute(start, 10, 20);

    // Shuffle so the pool targets different destinations each time it is filled.
    std::shuffle(candidateRoads.begin(), candidateRoads.end(), randomGenerator_);

    // ---- Build a pool of up to ROAD_PATH_POOL_SIZE distinct road paths ------
    // Store road paths (not lane sequences) — the expensive part is Dijkstra,
    // not convertRoadPathToLanes. Each subsequent spawn from this road gets a
    // cached road path and only pays for convertRoadPathToLanes.
    RoadPathCacheEntry cacheEntry;
    cacheEntry.expireAt   = simTime_ + routeCacheTtl_;
    cacheEntry.generation = cacheGeneration_;

    std::vector<SimulationLane*> firstRoute;
    const int maxCandidates = static_cast<int>(candidateRoads.size());
    for (int i = 0;
         i < maxCandidates &&
         static_cast<int>(cacheEntry.roadPaths.size()) < ROAD_PATH_POOL_SIZE;
         ++i)
    {
        std::vector<SimulationRoad*> roadPath =
            reconstructRoadPath(startRoad, candidateRoads[i], dijkstraPrev_);
        if (roadPath.empty()) continue;

        // Validate: can this road path produce a usable lane sequence from `start`?
        std::vector<SimulationLane*> route =
            convertRoadPathToLanes(start, roadPath, graph_, deadEndSet_, randomGenerator_);
        if (route.size() < 2 || !deadEndSet_.count(route.back())) continue;

        cacheEntry.roadPaths.push_back(std::move(roadPath));
        if (firstRoute.empty()) firstRoute = std::move(route);
    }

    if (firstRoute.empty())
        return generateRandomRoute(start, 10, 20);

    cacheEntry.nextIdx = 1; // first route already consumed above
    roadPathCache_[startRoad] = std::move(cacheEntry);
    return firstRoute;
}

SimulationLane* InternalRoutePlanner::findNearestRoutableLane(WorldPosition pos, double maxRadius) const {
    // Fast path: spatial grid built by buildCostGraph().
    if (!spatialGrid_.empty()) {
        SimulationLane* best  = nullptr;
        double bestDist2 = maxRadius * maxRadius;

        const int cellRadius = static_cast<int>(std::ceil(maxRadius / GRID_CELL)) + 1;
        const int cxBase = toCell(pos.x);
        const int cyBase = toCell(pos.y);

        for (int dy = -cellRadius; dy <= cellRadius; ++dy) {
            for (int dx = -cellRadius; dx <= cellRadius; ++dx) {
                auto it = spatialGrid_.find(cellKey(cxBase + dx, cyBase + dy));
                if (it == spatialGrid_.end()) continue;
                for (SimulationLane* lane : it->second) {
                    const auto& cl = lane->getTrimmedCenterline();
                    const WorldPosition& mid = cl[cl.size() / 2];
                    double ddx = mid.x - pos.x;
                    double ddy = mid.y - pos.y;
                    double d2 = ddx * ddx + ddy * ddy;
                    if (d2 < bestDist2) { bestDist2 = d2; best = lane; }
                }
            }
        }
        return best;
    }

    // Fallback: linear scan (grid not yet built).
    SimulationLane* best  = nullptr;
    double bestDist2 = maxRadius * maxRadius;
    for (const auto& [lane, succs] : graph_) {
        if (succs.empty()) continue;
        const auto& cl = lane->getTrimmedCenterline();
        if (cl.empty()) continue;
        const WorldPosition& mid = cl[cl.size() / 2];
        double dx = mid.x - pos.x;
        double dy = mid.y - pos.y;
        double d2 = dx * dx + dy * dy;
        if (d2 < bestDist2) { bestDist2 = d2; best = lane; }
    }
    return best;
}

std::vector<SimulationLane*> InternalRoutePlanner::generateParkingRoute(
    SimulationLane* start,
    ParkingSpot** outSpot,
    WorldPosition* outBuildingPos,
    double* outBuildingRadius)
{
    if (!parkingSystem_) return generateRoute(start);

    const auto& buildings = parkingSystem_->getBuildingCentroids();
    if (buildings.empty()) return generateRoute(start);

    // Pick a random building centroid.
    std::uniform_int_distribution<size_t> bidx(0, buildings.size() - 1);
    WorldPosition buildingPos = buildings[bidx(randomGenerator_)];
    constexpr double BUILDING_SEARCH_RADIUS = 80.0; // metres agents scan around the building

    // Find the nearest *routable* lane (not a dead-end) to the building.
    SimulationLane* targetLane = findNearestRoutableLane(buildingPos, 800.0);
    if (!targetLane || targetLane == start ||
        targetLane->getParentRoad() == start->getParentRoad())
        return generateRoute(start);

    SimulationRoad* startRoad  = start->getParentRoad();
    SimulationRoad* targetRoad = targetLane->getParentRoad();

    // Run Dijkstra from startRoad to targetRoad (reuse pre-allocated maps).
    using Entry = std::pair<double, SimulationRoad*>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    dijkstraDist_.clear();
    dijkstraPrev_.clear();

    dijkstraDist_[startRoad] = 0.0;
    pq.push({0.0, startRoad});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dijkstraDist_[u]) continue;
        if (u == targetRoad) break;

        auto adjIt = roadAdj_.find(u);
        if (adjIt == roadAdj_.end()) continue;

        for (SimulationRoad* v : adjIt->second) {
            if (v->isClosed()) continue;
            auto costIt = roadCost_.find(v);
            double edgeCost = (costIt != roadCost_.end()) ? costIt->second : 1.0;
            double nd = d + edgeCost;
            auto dIt = dijkstraDist_.find(v);
            if (dIt == dijkstraDist_.end() || nd < dIt->second) {
                dijkstraDist_[v] = nd;
                dijkstraPrev_[v] = u;
                pq.push({nd, v});
            }
        }
    }

    if (dijkstraDist_.find(targetRoad) == dijkstraDist_.end()) return generateRoute(start);

    // Reconstruct road path.
    std::vector<SimulationRoad*> roadPath;
    for (SimulationRoad* at = targetRoad; at; ) {
        roadPath.push_back(at);
        if (at == startRoad) break;
        auto it = dijkstraPrev_.find(at);
        if (it == dijkstraPrev_.end()) { roadPath.clear(); break; }
        at = it->second;
    }
    std::reverse(roadPath.begin(), roadPath.end());
    if (roadPath.empty() || roadPath.front() != startRoad) return generateRoute(start);

    // Convert to lane sequence — no dead-end constraint at the last hop (parking routes
    // end at any routable lane near the building, not at a map-border dead-end).
    std::vector<SimulationLane*> route =
        convertRoadPathToLanes(start, roadPath, graph_, deadEndSet_,
                               randomGenerator_, /*requireDeadEndAtLastHop=*/false);

    if (route.size() < 2) return generateRoute(start);

    // Output building position and radius so the agent knows where to start scanning.
    if (outBuildingPos)    *outBuildingPos    = buildingPos;
    if (outBuildingRadius) *outBuildingRadius = BUILDING_SEARCH_RADIUS;

    // Optionally pre-claim a nearby free spot for the caller.
    if (outSpot) {
        *outSpot = nullptr;
        auto freeSpots = parkingSystem_->findFreeSpots(buildingPos, BUILDING_SEARCH_RADIUS);
        for (ParkingSpot* s : freeSpots) {
            if (parkingSystem_->claimSpot(s->id, -2)) {
                *outSpot = s;
                break;
            }
        }
    }

    return route;
}

std::vector<SimulationLane*> InternalRoutePlanner::generateRouteTo(
    SimulationLane* start, SimulationLane* target)
{
    if (!start || !target || start == target) return {};

    SimulationRoad* startRoad  = start->getParentRoad();
    SimulationRoad* targetRoad = target->getParentRoad();

    // Same road: verify the target is actually a graph successor (otherwise the
    // agent has no intersection path to it and will get stuck).
    if (startRoad == targetRoad) {
        auto it = graph_.find(start);
        if (it != graph_.end()) {
            for (SimulationLane* s : it->second)
                if (s == target) return {start, target};
        }
        return {};
    }

    using Entry = std::pair<double, SimulationRoad*>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    dijkstraDist_.clear();
    dijkstraPrev_.clear();

    dijkstraDist_[startRoad] = 0.0;
    pq.push({0.0, startRoad});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dijkstraDist_[u]) continue;
        if (u == targetRoad) break;

        auto adjIt = roadAdj_.find(u);
        if (adjIt == roadAdj_.end()) continue;

        for (SimulationRoad* v : adjIt->second) {
            if (v->isClosed()) continue;
            auto costIt = roadCost_.find(v);
            double edgeCost = (costIt != roadCost_.end()) ? costIt->second : 1.0;
            double nd = d + edgeCost;
            auto dIt = dijkstraDist_.find(v);
            if (dIt == dijkstraDist_.end() || nd < dIt->second) {
                dijkstraDist_[v] = nd;
                dijkstraPrev_[v] = u;
                pq.push({nd, v});
            }
        }
    }

    if (dijkstraDist_.find(targetRoad) == dijkstraDist_.end()) return {};

    // Reconstruct road path.
    std::vector<SimulationRoad*> roadPath;
    for (SimulationRoad* at = targetRoad; at; ) {
        roadPath.push_back(at);
        if (at == startRoad) break;
        auto it = dijkstraPrev_.find(at);
        if (it == dijkstraPrev_.end()) { roadPath.clear(); break; }
        at = it->second;
    }
    std::reverse(roadPath.begin(), roadPath.end());
    if (roadPath.empty() || roadPath.front() != startRoad) return {};

    // Build the lane route for everything EXCEPT the last road — identical to
    // generateParkingRoute's intermediateRoads approach.  Then add the exact
    // target lane manually so we always end on the requested lane, not just the
    // closest-lateral lane on targetRoad that convertRoadPathToLanes would pick.
    std::vector<SimulationRoad*> intermediateRoads(roadPath.begin(), roadPath.end() - 1);
    std::vector<SimulationLane*> route;
    if (intermediateRoads.size() >= 2) {
        route = convertRoadPathToLanes(start, intermediateRoads, graph_,
                                       deadEndSet_, randomGenerator_, false);
        if (route.empty()) return {};
    } else {
        route.push_back(start);
    }

    // Final hop: find a graph successor of route.back() that lands on target.
    SimulationLane* lastLane = route.back();
    if (lastLane == target) return route;

    auto graphIt = graph_.find(lastLane);
    if (graphIt == graph_.end()) return {};

    // Target must be a direct successor — parallel-lane combinations would break
    // LaneDrivingController / IntersectionEntryController, so we reject those.
    for (SimulationLane* succ : graphIt->second) {
        if (succ == target) {
            route.push_back(target);
            return route;
        }
    }

    return {}; // target not directly reachable from this approach
}

std::vector<SimulationLane*> InternalRoutePlanner::generateRandomRoute(
    SimulationLane* start, const int minLength, const int maxLength)
{
    // Use the class member generator
    std::uniform_int_distribution<int> lenDist(minLength, maxLength);
    const int targetLen = lenDist(randomGenerator_);

    std::vector<SimulationLane*> route;
    route.reserve(targetLen);
    route.push_back(start);

    for (int i = 1; i < targetLen; ++i) {
        auto it = graph_.find(route.back());
        if (it == graph_.end() || it->second.empty()) break;

        // Filter out lanes on closed roads
        std::vector<SimulationLane*> open;
        for (SimulationLane* l : it->second) {
            if (!l->getParentRoad()->isClosed()) open.push_back(l);
        }
        if (open.empty()) break;

        std::uniform_int_distribution<size_t> choiceDist(0, open.size() - 1);
        route.push_back(open[choiceDist(randomGenerator_)]);
    }
    // A route of only the start lane means the agent has nowhere to go — treat as no route.
    if (route.size() < 5) return {};
    return route;
}

// ---- Congestion-aware cost update ----------------------------------------

void InternalRoutePlanner::updateRoadSpeeds(
    const std::unordered_map<SimulationRoad*, double>& avgSpeedMps)
{
    for (auto& [road, baseCost] : roadBaseCost_) {
        // Never overwrite the closure cost — closed roads stay at 1e18
        if (road->isClosed()) continue;

        auto it = avgSpeedMps.find(road);
        if (it == avgSpeedMps.end() || it->second < 0.5) {
            // No agents or effectively stopped across the whole road — revert to base cost.
            roadCost_[road] = baseCost;
            continue;
        }
        double measuredSpeed = it->second;
        double speedLimit    = road->getSpeedLimitMps();
        // ratio ∈ (0, 1]: 1 = free-flow, 0 → standstill
        double ratio = std::min(1.0, measuredSpeed / speedLimit);
        // Congestion multiplier: inverse of ratio, capped at 6× base cost.
        // A road at 50% of its speed limit gets 2× cost; at 20% it gets 5× cost.
        double congestionMult = std::min(6.0, 1.0 / std::max(0.1, ratio));
        roadCost_[road] = baseCost * congestionMult;
    }

    // Road costs changed — bump the generation so existing cache entries are treated as
    // stale on next access and lazily replaced. This avoids the "clear storm" where every
    // spawn in the same tick after a speed update triggers a full Dijkstra simultaneously.
    ++cacheGeneration_;
}

void InternalRoutePlanner::setRoadClosed(SimulationRoad* road, bool closed) {
    if (closed) {
        roadCost_[road] = 1e18; // effectively infinite — Dijkstra will never choose this road
    } else {
        // Restore the base cost (pre-congestion-adjustment value)
        auto it = roadBaseCost_.find(road);
        if (it != roadBaseCost_.end()) roadCost_[road] = it->second;
    }
    // Bump generation so cache entries are lazily invalidated on next access.
    ++cacheGeneration_;
}

int InternalRoutePlanner::countReachableDestinations() const {
    if (roadDestPool_.empty()) return 0;
    // Multi-source BFS from all non-closed roads; count dest pool entries reached.
    std::unordered_set<SimulationRoad*> visited;
    std::queue<SimulationRoad*> q;
    for (const auto& [road, _] : roadAdj_) {
        if (!road->isClosed()) {
            visited.insert(road);
            q.push(road);
        }
    }
    while (!q.empty()) {
        SimulationRoad* u = q.front(); q.pop();
        auto adjIt = roadAdj_.find(u);
        if (adjIt == roadAdj_.end()) continue;
        for (SimulationRoad* v : adjIt->second) {
            if (!v->isClosed() && !visited.count(v)) {
                visited.insert(v);
                q.push(v);
            }
        }
    }
    int count = 0;
    for (SimulationRoad* rd : roadDestPool_)
        if (!rd->isClosed() && visited.count(rd)) count++;
    return count;
}

void InternalRoutePlanner::updateGraph(LaneGraph newGraph) {
    graph_ = std::move(newGraph);
    // Rebuild all derived data (costs, destination pool, spatial grid) from the new graph.
    // This is only called during tinkering (not every frame) so the cost is acceptable.
    buildCostGraph();
}