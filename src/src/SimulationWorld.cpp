/*
 * Note: Claude (Anthropic) was used under human supervision for two purposes here:
 * documenting non-obvious implementation decisions, and adding the TRAFFIC_NO_PARALLEL
 * flag and its associated logic. All other implementation is the author's own work.
 *
 * This file is large by necessity: it owns the core update loop, the OSM-to-world
 * construction, and the JSON state serializers. Splitting it would only push the
 * shared state through delegation layers without reducing actual complexity.
 */
#include "../include/SimulationWorld.hpp"
#include "../include/SimulationNode.hpp"
#include "../include/SimulationRoad.hpp"
#include "../include/SimulationLane.hpp"
#include "../include/SimulationIntersection.hpp"
#include "../include/SimulationAgent.hpp"
#include "../include/InternalSpawnSystem.hpp"
#include "../include/InternalRoutePlanner.hpp"
#include "../include/ParkingSystem.hpp"
#include "../include/MathHelpers.hpp"
#include "../include/MetricsCollector.hpp"

#include <nlohmann/json.hpp>
#include <unordered_map>
#include <algorithm>
#include <iostream>
#include <memory>
#include <unordered_set>
#include <cmath>
#ifndef TRAFFIC_NO_PARALLEL
#  include <execution>
#endif

using json = nlohmann::json;

/* [AI-generated: Claude] */
// calculateControlPoint removed — cubic Bézier uses tangent handles directly.

// --- Helper: Robust end-direction from a centerline ---
// Scans backwards from the last point to find a non-degenerate direction vector.
// Guards against trimmed centerlines whose last two (or more) points coincide.
static WorldPosition endDir(const std::vector<WorldPosition>& pts) {
    if (pts.size() < 2) return {0.0, 0.0};
    const WorldPosition& tail = pts.back();
    for (int i = (int)pts.size() - 2; i >= 0; --i) {
        WorldPosition v = normalize(tail - pts[i]);
        if (v.x != 0.0 || v.y != 0.0) return v;
    }
    return {0.0, 0.0};
}

// Robust start-direction (scans forward from the first point).
static WorldPosition startDir(const std::vector<WorldPosition>& pts) {
    if (pts.size() < 2) return {0.0, 0.0};
    const WorldPosition& head = pts.front();
    for (size_t i = 1; i < pts.size(); ++i) {
        WorldPosition v = normalize(pts[i] - head);
        if (v.x != 0.0 || v.y != 0.0) return v;
    }
    return {0.0, 0.0};
}
/* [end AI-generated] */

// --- Helper: Get Max Lane Index for a Road Direction ---
static int getMaxLaneIndex(SimulationRoad* road, bool isForward) {
    int maxIdx = -1;
    for (const auto& lane : road->getLanes()) {
        if (lane->isForward() == isForward) {
            maxIdx = std::max(maxIdx, lane->getLaneIndex());
        }
    }
    return maxIdx;
}

// --- Constructor & Destructor ---

SimulationWorld::SimulationWorld()
{
    // Pre-reserve the flat agent pool so addresses are stable for the lifetime
    // of the simulation — lane queues and intrusive linked-list pointers rely on this.
    // Capacity is set to agentPoolCapacity_ (default 16384); call setSimulationTuning()
    // before initialize() to override.
    agents_.reserve(agentPoolCapacity_);

    // Initialize helper systems (seed 0 until setSeed() is called before initialize())
    spawner_ = std::make_unique<InternalSpawnSystem>(this, 0);
    // routePlanner_ is initialized in initialize() after the graph is built
}

SimulationWorld::~SimulationWorld() = default;

// --- Phase 1: Setup & Initialization ---
void SimulationWorld::setGlobalMapParameters(const GlobalMapParameters& params) {
    globalParameters_ = params;
}

[[nodiscard]] GlobalMapParameters SimulationWorld::getGlobalMapParameters() const {
    return globalParameters_;
}

/* [AI-debugged: Gemini] Gemini was used when adding new features do to the long character of this method.
 * All the logic is sort of artifficial and splitting it into multiple classes would only add boilerplate code.*/
void SimulationWorld::initialize(const std::string& mapJsonData) {
    // --- 1. Parse Simple Map JSON ---
    nodes_.clear();
    roads_.clear();
    intersections_.clear();
    json map = json::parse(mapJsonData);

    if (map.contains("rawOsm")) {
        rawOsmDataString_ = map["rawOsm"].dump();
    }

    if (map.contains("buildings")) {
        buildingsDataString_ = map["buildings"].dump();
    }

    if (map.contains("ways")) {
        waysDataString_ = map["ways"].dump();
    }

    if (map.contains("nodes")) {
        nodesDataString_ = map["nodes"].dump();
    }

    // 1. Nodes
    for (const auto& nodeJson : map["nodes"]) {
        int64_t id = nodeJson["id"];
        WorldPosition pos = {nodeJson["pos"]["x"], nodeJson["pos"]["y"]};
        IntersectionType type = IntersectionType::None;
        if (nodeJson.contains("type")) {
            type = static_cast<IntersectionType>(nodeJson["type"].get<int>());
        }
        else if (nodeJson.contains("signal") && nodeJson["signal"].get<bool>()) {
            type = IntersectionType::TrafficLight;
        }
        nodes_[id] = std::make_unique<SimulationNode>(id, pos, type);
    }

    // 2. Roads
    double laneWidth = globalParameters_.defaultLaneWidth;
    struct RoadData { double length; int totalLanes; };
    std::unordered_map<int64_t, RoadData> roadDataMap;

    // turn:lanes restriction: laneId → set of allowed TurnDirections (empty = unrestricted)
    std::unordered_map<int64_t, std::vector<TurnDirection>> laneTurnAllowed;

    for (const auto& wayJson : map["ways"]) {
        int64_t wayId = wayJson["id"];
        std::vector<SimulationNode*> wayNodes;

        // Find the node pointers for this way
        for (const auto& nodeId : wayJson["nodes"]) {
            if (nodes_.count(nodeId)) wayNodes.push_back(nodes_.at(nodeId).get());
        }
        if (wayNodes.size() < 2) continue; // Skip invalid ways

        double roadLen = 0.0;
        for (size_t i = 0; i < wayNodes.size() - 1; ++i) {
            roadLen += length(wayNodes[i+1]->getPosition() - wayNodes[i]->getPosition());
        }
        int lanesFwd = wayJson.contains("lanesForward") ? wayJson["lanesForward"].get<int>() : 1;
        int lanesBwd = wayJson.contains("lanesBackward") ? wayJson["lanesBackward"].get<int>() : 1;
        roadDataMap[wayId] = {roadLen, lanesFwd + lanesBwd};

        auto road = std::make_unique<SimulationRoad>(wayId, wayNodes);
        if (wayJson.contains("type"))
            road->setRoadType(static_cast<RoadType>(wayJson["type"].get<int>()));
        if (wayJson.contains("speedLimitKph")) {
            double kph = wayJson["speedLimitKph"].get<double>();
            if (kph > 0.0) road->setSpeedLimitMps(kph / 3.6);
        }

        // Parse parking lane tags
        auto parseParkingOrientation = [](const std::string& v) -> ParkingOrientation {
            if (v == "parallel")     return ParkingOrientation::Parallel;
            if (v == "diagonal")     return ParkingOrientation::Diagonal;
            if (v == "perpendicular" || v == "orthogonal") return ParkingOrientation::Perpendicular;
            return ParkingOrientation::None;
        };
        if (wayJson.contains("parkingLeft"))
            road->setParkingLeft(parseParkingOrientation(wayJson["parkingLeft"].get<std::string>()));
        if (wayJson.contains("parkingRight"))
            road->setParkingRight(parseParkingOrientation(wayJson["parkingRight"].get<std::string>()));

        double currentCenterOffset = (lanesBwd - lanesFwd) * (laneWidth / 2.0);

        for (int i = 0; i < lanesFwd; ++i) {
            double offset = -((laneWidth / 2.0) + (i * laneWidth)) - currentCenterOffset;
            int64_t laneId = (wayId * 1000) + 100 + i;
            road->addLane(std::make_unique<SimulationLane>(laneId, road.get(), true, offset, i, 0.0));
        }
        for (int i = 0; i < lanesBwd; ++i) {
            double offset = (laneWidth / 2.0) + (i * laneWidth) - currentCenterOffset;
            int64_t laneId = (wayId * 1000) + 200 + i;
            road->addLane(std::make_unique<SimulationLane>(laneId, road.get(), false, offset, i, 0.0));
        }

        // --- Parse turn:lanes restrictions (stored as JSON arrays by OSMMapLoader) ---
        // OSM order: leftmost lane first.  Our forward lane 0 = leftmost → direct mapping.
        // Backward lanes are reversed (lane 0 = rightmost in fwd direction = leftmost in bwd).
        auto parseTurnSpec = [](const std::string& spec) -> std::vector<TurnDirection> {
            std::vector<TurnDirection> dirs;
            if (spec.find("left")    != std::string::npos) dirs.push_back(TurnDirection::Left);
            if (spec.find("through") != std::string::npos) dirs.push_back(TurnDirection::Straight);
            if (spec.find("right")   != std::string::npos) dirs.push_back(TurnDirection::Right);
            return dirs; // empty = unrestricted (covers "none", blank, unknown)
        };

        if (wayJson.contains("turnLanesForward") && wayJson["turnLanesForward"].is_array()) {
            const auto& arr = wayJson["turnLanesForward"];
            for (int i = 0; i < lanesFwd && i < (int)arr.size(); ++i) {
                auto dirs = parseTurnSpec(arr[i].get<std::string>());
                if (!dirs.empty()) {
                    int64_t laneKey = (wayId * 1000) + 100 + i;
                    laneTurnAllowed[laneKey] = dirs;
                }
            }
        }
        if (wayJson.contains("turnLanesBackward") && wayJson["turnLanesBackward"].is_array()) {
            const auto& arr = wayJson["turnLanesBackward"];
            // Backward: OSM index 0 = leftmost in backward direction = highest laneIndex
            for (int i = 0; i < lanesBwd && i < (int)arr.size(); ++i) {
                int ourIdx = (lanesBwd - 1) - i;   // reverse mapping
                auto dirs = parseTurnSpec(arr[i].get<std::string>());
                if (!dirs.empty())
                    laneTurnAllowed[(wayId * 1000) + 200 + ourIdx] = dirs;
            }
        }

        roads_[wayId] = std::move(road);
    }

    // 3. Intersections (Your Standard Logic)
    struct RoadConnRaw { SimulationRoad* r; double len; WorldPosition dir; int totalLanes; };
    std::unordered_map<SimulationNode*, std::vector<RoadConnRaw>> nodeConnectivity;

    for (auto& [id, road] : roads_) {
        SimulationRoad* r = road.get();
        double len = roadDataMap[id].length;
        int lanes = roadDataMap[id].totalLanes;
        const auto& pts = r->getNodes();
        if (pts.size() < 2) continue;

        WorldPosition vFront = normalize(pts[1]->getPosition() - pts[0]->getPosition());
        nodeConnectivity[pts.front()].push_back({r, len, vFront, lanes});
        WorldPosition vBack = normalize(pts[pts.size()-2]->getPosition() - pts.back()->getPosition());
        nodeConnectivity[pts.back()].push_back({r, len, vBack, lanes});
    }

    std::unordered_map<SimulationNode*, SimulationIntersection*> nodeToIntersection;

    for (auto& [node, connected] : nodeConnectivity) {
        if (connected.size() > 1) {
            double minRoadLen = 1e9;
            double maxRoadWidth = 0.0;
            for(const auto& c : connected) {
                if(c.len < minRoadLen) minRoadLen = c.len;
                double w = c.totalLanes * laneWidth;
                if (w > maxRoadWidth) maxRoadWidth = w;
            }

            double widthBasedRadius = maxRoadWidth * 0.65;
            double lenBasedLimit = minRoadLen * 0.45;
            double finalRadius = std::min(widthBasedRadius, lenBasedLimit);

            if (finalRadius > 18.0) finalRadius = 18.0;
            if (finalRadius < globalParameters_.defaultIntersectionCurbRadius && lenBasedLimit > 8.0)
                 finalRadius = globalParameters_.defaultIntersectionCurbRadius;

            // Two-road straight-through: either a simple bend or a lane-count transition.
            bool isControlNode = (node->getType() != IntersectionType::None);
            if (connected.size() == 2 && !isControlNode) {
                double dp = dot(connected[0].dir, connected[1].dir);
                if (dp < -0.95) {
                    // Straight-through: scale radius to bridge any lateral offset mismatch
                    // caused by differing lane counts (e.g. 2-lane → 3-lane expansion).
                    int laneCountDiff = std::abs(connected[0].totalLanes - connected[1].totalLanes);
                    if (laneCountDiff > 0) {
                        // Give the bezier path enough length to transition smoothly.
                        finalRadius = std::max(laneWidth * 2.0,
                                               laneCountDiff * laneWidth * 1.5);
                    } else {
                        finalRadius = 0.5;   // same lane count: pure geometry join
                    }
                }
            }
            if (finalRadius < 0.1) finalRadius = 0.1;

            auto intersection = std::make_unique<SimulationIntersection>(
                node->getId(), node->getPosition(), finalRadius);
            nodeToIntersection[node] = intersection.get();
            intersections_.push_back(std::move(intersection));
        }
    }

    // 4. Link Lanes & Detect Signals on Approach
    std::unordered_set<int64_t> signaledIntersections; // NEW: Keep track of intersections that have signals nearby

    for (auto& [id, road] : roads_) {
        SimulationNode* firstNode = road->getNodes().front();
        SimulationNode* lastNode = road->getNodes().back();
        const auto& roadNodes = road->getNodes();

        for (auto& lane : road->getLanes()) {
            SimulationNode* startNode = lane->isForward() ? firstNode : lastNode;
            SimulationNode* exitNode = lane->isForward() ? lastNode : firstNode;

            if (auto it = nodeToIntersection.find(startNode); it != nodeToIntersection.end())
                lane->setStartIntersection(it->second);
            if (auto it = nodeToIntersection.find(exitNode); it != nodeToIntersection.end())
                lane->setExitIntersection(it->second);

            // --- SCAN FOR SIGNALS ---
            // If this lane enters an intersection, scan backwards to find if there is a signal.
            if (lane->getExitIntersection()) {
                bool foundSignal = false;
                WorldPosition signalPos;

                if (lane->isForward()) {
                    // Check nodes from End backwards
                    for (int k = (int)roadNodes.size() - 2; k >= 0; --k) {
                        SimulationNode* n = roadNodes[k];
                        // Stop scanning if too far (e.g. > 45m)
                        if (length(n->getPosition() - exitNode->getPosition()) > 45.0) break;

                        if (n->getType() == IntersectionType::TrafficLight) {
                            foundSignal = true;
                            signalPos = n->getPosition();
                            break;
                        }
                    }
                } else {
                    // Check nodes from Start forwards
                    for (size_t k = 1; k < roadNodes.size(); ++k) {
                        SimulationNode* n = roadNodes[k];
                        if (length(n->getPosition() - exitNode->getPosition()) > 45.0) break;

                        if (n->getType() == IntersectionType::TrafficLight) {
                            foundSignal = true;
                            signalPos = n->getPosition();
                            break;
                        }
                    }
                }

                if (foundSignal) {
                    lane->setCustomStopPosition(signalPos);
                    // Only mark via scan if the exit node is a real junction (3+ connections).
                    // Straight-through nodes (2 connections) should not get signals from
                    // approach-road scanning — only from their own OSM node tag.
                    if (nodeConnectivity[exitNode].size() >= 3)
                        signaledIntersections.insert(lane->getExitIntersection()->getId());
                }
            }

            lane->computeCenterline();
            lane->computeTrimmedCenterline();
        }
    }

    // --- 5. Build Turn Paths & Graph ---
    InternalRoutePlanner::LaneGraph laneGraph;

    for (auto& intersection : intersections_) {
        intersection->clearLanes();
    }

    // Link lanes to the intersection they start/end at
    for (auto& [id, road] : roads_) {
        for (auto& lane : road->getLanes()) {
            if (auto* startInt = lane->getStartIntersection()) startInt->addExitLane(lane);
            if (auto* exitInt = lane->getExitIntersection()) exitInt->addEntryLane(lane);
        }
    }

    // Turns Generation Loop
    for (auto& intersection : intersections_) {
        intersection->clearPaths();
        std::unordered_map<int64_t, bool> roadHasStraight;
        for (SimulationLane* entry : intersection->getEntryLanes()) {
            int64_t rId = entry->getParentRoad()->getId();
            if (roadHasStraight.count(rId)) continue;
            WorldPosition vIn = endDir(entry->getTrimmedCenterline());
            bool straightFound = false;
            for (SimulationLane* exit : intersection->getExitLanes()) {
                if (entry->getParentRoad() == exit->getParentRoad()) continue;
                WorldPosition vOut = startDir(exit->getTrimmedCenterline());
                if (dot(vIn, vOut) > 0.7) { straightFound = true; break; }
            }
            roadHasStraight[rId] = straightFound;
        }

        for (SimulationLane* fromLane : intersection->getEntryLanes()) {
            bool connected = false;
            int fromIdx = fromLane->getLaneIndex();
            int maxFromIdx = getMaxLaneIndex(fromLane->getParentRoad(), fromLane->isForward());
            double uFrom = (maxFromIdx > 0) ? (double)fromIdx / maxFromIdx : 0.5;
            bool hasStraight = roadHasStraight[fromLane->getParentRoad()->getId()];

            SimulationLane* bestFallback = nullptr;
            double bestFallbackScore = 1e9;

            for (SimulationLane* toLane : intersection->getExitLanes()) {
                if (fromLane == toLane || fromLane->getParentRoad() == toLane->getParentRoad()) continue;
                const auto& fromPts = fromLane->getTrimmedCenterline();
                const auto& toPts = toLane->getTrimmedCenterline();
                if (fromPts.empty() || toPts.empty()) continue;

                WorldPosition pStart = fromPts.back();
                WorldPosition pEnd = toPts.front();
                WorldPosition vIn  = endDir(fromPts);
                WorldPosition vOut = startDir(toPts);

                double dp = dot(vIn, vOut);
                double cp = vIn.x * vOut.y - vIn.y * vOut.x;

                bool isStraight = (dp > 0.7);
                bool isLeft     = (!isStraight && cp > 0);
                bool isRight    = (!isStraight && cp < 0);

                int toIdx = toLane->getLaneIndex();
                int maxToIdx = getMaxLaneIndex(toLane->getParentRoad(), toLane->isForward());
                double uTo = (maxToIdx > 0) ? (double)toIdx / maxToIdx : 0.5;

                double uTolStraight = (dp > 0.92) ? 0.55 : 0.35;
                bool singleLaneEntry = (maxFromIdx == 0);

                bool allowed = false;
                if (isStraight) { if (singleLaneEntry || std::abs(uFrom - uTo) < uTolStraight) allowed = true; }
                else if (isLeft) { if (!hasStraight) { if (singleLaneEntry || std::abs(uFrom - uTo) < 0.4) allowed = true; } else { if (fromIdx == 0 && toIdx <= 1) allowed = true; } }
                else if (isRight) { if (!hasStraight) { if (singleLaneEntry || std::abs(uFrom - uTo) < 0.4) allowed = true; } else { if (fromIdx == maxFromIdx && toIdx >= maxToIdx - 1) allowed = true; } }

                // Apply turn:lanes restriction if available for this entry lane.
                if (allowed) {
                    TurnDirection dir = isStraight ? TurnDirection::Straight
                                      : (isLeft    ? TurnDirection::Left
                                                   : TurnDirection::Right);
                    auto tlIt = laneTurnAllowed.find(fromLane->getId());
                    if (tlIt != laneTurnAllowed.end() && !tlIt->second.empty()) {
                        const auto& permitted = tlIt->second;
                        allowed = std::find(permitted.begin(), permitted.end(), dir)
                                  != permitted.end();
                    }
                }

                if (allowed) connected = true;
                else {
                    double score = (1.0 - dp) + std::abs(uFrom - uTo);
                    if (score < bestFallbackScore) { bestFallbackScore = score; bestFallback = toLane; }
                }

                if (allowed) {
                    TurnDirection dir = isStraight ? TurnDirection::Straight : (isLeft ? TurnDirection::Left : TurnDirection::Right);
                    SimulationIntersectionPath path(fromLane, toLane, dir);
                    path.setupPathCurve(pStart, vIn, pEnd, vOut, 20);
                    laneGraph[fromLane].push_back(toLane);
                    intersection->addPath(std::move(path));
                }
            }

            if (!connected && bestFallback) {
                const auto& fPts = fromLane->getTrimmedCenterline();
                const auto& tPts = bestFallback->getTrimmedCenterline();
                WorldPosition vIn  = endDir(fPts);
                WorldPosition vOut = startDir(tPts);
                double dp = dot(vIn, vOut);
                double cp = vIn.x * vOut.y - vIn.y * vOut.x;
                TurnDirection dir = (dp > 0.7) ? TurnDirection::Straight : (cp > 0 ? TurnDirection::Left : TurnDirection::Right);
                SimulationIntersectionPath path(fromLane, bestFallback, dir);
                path.setupPathCurve(fPts.back(), vIn, tPts.front(), vOut, 20);
                laneGraph[fromLane].push_back(bestFallback);
                intersection->addPath(std::move(path));
            }
        }
        intersection->computeConflictPoints();
    }

    // 6. Traffic Lights (Check our new 'signaledIntersections' list!)
    for (auto& intersection : intersections_) {
        // Activate if:
        // A. The central node itself is a signal (rare for big junctions)
        // B. Any incoming road had a signal (found in Step 4)
        bool active = false;
        if (nodes_[intersection->getId()]->getType() == IntersectionType::TrafficLight) active = true;
        if (signaledIntersections.count(intersection->getId())) active = true; // <--- The Fix

        if (!active) continue;

        const auto& paths = intersection->getPaths();
        if (paths.empty()) continue;

        intersection->setSignalControlled(true);

        // Green time scaled by the best (highest-priority) road type in each phase group.
        auto greenTimeForGroup = [&](const std::vector<int>& group) -> double {
            RoadType best = RoadType::Unknown;
            for (int idx : group) {
                RoadType rt = paths[idx].getFromLane()->getParentRoad()->getRoadType();
                if (static_cast<int>(rt) < static_cast<int>(best)) best = rt;
            }
            switch (best) {
                case RoadType::Motorway:
                case RoadType::Trunk:       return 30.0;
                case RoadType::Primary:     return 25.0;
                case RoadType::Secondary:   return 20.0;
                case RoadType::Tertiary:    return 15.0;
                default:                    return 10.0;
            }
        };

        std::vector<TrafficLightPhase> phases;
        std::vector<int> verticalGroup, horizontalGroup;
        for (size_t i = 0; i < paths.size(); ++i) {
            const auto& pts = paths[i].getPathPoints();
            if (pts.empty()) continue;
            WorldPosition dir = normalize(pts.back() - pts.front());
            if (std::abs(dir.y) > std::abs(dir.x)) verticalGroup.push_back(i);
            else horizontalGroup.push_back(i);
        }

        if (!verticalGroup.empty() && !horizontalGroup.empty()) {
            phases.push_back({verticalGroup, greenTimeForGroup(verticalGroup)});
            phases.push_back({horizontalGroup, greenTimeForGroup(horizontalGroup)});
        } else {
            // Round-robin per entry road
            std::map<int64_t, std::vector<int>> pathsByEntryRoad;
            for (size_t i = 0; i < paths.size(); ++i)
                pathsByEntryRoad[paths[i].getFromLane()->getParentRoad()->getId()].push_back(i);
            for (auto& [rid, pIndices] : pathsByEntryRoad)
                phases.push_back({pIndices, greenTimeForGroup(pIndices)});
        }

        if (!phases.empty()) intersection->getController().setPhases(phases);
    }

    // Preserve the lane graph and turn-restriction map for incremental tinkering rebuilds.
    laneGraph_       = laneGraph;
    laneTurnAllowed_ = laneTurnAllowed;

    routePlanner_ = std::make_unique<InternalRoutePlanner>(std::move(laneGraph), simSeed_);
    routePlanner_->setRouteCacheTtl(routeCacheTtl_);
    if (map.contains("mapBounds")) {
        const auto& mb = map["mapBounds"];
        routePlanner_->setMapBounds(
            mb.value("minX", 0.0), mb.value("minY", 0.0),
            mb.value("maxX", 0.0), mb.value("maxY", 0.0));
    }
    routePlanner_->buildCostGraph();

    // --- Parking System initialization ---
    parkingSystem_ = std::make_unique<ParkingSystem>();

    std::vector<SimulationRoad*> allRoads;
    allRoads.reserve(roads_.size());
    for (auto& [id, road] : roads_) allRoads.push_back(road.get());
    parkingSystem_->generateSpots(allRoads, globalParameters_.defaultLaneWidth);

    // Extract building centroids from the buildings JSON data
    if (!buildingsDataString_.empty()) {
        try {
            auto bJson = json::parse(buildingsDataString_);
            std::vector<WorldPosition> centroids;
            for (const auto& bld : bJson) {
                if (bld.contains("outer") && !bld["outer"].empty() && !bld["outer"][0].empty()) {
                    double sumX = 0.0, sumY = 0.0;
                    int cnt = 0;
                    for (const auto& pt : bld["outer"][0]) {
                        if (pt.contains("x") && pt.contains("y")) {
                            sumX += pt["x"].get<double>();
                            sumY += pt["y"].get<double>();
                            ++cnt;
                        }
                    }
                    if (cnt > 0) centroids.push_back({sumX / cnt, sumY / cnt});
                }
            }
            parkingSystem_->setBuildingCentroids(std::move(centroids));
        } catch (...) {}
    }

    routePlanner_->setParkingSystem(parkingSystem_.get());

    // NOTE: seedParkedAgents() is NOT called here — vehicleTypes_ is populated
    // by addVehicleType() *after* loadMapFromOSM() returns (see main.cpp).
    // The seeding happens lazily on the first step() call once types are known.

    // ... (Spawn Regions) ...
    if (map.contains("spawnRegions")) {
         for (const auto& spawnJson : map["spawnRegions"]) {
             try {
                 SpawnRegionConfig config;
                 config.regionId = spawnJson["regionId"];
                 config.bounds.min.x = spawnJson["bounds"]["min"]["x"];
                 config.bounds.min.y = spawnJson["bounds"]["min"]["y"];
                 config.bounds.max.x = spawnJson["bounds"]["max"]["x"];
                 config.bounds.max.y = spawnJson["bounds"]["max"]["y"];
                 config.spawnRatePerSec = spawnJson["spawnRatePerSec"];
                 config.vehicleTypeId = spawnJson["vehicleTypeId"];
                 config.routePolicy = spawnJson["routePolicy"];
                 this->addSpawnRegion(config);
             } catch (...) {}
         }
     }
}


// --- Phase 2: Post-Init Map Tinkering ---

void SimulationWorld::updateNodePosition(int64_t nodeId, const WorldPosition& newPosition) {
    auto it = nodes_.find(nodeId);
    if (it == nodes_.end()) {
        std::cerr << "[Tinkering] updateNodePosition: node " << nodeId << " not found\n";
        return;
    }
    it->second->setPosition(newPosition);

    // Rebuild geometry for every road that references this node
    for (auto& [wayId, road] : roads_) {
        for (SimulationNode* n : road->getNodes()) {
            if (n->getId() == nodeId) {
                rebuildRoadGeometry(road.get());
                break;
            }
        }
    }

    // Push the updated graph to the route planner
    if (routePlanner_) routePlanner_->updateGraph(laneGraph_);
}

void SimulationWorld::updateWayProperties(int64_t wayId, const WayConfig& config) {
    auto it = roads_.find(wayId);
    if (it == roads_.end()) {
        std::cerr << "[Tinkering] updateWayProperties: way " << wayId << " not found\n";
        return;
    }
    SimulationRoad* road = it->second.get();

    if (config.speedLimit.has_value()) {
        road->setSpeedLimitMps(config.speedLimit.value());
        // Flush cache so the next Dijkstra run uses the updated cost
        if (routePlanner_) routePlanner_->invalidateCache();
    }

    if (config.numLanes.has_value()) {
        // Changing lane count requires creating/destroying SimulationLane objects and
        // re-linking intersections. Not yet implemented.
        std::cerr << "[Tinkering] updateWayProperties: numLanes change not yet supported "
                     "(requires full lane-pool rebuild)\n";
    }
    // isOneWay changes would similarly require lane pool changes.
}

void SimulationWorld::updateIntersectionTurn(int64_t intersectionId,
                                             int64_t fromWayId,
                                             int64_t toWayId,
                                             const TurnPathConfig& config) {
    SimulationIntersection* intersection = nullptr;
    for (auto& isect : intersections_) {
        if (isect->getId() == intersectionId) { intersection = isect.get(); break; }
    }
    if (!intersection) {
        std::cerr << "[Tinkering] updateIntersectionTurn: intersection " << intersectionId << " not found\n";
        return;
    }
    if (!roads_.count(fromWayId) || !roads_.count(toWayId)) {
        std::cerr << "[Tinkering] updateIntersectionTurn: from/to road not found\n";
        return;
    }

    // Rebuild all paths for this intersection (curvatureWeight support to be added to TurnPathConfig later)
    rebuildIntersectionPaths(intersection);
    if (routePlanner_) routePlanner_->updateGraph(laneGraph_);
}

void SimulationWorld::rebuildRoadGeometry(SimulationRoad* road) {
    // 1. Recompute lane centerlines
    for (auto& lane : road->getLanes()) {
        lane->computeCenterline();
        lane->computeTrimmedCenterline();
    }

    // 2. Rebuild paths for all intersections this road's lanes touch
    std::unordered_set<SimulationIntersection*> affected;
    for (auto& lane : road->getLanes()) {
        if (auto* si = lane->getStartIntersection()) affected.insert(si);
        if (auto* ei = lane->getExitIntersection()) affected.insert(ei);
    }
    for (auto* intersection : affected) {
        rebuildIntersectionPaths(intersection);
    }
    // Route planner graph update is the caller's responsibility (to batch multiple roads)
}

void SimulationWorld::rebuildIntersectionPaths(SimulationIntersection* intersection) {
    // Remove all graph edges originating from this intersection's entry lanes
    for (SimulationLane* entry : intersection->getEntryLanes()) {
        laneGraph_.erase(entry);
    }

    // Clear existing paths (do NOT call this while agents are inside the intersection)
    intersection->clearPaths();

    // Pre-compute which roads have a straight-through connection at this intersection
    std::unordered_map<int64_t, bool> roadHasStraight;
    for (SimulationLane* entry : intersection->getEntryLanes()) {
        int64_t rId = entry->getParentRoad()->getId();
        if (roadHasStraight.count(rId)) continue;
        WorldPosition vIn = endDir(entry->getTrimmedCenterline());
        bool straightFound = false;
        for (SimulationLane* exit : intersection->getExitLanes()) {
            if (entry->getParentRoad() == exit->getParentRoad()) continue;
            WorldPosition vOut = startDir(exit->getTrimmedCenterline());
            if (dot(vIn, vOut) > 0.7) { straightFound = true; break; }
        }
        roadHasStraight[rId] = straightFound;
    }

    // Rebuild turning paths (same heuristics as the initial buildGraph call)
    for (SimulationLane* fromLane : intersection->getEntryLanes()) {
        bool connected = false;
        int fromIdx = fromLane->getLaneIndex();
        int maxFromIdx = getMaxLaneIndex(fromLane->getParentRoad(), fromLane->isForward());
        double uFrom = (maxFromIdx > 0) ? (double)fromIdx / maxFromIdx : 0.5;
        bool hasStraight = roadHasStraight[fromLane->getParentRoad()->getId()];

        SimulationLane* bestFallback = nullptr;
        double bestFallbackScore = 1e9;

        for (SimulationLane* toLane : intersection->getExitLanes()) {
            if (fromLane == toLane || fromLane->getParentRoad() == toLane->getParentRoad()) continue;
            const auto& fromPts = fromLane->getTrimmedCenterline();
            const auto& toPts = toLane->getTrimmedCenterline();
            if (fromPts.size() < 2 || toPts.size() < 2) continue;

            WorldPosition pStart = fromPts.back();
            WorldPosition pEnd   = toPts.front();
            WorldPosition vIn    = endDir(fromPts);
            WorldPosition vOut   = startDir(toPts);

            double dp = dot(vIn, vOut);
            double cp = vIn.x * vOut.y - vIn.y * vOut.x;
            bool isStraight = (dp > 0.7);
            bool isLeft     = (!isStraight && cp > 0);
            bool isRight    = (!isStraight && cp < 0);

            int toIdx = toLane->getLaneIndex();
            int maxToIdx = getMaxLaneIndex(toLane->getParentRoad(), toLane->isForward());
            double uTo = (maxToIdx > 0) ? (double)toIdx / maxToIdx : 0.5;
            double uTolStraight = (dp > 0.92) ? 0.55 : 0.35;
            bool singleLaneEntry = (maxFromIdx == 0);

            bool allowed = false;
            if (isStraight)      { if (singleLaneEntry || std::abs(uFrom - uTo) < uTolStraight) allowed = true; }
            else if (isLeft)     { if (!hasStraight) { if (singleLaneEntry || std::abs(uFrom - uTo) < 0.4) allowed = true; } else { if (fromIdx == 0 && toIdx <= 1) allowed = true; } }
            else if (isRight)    { if (!hasStraight) { if (singleLaneEntry || std::abs(uFrom - uTo) < 0.4) allowed = true; } else { if (fromIdx == maxFromIdx && toIdx >= maxToIdx - 1) allowed = true; } }

            if (allowed) {
                TurnDirection dir = isStraight ? TurnDirection::Straight : (isLeft ? TurnDirection::Left : TurnDirection::Right);
                auto tlIt = laneTurnAllowed_.find(fromLane->getId());
                if (tlIt != laneTurnAllowed_.end() && !tlIt->second.empty()) {
                    const auto& permitted = tlIt->second;
                    allowed = std::find(permitted.begin(), permitted.end(), dir) != permitted.end();
                }
            }

            if (!allowed) {
                double score = (1.0 - dp) + std::abs(uFrom - uTo);
                if (score < bestFallbackScore) { bestFallbackScore = score; bestFallback = toLane; }
            } else {
                connected = true;
                TurnDirection dir = isStraight ? TurnDirection::Straight : (isLeft ? TurnDirection::Left : TurnDirection::Right);
                SimulationIntersectionPath path(fromLane, toLane, dir);
                path.setupPathCurve(pStart, vIn, pEnd, vOut, 20);
                laneGraph_[fromLane].push_back(toLane);
                intersection->addPath(std::move(path));
            }
        }

        if (!connected && bestFallback) {
            const auto& fPts = fromLane->getTrimmedCenterline();
            const auto& tPts = bestFallback->getTrimmedCenterline();
            if (fPts.size() >= 2 && tPts.size() >= 2) {
                WorldPosition vIn  = normalize(fPts.back() - fPts[fPts.size() - 2]);
                WorldPosition vOut = normalize(tPts[1] - tPts[0]);
                double dp = dot(vIn, vOut);
                double cp = vIn.x * vOut.y - vIn.y * vOut.x;
                TurnDirection dir = (dp > 0.7) ? TurnDirection::Straight : (cp > 0 ? TurnDirection::Left : TurnDirection::Right);
                SimulationIntersectionPath path(fromLane, bestFallback, dir);
                path.setupPathCurve(fPts.back(), vIn, tPts.front(), vOut, 20);
                laneGraph_[fromLane].push_back(bestFallback);
                intersection->addPath(std::move(path));
            }
        }
    }

    intersection->computeConflictPoints();
}

void SimulationWorld::setSimulationTuning(const SimulationTuning& t) {
    parkingDestProbability_ = t.parkingDestinationProbability;
    speedUpdateInterval_    = t.speedUpdateIntervalS;
    isectSnapshotInterval_  = t.isectSnapshotIntervalS;
    routeCacheTtl_          = t.routeCacheTtlS;
    agentPoolCapacity_      = t.agentPoolCapacity;

    if (routePlanner_) {
        routePlanner_->setRouteCacheTtl(routeCacheTtl_);
    }
}

void SimulationWorld::rebuildAgentGrid() const {
    agentSpatialGrid_.clear();
    for (const auto& agent : agents_) {
        if (agent.isGhost()) continue;
        WorldPosition pos = agent.getPosition();
        agentSpatialGrid_[agentCellKey(agentToCell(pos.x), agentToCell(pos.y))].push_back(
            const_cast<SimulationAgent*>(&agent));
    }
}


// --- Phase 3: Simulation Setup ---

void SimulationWorld::addVehicleType(const VehicleTypeConfig& config) {
    vehicleTypes_[config.typeId] = config;
}
void SimulationWorld::defineRoute(const RouteDefinition& route) {
    definedRoutes_[route.routeId] = route;
}
void SimulationWorld::addSpawnRegion(const SpawnRegionConfig& config) {
    spawnRegionConfigs_.push_back(config);
    spawner_->addSpawnRegion(config);
}

void SimulationWorld::scaleSpawnRates(double factor) {
    for (auto& cfg : spawnRegionConfigs_)
        cfg.spawnRatePerSec *= factor;
    spawner_->scaleAllRates(factor);
}

void SimulationWorld::addPublicTransportLine(const PublicTransportLineConfig& config) {
    // TODO: Implement public transport spawner logic
}


// --- Phase 4: Simulation Control & Execution ---

void SimulationWorld::step(double deltaTime) {
    // Logic migrated from City::update
    simTime_ += deltaTime;

    // Batched parking seed: build the queue on the first eligible step, then drain
    // SEED_BATCH_PER_STEP entries per step so the initial Dijkstra load is spread
    // across many ticks rather than spiking the first step.
    if (!hasSeededParking_ && !vehicleTypes_.empty() && parkingSystem_) {
        if (!seedingStarted_) {
            buildSeedQueue(0.5f);
            seedingStarted_ = true;
        }
        drainSeedQueue(SEED_BATCH_PER_STEP);
        if (seedCursor_ >= seedQueue_.size()) {
            hasSeededParking_ = true;
            std::cout << "[Parking] Seeding complete ("
                      << parkingSystem_->getOccupancy() << "/"
                      << parkingSystem_->getTotalSpots() << " spots occupied).\n";
        }
    }

    // 1. Spawn new agents
    spawner_->update(deltaTime);

    // 2. Update agents — two-pass to maximise parallelism.
    //
    // Pre-tag each agent with whether its current controller is thread-safe
    // (only reads shared lane structure, only writes its own fields).  This
    // snapshot must be taken BEFORE any updates begin so that agents that
    // transition to a non-thread-safe controller mid-pass are not updated
    // twice.
    for (auto& agent : agents_) {
        const auto* ctrl = agent.getController();
        agent.setParallelSafe(!agent.isGhost() && ctrl && ctrl->isThreadSafe());
    }

    // Pass 1 (parallel): LaneDrivingController agents — IDM + physics only,
    // no shared queue/lock mutations.
#ifndef TRAFFIC_NO_PARALLEL
    std::for_each(std::execution::par_unseq, agents_.begin(), agents_.end(),
        [&](SimulationAgent& a) {
            if (a.isParallelSafe()) {
                a.setAgentSimTime(simTime_);
                a.update(deltaTime);
            }
        });
#endif

    // Pass 2 (serial): intersection, lane-change, and parking controllers —
    // these write to shared intersection lock maps or agent queues.
    // Also covers the TRAFFIC_NO_PARALLEL fallback (all agents serial).
    for (auto& agent : agents_) {
#ifdef TRAFFIC_NO_PARALLEL
        agent.setAgentSimTime(simTime_);
        agent.update(deltaTime);
#else
        if (!agent.isParallelSafe()) {
            agent.setAgentSimTime(simTime_);
            agent.update(deltaTime);
        }
#endif
    }

    // 2b. Periodically sample per-road average speeds and push to the route planner
    // so that Dijkstra reroutes avoid congested roads.
    if (routePlanner_) routePlanner_->advanceSimTime(deltaTime);
    speedUpdateTimer_ += deltaTime;
    if (speedUpdateTimer_ >= speedUpdateInterval_ && routePlanner_) {
        speedUpdateTimer_ = 0.0;
        std::unordered_map<SimulationRoad*, std::pair<double, int>> acc;
        for (const auto& agent : agents_) {
            if (agent.isGhost() || agent.isParked() || agent.isInIntersection()) continue;
            const SimulationLane* lane = agent.getCurrentLane();
            if (!lane) continue;
            auto& entry = acc[lane->getParentRoad()];
            entry.first  += agent.getSpeed();
            entry.second += 1;
        }
        std::unordered_map<SimulationRoad*, double> avgSpeeds;
        avgSpeeds.reserve(acc.size());
        for (auto& [road, entry] : acc) {
            if (entry.second > 0)
                avgSpeeds[road] = entry.first / entry.second;
        }
        routePlanner_->updateRoadSpeeds(avgSpeeds);
    }

    for (auto& intersection : intersections_) intersection->update(deltaTime);

    // 3. Remove completed agents
    purgeCompletedAgents();

    // 4. Trim throughput window to last 60 sim-seconds
    const double window = 60.0;
    while (!completionTimestamps_.empty() && completionTimestamps_.front() < simTime_ - window) {
        completionTimestamps_.pop_front();
    }

    // 5. Periodic intersection snapshots for analytics (only when a run is active)
    if (metricsCollector_ && !activeRunId_.empty()) {
        isectSnapshotTimer_ += deltaTime;
        if (isectSnapshotTimer_ >= isectSnapshotInterval_) {
            isectSnapshotTimer_ = 0.0;
            for (const auto& isect : intersections_) {
                int lockedCount = static_cast<int>(
                    isect->getController().getLocks().size());
                if (lockedCount >= 2) {
                    metricsCollector_->recordIntersectionSnapshot(
                        activeRunId_, simTime_, isect->getId(), lockedCount);
                }
            }
        }
    }

    // Note: agentSpatialGrid_ is rebuilt lazily inside getVehicleStatesInBounds
    // when the query bounds are small enough to use it. No rebuild here.
}


// --- Phase 5: Data-Out (Querying State) ---

[[nodiscard]] std::string SimulationWorld::getMapDataJSON() const {
    using json = nlohmann::json;
    json mapData;
    mapData["lanes"] = json::array();
    mapData["intersections"] = json::array(); // Changed from intersection_paths to support the object structure
    mapData["intersection_paths"] = json::array(); // Keep this for the paths themselves
    mapData["spawn_regions"] = json::array();

    // 1. LANES
    for (const auto& [id, road] : roads_) {
        for (const auto& lane : road->getLanes()) {
            json laneJson;
            laneJson["id"] = lane->getId();
            laneJson["speedLimitMps"] = road->getSpeedLimitMps();
            laneJson["points"] = json::array();
            for (const auto& p : lane->getTrimmedCenterline()) {
                laneJson["points"].push_back({{"x", p.x}, {"y", p.y}});
            }
            mapData["lanes"].push_back(laneJson);
        }
    }

    // 2. INTERSECTIONS & CONFLICTS
    for (const auto& intersection : intersections_) {
        json interJson;
        interJson["id"] = intersection->getId();
        interJson["x"] = intersection->getPosition().x;
        interJson["y"] = intersection->getPosition().y;

        // EXPORT THE SIGNAL FLAG so frontend can draw boxes if needed
        // We override the OSM node type if we auto-detected it as a signal
        if (intersection->isSignalControlled()) {
            interJson["type"] = (int)IntersectionType::TrafficLight;
        } else {
            interJson["type"] = (int)nodes_.at(intersection->getId())->getType();
        }

        // Add the conflict points here, nested inside the intersection
        interJson["conflicts"] = json::array();
        const auto& paths = intersection->getPaths();
        for(const auto& c : intersection->getConflictPoints()) {
            // Calculate indices based on memory address offset
            // This works because paths are stored in a std::vector
            long idxA = static_cast<const SimulationIntersectionPath*>(c.pathA) - paths.data();
            long idxB = static_cast<const SimulationIntersectionPath*>(c.pathB) - paths.data();

            interJson["conflicts"].push_back({
                {"id", c.id},
                {"x", c.position.x},
                {"y", c.position.y},
                {"pA", idxA},
                {"pB", idxB}
            });
        }
        interJson["paths"] = json::array();
        for (const auto& path : intersection->getPaths()) {
            const auto& pts = path.getPathPoints();
            if (pts.empty()) continue;

            // We only need the start point to draw the traffic light bar
            interJson["paths"].push_back({
                {"x", pts.front().x},
                {"y", pts.front().y}
            });
        }

        mapData["intersections"].push_back(interJson);

        // 3. PATHS (Existing logic)
        for (const auto& path : intersection->getPaths()) {
            json pathJson;
            pathJson["from_lane"] = path.getFromLane()->getId();
            pathJson["to_lane"] = path.getToLane()->getId();
            pathJson["points"] = json::array();
            for (const auto& p : path.getPathPoints()) {
                pathJson["points"].push_back({{"x", p.x}, {"y", p.y}});
            }
            mapData["intersection_paths"].push_back(pathJson);
        }
    }

    // 4. SPAWN REGIONS
    for (const auto& region : spawnRegionConfigs_) {
        json regionJson;
        regionJson["id"] = region.regionId;
        regionJson["min"] = {{"x", region.bounds.min.x}, {"y", region.bounds.min.y}};
        regionJson["max"] = {{"x", region.bounds.max.x}, {"y", region.bounds.max.y}};
        mapData["spawn_regions"].push_back(regionJson);
    }

    if (!rawOsmDataString_.empty()) {
        mapData["rawOsm"] = json::parse(rawOsmDataString_);
    }

    if (!buildingsDataString_.empty()) {
        mapData["buildings"] = json::parse(buildingsDataString_);
    }

    if (!waysDataString_.empty()) {
        mapData["ways"] = json::parse(waysDataString_);
    }

    if (!nodesDataString_.empty()) {
        mapData["osmNodes"] = json::parse(nodesDataString_);
    }

    return mapData.dump();
}

std::string SimulationWorld::getIntersectionPositionsJSON() const {
    std::ostringstream out;
    out << '[';
    bool first = true;
    for (const auto& inter : intersections_) {
        if (!first) out << ',';
        first = false;
        const auto& pos = inter->getPosition();
        out << "{\"id\":" << inter->getId()
            << ",\"x\":" << pos.x
            << ",\"y\":" << pos.y << '}';
    }
    out << ']';
    return out.str();
}

[[nodiscard]] SimulationMetrics SimulationWorld::getMetrics() const {
    SimulationMetrics metrics;
    metrics.totalVehicles      = static_cast<int64_t>(agents_.size());
    metrics.totalSpawned       = totalSpawned_;
    metrics.totalCompleted     = totalCompleted_;
    metrics.avgTravelTime      = avgTravelTime_;
    metrics.totalExcessSeconds = totalExcessSeconds_;

    double  totalSpeed = 0.0;
    double  totalGap   = 0.0;
    int64_t gapCount   = 0;

    for (const auto& agent : agents_) {
        const double speed = agent.getSpeed();
        totalSpeed += speed;
        if (speed > metrics.maxSpeed)  metrics.maxSpeed = speed;
        if (speed < 1.0)               metrics.vehiclesInQueue++;
        if (agent.isInIntersection())  metrics.vehiclesInIntersection++;
        if (agent.isYielding())        metrics.yieldingVehicles++;

        const double gap = agent.getGapToLeader();
        if (std::isfinite(gap)) { totalGap += gap; gapCount++; }
    }

    if (!agents_.empty())
        metrics.avgSpeed = totalSpeed / static_cast<double>(agents_.size());
    if (gapCount > 0)
        metrics.avgGapToLeader = totalGap / static_cast<double>(gapCount);

    // throughput: completions in last 60s window, scaled to per-minute
    const double elapsed = std::min(simTime_, 60.0);
    if (elapsed > 0.0)
        metrics.throughputPerMinute =
            (static_cast<double>(completionTimestamps_.size()) / elapsed) * 60.0;

    return metrics;
}

[[nodiscard]] std::vector<VehicleState> SimulationWorld::getVehicleStatesInBounds(
    const BoundingBox& bounds) const {

    std::vector<VehicleState> states;

    auto buildState = [&](const SimulationAgent& agent) {
        WorldPosition pos = agent.getPosition();
        VehicleState state;
        state.id           = agent.getId();
        state.typeId       = agent.getTypeId();
        state.pos          = pos;
        state.heading      = agent.getHeading();
        state.speed        = agent.getSpeed();
        state.acceleration = agent.getAcceleration();
        state.leaderId     = agent.getLeaderId();
        state.gapToLeader  = agent.getGapToLeader();
        state.laneId       = agent.getCurrentLane() ? agent.getCurrentLane()->getId() : -1;
        state.parked       = agent.isParked();
        if (agent.isParked()) {
            double rem = agent.getDepartureSimTime() - simTime_;
            state.timeUntilDeparture = rem > 0.0 ? rem : 0.0;
        }
        states.push_back(state);
    };

    // Compute cell range for the query bounds.
    const int cxMin = agentToCell(bounds.min.x);
    const int cyMin = agentToCell(bounds.min.y);
    const int cxMax = agentToCell(bounds.max.x);
    const int cyMax = agentToCell(bounds.max.y);

    // If the query is very large (e.g. global -1e9..1e9 used by the sim loop snapshot),
    // the cell range would be enormous and the nested loop would be catastrophically slow.
    // Fall back to a direct linear scan over the agents vector in that case.
    const int64_t cellSpan = (int64_t)(cxMax - cxMin + 1) * (int64_t)(cyMax - cyMin + 1);
    const int64_t GRID_SCAN_THRESHOLD = 1000000; // ~1M cells ≈ 50km × 50km at 50m/cell

    if (cellSpan > GRID_SCAN_THRESHOLD) {
        // Query covers the full world (e.g. snapshot build) — linear scan is faster
        // than grid lookup and avoids the rebuild cost entirely.
        for (const auto& agent : agents_) {
            WorldPosition pos = agent.getPosition();
            if (pos.x >= bounds.min.x && pos.x <= bounds.max.x &&
                pos.y >= bounds.min.y && pos.y <= bounds.max.y)
            {
                buildState(agent);
            }
        }
        return states;
    }

    // Grid-accelerated query: O(cells_in_range * avg_agents_per_cell)
    // Rebuild the grid lazily — only when we're actually going to use it.
    rebuildAgentGrid();
    for (int cx = cxMin; cx <= cxMax; ++cx) {
        for (int cy = cyMin; cy <= cyMax; ++cy) {
            auto it = agentSpatialGrid_.find(agentCellKey(cx, cy));
            if (it == agentSpatialGrid_.end()) continue;
            for (const SimulationAgent* agent : it->second) {
                WorldPosition pos = agent->getPosition();
                if (pos.x >= bounds.min.x && pos.x <= bounds.max.x &&
                    pos.y >= bounds.min.y && pos.y <= bounds.max.y)
                {
                    buildState(*agent);
                }
            }
        }
    }
    return states;
}

[[nodiscard]] std::optional<DetailedVehicleState> SimulationWorld::getDetailedVehicleState(
    int64_t vehicleId) const {

    for (const auto& agent : agents_) {
        if (agent.getId() == vehicleId) {
            DetailedVehicleState state;
            state.id = agent.getId();
            state.typeId = agent.getTypeId();
            state.pos = agent.getPosition();
            state.heading = agent.getHeading();
            state.speed = agent.getSpeed();
            state.leaderId    = agent.getLeaderId();
            state.gapToLeader = agent.getGapToLeader();
            state.timeHeadway = (state.speed > 1e-3) ? (state.gapToLeader / state.speed)
                                             : std::numeric_limits<double>::infinity();
            state.controllerState = agent.getControllerState();
            state.routeIndex = agent.getRouteIndex();

            if (agent.getCurrentLane()) {
                state.currentLaneId = std::to_string(agent.getCurrentLane()->getId());
            } else if (agent.isInIntersection()) {
                state.currentLaneId = "Intersection";
            } else {
                state.currentLaneId = "None";
            }
            state.acceleration = agent.getAcceleration();
            state.isYielding = agent.isYielding();
            state.yieldingToId = agent.getYieldingToId();
            state.conflictPointPos = agent.getConflictPointPos();
            state.debugHitbox = agent.getDebugHitbox();
            state.blockedReason   = agent.getBlockedReason();
            state.pathProgress    = agent.pathProgress();
            state.laneLength      = agent.getCurrentLane()
                ? agent.getCurrentLane()->getTrimmedCenterlineLength() : 0.0;
            state.inIntersection  = agent.isInIntersection();

            // --- Route polyline: remaining lanes from current routeIndex ---
            {
                const auto& route = agent.getRoute();
                int idx = agent.getRouteIndex();
                // First lane: include only points from current progress onward
                if (idx < (int)route.size() && route[idx]) {
                    const auto& pts = route[idx]->getTrimmedCenterline();
                    double prog = agent.pathProgress();
                    double acc  = 0.0;
                    bool started = false;
                    for (size_t i = 0; i + 1 < pts.size(); ++i) {
                        double segLen = length(pts[i+1] - pts[i]);
                        if (!started && acc + segLen >= prog) {
                            // Interpolate the exact starting point
                            double t = (prog - acc) / (segLen > 1e-9 ? segLen : 1.0);
                            WorldPosition p = pts[i] + (pts[i+1] - pts[i]) * t;
                            state.routePolyline.push_back(p);
                            started = true;
                        }
                        if (started) state.routePolyline.push_back(pts[i+1]);
                        acc += segLen;
                    }
                    if (!started && !pts.empty())
                        state.routePolyline.push_back(pts.back());
                }
                // Remaining lanes: full centerlines
                for (int i = idx + 1; i < (int)route.size(); ++i) {
                    if (!route[i]) continue;
                    const auto& pts = route[i]->getTrimmedCenterline();
                    for (const auto& p : pts) state.routePolyline.push_back(p);
                }
            }

            // --- Parked state ---
            state.isParked = agent.isParked();
            if (agent.isParked()) {
                double remaining = agent.getDepartureSimTime() - simTime_;
                state.timeUntilDeparture = remaining > 0.0 ? remaining : 0.0;
            }

            // --- Life history ---
            state.history = agent.getHistory();

            // --- Parking info ---
            if (agent.isParkingSearchMode()) {
                state.isParkingRoute = true;
                double bRadius = agent.getParkingBuildingRadius();
                if (bRadius > 0.0) {
                    // Show the real building destination with its search radius.
                    state.parkingDest         = agent.getParkingDestBuilding();
                    state.parkingSearchRadius = bRadius;
                } else {
                    // Fallback: midpoint of the last route lane.
                    const auto& route = agent.getRoute();
                    if (!route.empty() && route.back()) {
                        const auto& pts = route.back()->getTrimmedCenterline();
                        if (!pts.empty()) {
                            size_t mid = pts.size() / 2;
                            state.parkingDest = pts[mid];
                        }
                    }
                    state.parkingSearchRadius = 60.0;
                }
            } else if (agent.getTargetSpot()) {
                state.isParkingRoute      = true;
                state.parkingDest         = agent.getTargetSpot()->position;
                state.parkingSearchRadius = 8.0; // small ring around the claimed spot
            }

            return state;
        }
    }
    return std::nullopt;
}

[[nodiscard]] std::vector<HeatmapCell> SimulationWorld::getVehicleHeatmap(
    const BoundingBox& bounds,
    const int gridResolutionX,
    const int gridResolutionY) const
{
    // --- 1. Create and initialize the 2D grid ---

    // We need a temporary grid to store the sum of speeds before we can average.
    struct TempGridCell {
        int vehicleCount = 0;
        double speedSum = 0.0;
        BoundingBox bounds;
    };

    const int totalCells = gridResolutionX * gridResolutionY;
    std::vector<TempGridCell> grid(totalCells);

    // Calculate the size of each cell
    const double cellWidth = (bounds.max.x - bounds.min.x) / static_cast<double>(gridResolutionX);
    const double cellHeight = (bounds.max.y - bounds.min.y) / static_cast<double>(gridResolutionY);

    // Pre-calculate the bounds for each cell
    for (int y = 0; y < gridResolutionY; ++y) {
        for (int x = 0; x < gridResolutionX; ++x) {
            const int index = y * gridResolutionX + x;
            TempGridCell& cell = grid[index];

            cell.bounds.min.x = bounds.min.x + static_cast<double>(x) * cellWidth;
            cell.bounds.min.y = bounds.min.y + static_cast<double>(y) * cellHeight;
            cell.bounds.max.x = cell.bounds.min.x + cellWidth;
            cell.bounds.max.y = cell.bounds.min.y + cellHeight;
        }
    }

    // --- 2. Bin all agents into the grid ---

    // Pre-calculate inverse cell size for faster index lookup
    const double invCellWidth = (cellWidth > 0) ? 1.0 / cellWidth : 0;
    const double invCellHeight = (cellHeight > 0) ? 1.0 / cellHeight : 0;

    for (const auto& agent : agents_) {
        const WorldPosition pos = agent.getPosition();

        // Check if agent is within the total bounds
        if (pos.x < bounds.min.x || pos.x >= bounds.max.x ||
            pos.y < bounds.min.y || pos.y >= bounds.max.y)
        {
            continue; // Agent is outside the requested heatmap area
        }

        // Calculate which cell the agent falls into
        const int x = static_cast<int>((pos.x - bounds.min.x) * invCellWidth);
        const int y = static_cast<int>((pos.y - bounds.min.y) * invCellHeight);
        const int index = y * gridResolutionX + x;
        if(index >= 0 && index < totalCells) {
            grid[index].vehicleCount++;
            grid[index].speedSum += agent.getSpeed();
        }
    }
    std::vector<HeatmapCell> result;
    result.reserve(totalCells); // Reserve space, though we'll only add non-empty cells

    for (const auto& tempCell : grid) {
        if (tempCell.vehicleCount > 0) {
            // This cell has agents in it, so add it to the result
            HeatmapCell finalCell;
            finalCell.bounds = tempCell.bounds;
            finalCell.vehicleCount = tempCell.vehicleCount;
            finalCell.averageSpeed = tempCell.speedSum / static_cast<double>(tempCell.vehicleCount);
            result.push_back(finalCell);
        }
    }
    return result;
}


[[nodiscard]] std::vector<LaneTrafficStat> SimulationWorld::getLaneTrafficStats() const {
    struct Acc { int count = 0; int stuck = 0; double speedSum = 0.0; };
    std::unordered_map<int64_t, Acc> acc;

    for (const auto& agent : agents_) {
        if (agent.isGhost()) continue;
        SimulationLane* lane = agent.getCurrentLane();
        if (!lane) continue; // agent is inside an intersection
        const double spd = agent.getSpeed();
        auto& a = acc[lane->getId()];
        a.count++;
        a.speedSum += spd;
        if (spd < 1.0) a.stuck++;
    }

    std::vector<LaneTrafficStat> result;
    result.reserve(acc.size());
    for (const auto& [id, a] : acc) {
        result.push_back({
            id,
            a.count,
            a.stuck,
            a.count > 0 ? a.speedSum / static_cast<double>(a.count) : 0.0
        });
    }
    return result;
}

// --- Phase 6: Runtime Interaction ---

void SimulationWorld::setVehicleRoute(int64_t vehicleId, const std::string& routeId) {
    // TODO: Implement
    // 1. Find agent by ID
    // 2. Get new route from definedRoutes_
    // 3. Convert list of Way IDs to list of SimulationLane*
    // 4. Set the agent's route
}

void SimulationWorld::setRoadClosed(int64_t wayId, bool isClosed) {
    auto it = roads_.find(wayId);
    if (it == roads_.end()) return;
    SimulationRoad* road = it->second.get();
    road->setIsClosed(isClosed);

    if (routePlanner_) {
        routePlanner_->setRoadClosed(road, isClosed);
    }

    // Reroute any agent whose current lane OR any future route waypoint is on the closed road
    if (isClosed) {
        for (auto& agent : agents_) {
            SimulationLane* current = agent.getCurrentLane();
            if (!current) continue;

            bool needsReroute = (current->getParentRoad() == road);

            if (!needsReroute) {
                const auto& route = agent.getRoute();
                int idx = agent.getRouteIndex();
                for (int i = idx; i < static_cast<int>(route.size()); ++i) {
                    if (route[i]->getParentRoad() == road) {
                        needsReroute = true;
                        break;
                    }
                }
            }

            if (needsReroute) {
                agent.regenerateRoute(current);
            }
        }
    }
}

void SimulationWorld::setLaneClosed(int64_t wayId, int laneIdx, bool isClosed) {
    auto it = roads_.find(wayId);
    if (it == roads_.end()) return;
    SimulationRoad* road = it->second.get();

    SimulationLane* targetLane = nullptr;
    for (SimulationLane* lane : road->getLanes()) {
        if (lane->getLaneIndex() == laneIdx) {
            targetLane = lane;
            break;
        }
    }
    if (!targetLane) return;
    targetLane->setIsClosed(isClosed);

    // Reroute agents currently on this lane
    if (isClosed) {
        for (auto& agent : agents_) {
            SimulationLane* cur = agent.getCurrentLane();
            if (cur && cur == targetLane) {
                agent.regenerateRoute(cur);
            }
        }
    }

    // Update route planner: if ALL lanes on the road are now closed, block the road;
    // if at least one is open, unblock (in case we're re-opening after a full closure).
    if (routePlanner_) {
        bool anyOpen = false;
        for (SimulationLane* lane : road->getLanes()) {
            if (!lane->isClosed()) { anyOpen = true; break; }
        }
        routePlanner_->setRoadClosed(road, !anyOpen);
    }
}

void SimulationWorld::setSeed(uint64_t seed) {
    simSeed_ = seed;
    if (spawner_) spawner_->setSeed(seed);
    // routePlanner_ is (re-)created in initialize() using simSeed_, so no action needed here.
}


// --- Private Helper Methods ---

[[nodiscard]] std::vector<SimulationLane*> SimulationWorld::getStartLanesInBounds(const BoundingBox& bounds) const
{
    std::vector<SimulationLane*> startLanes;
    for (const auto& [id, road] : roads_) {
        for (SimulationLane* lane : road->getLanes()) {
            if (lane->getStartIntersection() != nullptr) {
                continue;
            }
            const auto& path = lane->getTrimmedCenterline();
            if (path.empty()) {
                continue;
            }
            const WorldPosition& startPos = path.front();
            if (startPos.x >= bounds.min.x && startPos.x <= bounds.max.x &&
                startPos.y >= bounds.min.y && startPos.y <= bounds.max.y)
            {
                startLanes.push_back(lane);
            }
        }
    }
    return startLanes;
}

void SimulationWorld::purgeCompletedAgents() {
    // Swap-and-pop: move the last agent into the dead slot so the vector stays
    // dense without shifting every subsequent element.  Only one fixup per removal.
    for (std::size_t i = 0; i < agents_.size(); ) {
        SimulationAgent& agent = agents_[i];
        if (!agent.isCompleted()) { ++i; continue; }

        // 1. Release any parking spot held by this agent
        if (parkingSystem_) {
            if (ParkingSpot* spot = agent.getParkedSpot()) {
                parkingSystem_->releaseSpot(spot->id);
            }
            if (ParkingSpot* spot = agent.getTargetSpot()) {
                parkingSystem_->releaseSpot(spot->id);
                agent.setTargetSpot(nullptr);
            }
        }

        // 2. Deregister from Lane/Path Queues
        if (agent.isInIntersection() && agent.getActivePath()) {
            agent.getActivePath()->getAgentQueue().remove(&agent);
        } else if (agent.getCurrentLane()) {
            agent.getCurrentLane()->getAgentQueue().remove(&agent);
        }

        // 2. Release any intersection locks held by this agent
        for (auto& intersection : intersections_) {
            intersection->getController().releaseAllForAgent(agent.getId());
        }

        // 3. Metrics tracking
        totalCompleted_++;
        completionTimestamps_.push_back(simTime_);
        auto spawnIt = spawnTimes_.find(agent.getId());
        if (spawnIt != spawnTimes_.end()) {
            const SpawnRecord& rec = spawnIt->second;
            double travelTime = simTime_ - rec.spawnTime;
            // Count reroutes from history events
            int rerouteCount = 0;
            for (const auto& ev : agent.getHistory())
                if (ev.type == AgentHistoryEvent::Type::RouteRegenerated) ++rerouteCount;
            // Record to DB if a run is active
            if (metricsCollector_ && !activeRunId_.empty()) {
                metricsCollector_->recordTrip(activeRunId_,
                    agent.getId(),
                    rec.spawnTime,
                    simTime_,
                    travelTime,
                    rec.freeflowEstimate,
                    rerouteCount);
            }
            spawnTimes_.erase(spawnIt);
            completedWithTimes_++;
            avgTravelTime_ += (travelTime - avgTravelTime_) / completedWithTimes_;
            totalExcessSeconds_      += std::max(0.0, travelTime - rec.freeflowEstimate);
            totalCompletedTravelSum_ += travelTime;
        }

        // 4. Swap-and-pop: overwrite dead slot with the last live agent
        const std::size_t last = agents_.size() - 1;
        if (i != last) {
            SimulationAgent* oldPtr = &agents_[last];
            agents_[i] = std::move(agents_[last]);
            // Fix self-referential waypoints_ (lane-change owned path)
            agents_[i].fixSelfWaypointsAfterMove(oldPtr);
            // Fix raw pointers held by neighbors and owning queue
            fixMovedAgent(oldPtr, &agents_[i]);
        }
        agents_.pop_back();
        // Don't advance i — the swapped-in agent now occupies slot i and must be checked
    }
}

void SimulationWorld::fixMovedAgent(SimulationAgent* oldPtr, SimulationAgent* newPtr) {
    // Update the intrusive linked-list neighbors
    if (SimulationAgent* prev = newPtr->getPrevInLane())
        prev->setNextInLane(newPtr);
    if (SimulationAgent* next = newPtr->getNextInLane())
        next->setPrevInLane(newPtr);

    // Update the owning queue's head/tail if this agent sits at either end
    SimulationAgentQueue* queue = nullptr;
    if (newPtr->isInIntersection() && newPtr->getActivePath())
        queue = &newPtr->getActivePath()->getAgentQueue();
    else if (newPtr->getCurrentLane())
        queue = &newPtr->getCurrentLane()->getAgentQueue();

    if (queue)
        queue->replacePointer(oldPtr, newPtr);
}

/**
 * @brief Checks if an agent can be spawned on a lane.
 * Logic migrated from City::canSpawn.
 */
void SimulationWorld::setSpawningEnabled(bool e) { if (spawner_) spawner_->setEnabled(e); }
bool SimulationWorld::isSpawningEnabled() const { return spawner_ ? spawner_->isEnabled() : false; }

void SimulationWorld::setMetricsCollector(MetricsCollector* collector, const std::string& runId) {
    metricsCollector_ = collector;
    activeRunId_      = runId;
    isectSnapshotTimer_ = 0.0;
}
std::vector<InternalSpawnSystem::RegionInfo> SimulationWorld::getSpawnRegionInfo() const {
    return spawner_ ? spawner_->getRegionInfo() : std::vector<InternalSpawnSystem::RegionInfo>{};
}

// ── Trip log recording/replay ─────────────────────────────────────────────────

void SimulationWorld::setTripLogRecording(TripLog* log) {
    tripLogOut_ = log;
}

void SimulationWorld::setTripLogReplay(const TripLog* log) {
    replayDestByLane_.clear();
    excludedCount_ = 0;
    laneById_.clear();
    if (!log || log->empty()) return;

    // Build laneById_ from all roads so we can look up lanes by stable ID.
    for (const auto& [wayId, road] : roads_) {
        for (SimulationLane* lane : road->getLanes()) {
            laneById_[lane->getId()] = lane;
        }
    }

    // Partition entries: closed spawn lanes → excluded immediately;
    // open spawn lanes → queued for replay in spawnNewAgent.
    for (const auto& rec : *log) {
        auto it = laneById_.find(rec.spawnLaneId);
        if (it == laneById_.end()) continue;
        SimulationLane* spawnLane = it->second;
        if (spawnLane->isClosed() || spawnLane->getParentRoad()->isClosed()) {
            excludedCount_++;
        } else {
            replayDestByLane_[rec.spawnLaneId].push_back(rec.destLaneId);
        }
    }
}

[[nodiscard]] bool SimulationWorld::canSpawn(SimulationLane* lane, double minGap) const {
    if (lane->isClosed() || lane->getParentRoad()->isClosed()) return false;
    // Look at the last car on the lane (tail of the queue)
    SimulationAgent* agentInFront = lane->getAgentQueue().getTail();
    if (!agentInFront) {
        return true; // Lane is empty
    }

    // Check gap
    double gap = agentInFront->pathProgress(); // Agent's distance from start of lane
    double needed = agentInFront->getLength() + minGap;
    return (gap >= needed);
}

void SimulationWorld::spawnNewAgent(SimulationLane* startLane, int64_t agentId) {
    // 1. Get a route
    if (!routePlanner_) return;

    bool wantsParkingRoute = false;
    WorldPosition buildingPos = {0.0, 0.0};
    double        buildingRadius = 0.0;
    std::vector<SimulationLane*> route;

    // ── Replay mode: use forced destination from baseline trip log ────────────
    if (!replayDestByLane_.empty()) {
        auto qIt = replayDestByLane_.find(startLane->getId());
        if (qIt != replayDestByLane_.end() && !qIt->second.empty()) {
            int64_t destLaneId = qIt->second.front();
            qIt->second.pop_front();
            auto dIt = laneById_.find(destLaneId);
            if (dIt != laneById_.end()) {
                route = routePlanner_->generateRouteTo(startLane, dIt->second);
            }
            if (route.empty()) {
                // Destination unreachable after closures → excluded, don't spawn.
                excludedCount_++;
                return;
            }
            // Route found — fall through to spawn logic below with this route.
        } else {
            // No queued entry for this lane (extra spawn in candidate vs baseline) → normal routing.
            route = routePlanner_->generateRoute(startLane);
            if (route.empty()) return;
        }
    } else {
        // ── Normal / recording mode: standard route generation ────────────────
        const auto& buildings = parkingSystem_ ? parkingSystem_->getBuildingCentroids()
                                               : std::vector<WorldPosition>{};
        if (parkingSystem_ && !buildings.empty()) {
            float roll = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
            if (roll < static_cast<float>(parkingDestProbability_)) {
                route = routePlanner_->generateParkingRoute(startLane, nullptr,
                                                            &buildingPos, &buildingRadius);
                wantsParkingRoute = !route.empty() && buildingRadius > 0.0;
            }
        }
        if (route.empty()) {
            route = routePlanner_->generateRoute(startLane);
        }
        if (route.empty()) return;

        // Recording mode: append this spawn's trip record.
        if (tripLogOut_) {
            tripLogOut_->push_back({startLane->getId(), route.back()->getId()});
        }
    }

    // 2. Get vehicle config
    // TODO: This should be passed down from the spawner
    const std::string typeId = "default_car";
    if (vehicleTypes_.find(typeId) == vehicleTypes_.end()) {
        // No vehicle type configured
        return;
    }
    const VehicleTypeConfig& config = vehicleTypes_.at(typeId);

    // 3. Guard against pool exhaustion — addresses must remain stable
    if (agents_.size() >= agentPoolCapacity_) {
        return; // silently skip; spawn rate will naturally back off
    }

    // 4. Construct in-place. The constructor registers itself with startLane's queue
    //    using 'this', which is valid because the vector will not reallocate.
    agents_.emplace_back(agentId, typeId, route, config.maxSpeed, 0.0, routePlanner_.get());
    SimulationAgent& ag = agents_.back();
    ag.setParkingSystem(parkingSystem_.get());
    ag.addHistoryEvent(AgentHistoryEvent::Type::Spawned, simTime_,
                       wantsParkingRoute ? "parking route" : "normal route");
    if (wantsParkingRoute) {
        ag.setParkingSearchMode(true);
        ag.setParkingDestBuilding(buildingPos);
        ag.setParkingBuildingRadius(buildingRadius);
        ag.addHistoryEvent(AgentHistoryEvent::Type::StartedParkingSearch, simTime_,
                           "heading to building");
    }

    // 5. Metrics tracking
    totalSpawned_++;
    // Compute free-flow estimate: Σ(trimmed_length / speed_limit) over the initial route
    double freeflow = 0.0;
    for (const auto* lane : route) {
        double spd = lane->getParentRoad()->getSpeedLimitMps();
        if (spd > 0.0)
            freeflow += lane->getTrimmedCenterlineLength() / spd;
    }
    totalSpawnedFreeflowSum_ += freeflow;
    spawnTimes_[agentId] = {simTime_, freeflow};
}

void SimulationWorld::buildSeedQueue(float fraction) {
    seedQueue_.clear();
    seedCursor_ = 0;
    if (!parkingSystem_) return;
    const auto& spots = parkingSystem_->getSpots();
    seedQueue_.reserve(spots.size());
    for (int i = 0; i < static_cast<int>(spots.size()); ++i) {
        if (!spots[i].adjacentLane) continue;
        float r = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
        if (r <= fraction) seedQueue_.push_back(i);
    }
}

void SimulationWorld::drainSeedQueue(int batchSize) {
    if (!parkingSystem_ || !routePlanner_) return;
    const std::string typeId = "default_car";
    if (vehicleTypes_.find(typeId) == vehicleTypes_.end()) return;
    const VehicleTypeConfig& cfg = vehicleTypes_.at(typeId);
    const auto& spots = parkingSystem_->getSpots();

    const size_t end = std::min(seedCursor_ + static_cast<size_t>(batchSize), seedQueue_.size());
    for (; seedCursor_ < end; ++seedCursor_) {
        if (agents_.size() >= agentPoolCapacity_ - 100) {
            seedCursor_ = seedQueue_.size(); // pool full — skip remaining
            return;
        }
        const ParkingSpot& spotRef = spots[seedQueue_[seedCursor_]];

        auto route = routePlanner_->generateRoute(spotRef.adjacentLane);
        if (route.empty()) continue;

        int64_t agentId = spawner_->getNextAgentId();
        if (!parkingSystem_->claimSpot(spotRef.id, agentId)) continue;

        ParkingSpot* spot = parkingSystem_->getSpot(spotRef.id);
        if (!spot) continue;

        agents_.emplace_back(agentId, typeId, route, cfg.maxSpeed, 0.0, routePlanner_.get());
        SimulationAgent& ag = agents_.back();
        ag.setParkingSystem(parkingSystem_.get());

        double departTime = (static_cast<double>(std::rand()) / RAND_MAX) * 28800.0;
        ag.park(spot, departTime);
        ag.setController(std::make_unique<ParkingDepartureController>(
            spot->adjacentLane, spot->laneProgress, spot->position, spot->heading));

        totalSpawned_++;
        spawnTimes_[agentId] = {simTime_, 0.0};
    }
}

std::string SimulationWorld::getParkingSpotsJSON() const {
    json result;
    json spotsArr = json::array();
    if (parkingSystem_) {
        for (const auto& spot : parkingSystem_->getSpots()) {
            spotsArr.push_back({
                {"id",          spot.id},
                {"x",           spot.position.x},
                {"y",           spot.position.y},
                {"heading",     spot.heading},
                {"orientation", static_cast<int>(spot.orientation)},
                {"occupied",    spot.isOccupied()}
            });
        }
        result["total"]     = parkingSystem_->getTotalSpots();
        result["occupancy"] = parkingSystem_->getOccupancy();
    } else {
        result["total"]     = 0;
        result["occupancy"] = 0;
    }
    result["spots"] = std::move(spotsArr);
    return result.dump();
}

std::vector<IntersectionStateData> SimulationWorld::getIntersectionStates() const {
    std::vector<IntersectionStateData> states;
    for (const auto& intersection : intersections_) {
        IntersectionStateData state;
        state.id = intersection->getId();

        // Locks
        const auto& locks = intersection->getController().getLocks();
        for (const auto& [cId, agentId] : locks) {
            state.lockedConflicts.push_back({cId, agentId});
        }

        // Check the explicit flag we set in Step 6
        if (intersection->isSignalControlled()) {
            const auto& paths = intersection->getPaths();
            for (size_t i = 0; i < paths.size(); ++i) {
                LightState s = intersection->getController().getLightState(i);
                state.lightStates.push_back({(int)i, (int)s});
            }
        }

        states.push_back(state);
    }
    return states;
}