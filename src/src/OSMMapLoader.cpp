/*
 * Note: Claude (Anthropic) was used to assist with commenting this file —
 * clarifying what the code physically does and documenting non-obvious decisions.
 * The implementation logic was written by the author (unless stated otherwise).
 */
#include "../include/OSMMapLoader.hpp"
#include "../include/SimulationTypes.hpp" // For Enums
#include <tinyxml2.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <unordered_map>
#include <map>
#include <set>
#include <cmath>
#include <algorithm>
#include <vector>
#include <deque>

using json = nlohmann::json;

// --- Internal Data Types ---
struct NodeInfo {
    double lat;
    double lon;
    IntersectionType type;
};

struct ProjectedPos { double x; double y; };

struct RoadSegment {
    int64_t from;
    int64_t to;
    int lanesFwd;
    int lanesBwd;
    int64_t originalWayId;
    RoadType roadType;
    bool visited = false;
};

/* [AI-generated: Claude] */
// --- Helper: Math ---
static ProjectedPos latLonToWorld(const double lat, const double lon, const double refLat, const double refLon) {
    constexpr double R = 6378137.0;
    constexpr double DEG2RAD = 3.1415926535 / 180.0;
    double x = (lon - refLon) * (DEG2RAD * R * std::cos(refLat * DEG2RAD));
    double y = (lat - refLat) * (DEG2RAD * R);
    return {x, y};
}
/* [end AI-generated] */

// --- Helper: Parse Node Tags ---
static IntersectionType parseNodeTags(const tinyxml2::XMLElement* el) {
    bool isSignal = false;
    bool isStop = false;
    bool isCrossing = false;

    for (auto* tag = el->FirstChildElement("tag"); tag; tag = tag->NextSiblingElement("tag")) {
        std::string k = tag->Attribute("k");
        std::string v = tag->Attribute("v");

        if (k == "highway") {
            if (v == "traffic_signals") isSignal = true;
            else if (v == "stop") isStop = true;
            else if (v == "crossing") isCrossing = true;
        }
        if (k == "crossing") isCrossing = true;
        if (k == "crossing_ref") isCrossing = true;
    }

    // Filter: Ignore pedestrian crossings
    if (isCrossing) return IntersectionType::None;

    if (isStop) return IntersectionType::Stop;
    if (isSignal) return IntersectionType::TrafficLight;

    return IntersectionType::None;
}

// --- Helper: Parse maxspeed tag → km/h (0 = not specified) ---
static double parseMaxspeedKph(const std::string& v) {
    if (v.empty() || v == "none" || v == "unlimited" || v == "signals") return 0.0;
    // Country-code values like "FR:urban", "DE:living_street" — ignore
    if (v.find(':') != std::string::npos) return 0.0;
    // "XX mph" — convert to km/h
    auto mphPos = v.find("mph");
    if (mphPos != std::string::npos) {
        try { return std::stod(v.substr(0, mphPos)) * 1.60934; } catch (...) {}
        return 0.0;
    }
    // Plain integer assumed km/h
    try { return std::stod(v); } catch (...) {}
    return 0.0;
}

// --- Helper: Parse Way Tags ---
// Road-type spawn rate weight: higher-order roads carry proportionally more traffic.
static double roadTypeSpawnWeight(RoadType rt) {
    switch (rt) {
        case RoadType::Motorway:    return 1.0;
        case RoadType::Trunk:       return 1.0;
        case RoadType::Primary:     return 0.9;
        case RoadType::Secondary:   return 0.7;
        case RoadType::Tertiary:    return 0.4;
        case RoadType::Residential: return 0.15;
        default:                    return 0.1;
    }
}

static RoadType parseRoadType(const std::string& v) {
    if (v == "motorway" || v == "motorway_link") return RoadType::Motorway;
    if (v == "trunk" || v == "trunk_link") return RoadType::Trunk;
    if (v == "primary" || v == "primary_link") return RoadType::Primary;
    if (v == "secondary" || v == "secondary_link") return RoadType::Secondary;
    if (v == "tertiary" || v == "tertiary_link") return RoadType::Tertiary;
    if (v == "residential" || v == "living_street") return RoadType::Residential;
    return RoadType::Unknown;
}

std::string OSMMapLoader::parseOSMtoJSON(const std::string& osmXmlData) {
    tinyxml2::XMLDocument doc;
    if (doc.Parse(osmXmlData.c_str()) != tinyxml2::XML_SUCCESS) return "{}";
    auto* root = doc.RootElement();
    if (!root) return "{}";

    // 1. Load Nodes
    std::unordered_map<int64_t, NodeInfo> allNodes;
    double minLat = 90, maxLat = -90, minLon = 180, maxLon = -180;

    for (auto* el = root->FirstChildElement("node"); el; el = el->NextSiblingElement("node")) {
        int64_t id = el->Int64Attribute("id");
        double lat = el->DoubleAttribute("lat");
        double lon = el->DoubleAttribute("lon");

        IntersectionType type = parseNodeTags(el);

        allNodes[id] = {lat, lon, type};
        if (lat < minLat) minLat = lat; if (lat > maxLat) maxLat = lat;
        if (lon < minLon) minLon = lon; if (lon > maxLon) maxLon = lon;
    }
    double refLat = (minLat + maxLat) / 2.0;
    double refLon = (minLon + maxLon) / 2.0;

    // Parse <bounds> for spawn-region filtering.
    // When a <bounds> element is present it defines the download rectangle; degree-1
    // nodes strictly inside it are interior dead-ends and are skipped as spawn points.
    // When it is absent (maps downloaded in sections, JOSM exports, etc.) we cannot
    // reliably distinguish edge stubs from interior dead-ends, so we allow all of them.
    double boundsMinLat = minLat, boundsMaxLat = maxLat;
    double boundsMinLon = minLon, boundsMaxLon = maxLon;
    bool hasBoundsTag = false;
    if (auto* boundsEl = root->FirstChildElement("bounds")) {
        boundsMinLat = boundsEl->DoubleAttribute("minlat", minLat);
        boundsMaxLat = boundsEl->DoubleAttribute("maxlat", maxLat);
        boundsMinLon = boundsEl->DoubleAttribute("minlon", minLon);
        boundsMaxLon = boundsEl->DoubleAttribute("maxlon", maxLon);
        hasBoundsTag = true;
    }
    auto boundsMinW = latLonToWorld(boundsMinLat, boundsMinLon, refLat, refLon);
    auto boundsMaxW = latLonToWorld(boundsMaxLat, boundsMaxLon, refLat, refLon);

    std::unordered_map<int64_t, ProjectedPos> worldNodes;
    for (auto& [id, info] : allNodes) {
        worldNodes[id] = latLonToWorld(info.lat, info.lon, refLat, refLon);
    }

    // --- Keep track of all raw ways & buildings ---
    std::unordered_map<int64_t, std::vector<int64_t>> allRawWays;
    struct RawBuilding {
        int64_t id;
        std::vector<std::vector<int64_t>> outers;
        std::vector<std::vector<int64_t>> inners;
    };
    std::vector<RawBuilding> buildings;

    // 2. Parse Ways
    std::map<int64_t, std::vector<RoadSegment>> graph;
    std::vector<RoadSegment> allSegments;
    // Maps original OSM way ID → (turnLanesForward, turnLanesBackward) raw pipe strings
    std::unordered_map<int64_t, std::pair<std::string,std::string>> wayTurnLanes;
    // Maps original OSM way ID → (parkingLeft, parkingRight) value strings
    std::unordered_map<int64_t, std::pair<std::string,std::string>> wayParkingLanes;
    // Maps original OSM way ID → speed limit in km/h (0 = not specified)
    std::unordered_map<int64_t, double> waySpeedLimits;
    // Maps original OSM way ID → layer (-1 tunnel, 0 normal, 1+ bridge)
    std::unordered_map<int64_t, int> wayLayers;
    // Maps original OSM way ID → OSM name tag
    std::map<int64_t, std::string> wayNames;

    const std::set<std::string> highwayWhitelist = {
        "motorway", "trunk", "primary", "secondary", "tertiary",
        "unclassified", "residential", "motorway_link", "trunk_link",
        "primary_link", "secondary_link", "tertiary_link", "living_street"
    };

    int64_t nextWayId = 1;

    for (auto* way = root->FirstChildElement("way"); way; way = way->NextSiblingElement("way")) {
        int64_t wayId = way->Int64Attribute("id");
        bool isCar = false;
        bool isOneWay = false;
        bool isBuilding = false;
        int lanes = 1;
        int lanesFwdTag = -1;   // -1 = not present in OSM data
        int lanesBwdTag = -1;
        RoadType rType = RoadType::Unknown;
        std::string turnLanesFwd;   // "left|through|right" — OSM turn:lanes / turn:lanes:forward
        std::string turnLanesBwd;   // turn:lanes:backward
        std::string parkingLeft;    // OSM parking:lane:left value
        std::string parkingRight;   // OSM parking:lane:right value
        double speedLimitKph = 0.0; // OSM maxspeed tag (0 = not specified)
        int wayLayer = 0;           // OSM layer tag (negative = tunnel, positive = bridge)
        bool isBridge = false;
        bool isTunnel = false;
        std::string wayName;        // OSM name tag

        for (auto* tag = way->FirstChildElement("tag"); tag; tag = tag->NextSiblingElement("tag")) {
            std::string k = tag->Attribute("k");
            std::string v = tag->Attribute("v");
            if (k == "layer") { try { wayLayer = std::stoi(v); } catch(...) {} }
            if (k == "bridge" && v == "yes") isBridge = true;
            if (k == "tunnel" && v == "yes") isTunnel = true;
            if (k == "highway") {
                if (highwayWhitelist.count(v)) {
                    isCar = true;
                    rType = parseRoadType(v);
                    if (rType == RoadType::Primary || rType == RoadType::Trunk) lanes = 2;
                    if (rType == RoadType::Motorway) lanes = 3;
                }
            }
            if (k == "oneway" && v == "yes") isOneWay = true;
            if (k == "lanes")          { try { lanes       = std::max(1, std::stoi(v)); } catch(...) {} }
            if (k == "lanes:forward")  { try { lanesFwdTag = std::max(0, std::stoi(v)); } catch(...) {} }
            if (k == "lanes:backward") { try { lanesBwdTag = std::max(0, std::stoi(v)); } catch(...) {} }
            if (k == "building") isBuilding = true;
            if (k == "turn:lanes" || k == "turn:lanes:forward") turnLanesFwd = v;
            if (k == "turn:lanes:backward") turnLanesBwd = v;
            if (k == "maxspeed") speedLimitKph = parseMaxspeedKph(v);
            // Parking lane tags
            if (k == "parking:lane:left")  parkingLeft  = v;
            if (k == "parking:lane:right") parkingRight = v;
            if (k == "parking:lane:both")  { parkingLeft = v; parkingRight = v; }
            // Orientation subtags override if present
            if (k == "parking:lane:left:orientation")  parkingLeft  = v;
            if (k == "parking:lane:right:orientation") parkingRight = v;
            if (k == "name") wayName = v;
        }

        std::vector<int64_t> ids;
        for (auto* nd = way->FirstChildElement("nd"); nd; nd = nd->NextSiblingElement("nd")) {
            ids.push_back(nd->Int64Attribute("ref"));
        }
        if (ids.size() < 2) continue;

        // Save raw way for relations
        allRawWays[wayId] = ids;

        // If it's a simple building, save it as a polygon with 1 outer ring
        if (isBuilding) {
            buildings.push_back({wayId, {ids}, {}});
        }

        if (!isCar) continue;

        if (!turnLanesFwd.empty() || !turnLanesBwd.empty())
            wayTurnLanes[wayId] = {turnLanesFwd, turnLanesBwd};

        if (!parkingLeft.empty() || !parkingRight.empty())
            wayParkingLanes[wayId] = {parkingLeft, parkingRight};

        if (speedLimitKph > 0.0)
            waySpeedLimits[wayId] = speedLimitKph;

        // Derive layer from bridge/tunnel tags if no explicit layer is set
        if (wayLayer == 0) {
            if (isBridge) wayLayer = 1;
            else if (isTunnel) wayLayer = -1;
        }
        if (wayLayer != 0)
            wayLayers[wayId] = wayLayer;

        // Derive per-direction lane counts.
        // Priority: lanes:forward/lanes:backward > symmetric split of lanes.
        // For odd totals the extra lane goes to the forward direction.
        int fwd, bwd;
        if (isOneWay) {
            fwd = lanes;
            bwd = 0;
        } else if (lanesFwdTag >= 0 && lanesBwdTag >= 0) {
            fwd = std::max(1, lanesFwdTag);
            bwd = std::max(1, lanesBwdTag);
        } else if (lanesFwdTag >= 0) {
            fwd = std::max(1, lanesFwdTag);
            bwd = std::max(1, lanes - lanesFwdTag);
        } else if (lanesBwdTag >= 0) {
            bwd = std::max(1, lanesBwdTag);
            fwd = std::max(1, lanes - lanesBwdTag);
        } else {
            // No directional tags: split evenly, bias the odd lane to forward.
            int half = lanes / 2;          // rounds down
            bwd = std::max(1, half);
            fwd = std::max(1, lanes - half);
        }

        for (size_t i = 0; i < ids.size() - 1; ++i) {
            RoadSegment seg;
            seg.from = ids[i];
            seg.to = ids[i+1];
            seg.lanesFwd = fwd;
            seg.lanesBwd = bwd;
            seg.originalWayId = wayId;
            seg.roadType = rType;
            allSegments.push_back(seg);
            graph[seg.from].push_back(seg);
            graph[seg.to].push_back(seg);
        }
        if (!wayName.empty()) wayNames[wayId] = wayName;
    }

    // --- 2.1 Parse Relations (Complex Multipolygon Buildings) ---
    for (auto* rel = root->FirstChildElement("relation"); rel; rel = rel->NextSiblingElement("relation")) {
        bool isMultipolygon = false;
        bool isBuilding = false;
        for (auto* tag = rel->FirstChildElement("tag"); tag; tag = tag->NextSiblingElement("tag")) {
            std::string k = tag->Attribute("k");
            std::string v = tag->Attribute("v");
            if (k == "type" && v == "multipolygon") isMultipolygon = true;
            if (k == "building") isBuilding = true;
        }

        if (isMultipolygon && isBuilding) {
            RawBuilding b;
            b.id = rel->Int64Attribute("id");
            for (auto* mem = rel->FirstChildElement("member"); mem; mem = mem->NextSiblingElement("member")) {
                std::string type = mem->Attribute("type") ? mem->Attribute("type") : "";
                if (type != "way") continue;

                int64_t ref = mem->Int64Attribute("ref");
                std::string role = mem->Attribute("role") ? mem->Attribute("role") : "";

                if (allRawWays.count(ref)) {
                    if (role == "inner") b.inners.push_back(allRawWays[ref]);
                    else b.outers.push_back(allRawWays[ref]); // Default to outer
                }
            }
            if (!b.outers.empty()) buildings.push_back(b);
        }
    }

    // FIXME: 1. Identify true topological junctions
    std::unordered_map<int64_t, std::set<int64_t>> nodeWays;
    for (const auto& seg : allSegments) {
        nodeWays[seg.from].insert(seg.originalWayId);
        nodeWays[seg.to].insert(seg.originalWayId);
    }

    std::vector<int64_t> junctionNodes;
    for (const auto& [id, ways] : nodeWays) {
        if (ways.size() > 1) {
            junctionNodes.push_back(id);
        }
    }

    // FIXME: 2. Cluster ONLY the junctions that are very close to each other
    const double CLUSTER_RADIUS = 6.0;
    std::map<int64_t, int64_t> alias;
    auto getAlias = [&](int64_t id) {
        int64_t root = id;
        while (alias.count(root) && alias[root] != root) root = alias[root];
        int64_t curr = id;
        while (alias.count(curr) && alias[curr] != curr) {
            int64_t nxt = alias[curr];
            alias[curr] = root;
            curr = nxt;
        }
        return root;
    };

    for (size_t i = 0; i < junctionNodes.size(); ++i) {
        for (size_t j = i + 1; j < junctionNodes.size(); ++j) {
            int64_t u = getAlias(junctionNodes[i]);
            int64_t v = getAlias(junctionNodes[j]);
            if (u == v) continue;

            ProjectedPos p1 = worldNodes[u];
            ProjectedPos p2 = worldNodes[v];
            double dx = p1.x - p2.x, dy = p1.y - p2.y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist < CLUSTER_RADIUS) {
                alias[v] = u;
                worldNodes[u].x = (p1.x + p2.x) / 2.0;
                worldNodes[u].y = (p1.y + p2.y) / 2.0;
            }
        }
    }

    // 3. Apply aliases to reconstruct the graph safely
    std::vector<RoadSegment> cleanedSegments;
    graph.clear();

    for (auto& seg : allSegments) {
        if (alias.count(seg.from)) seg.from = getAlias(seg.from);
        if (alias.count(seg.to))   seg.to   = getAlias(seg.to);

        if (seg.from != seg.to) {
            cleanedSegments.push_back(seg);
            graph[seg.from].push_back(seg);
            graph[seg.to].push_back(seg);
        }
    }
    allSegments = cleanedSegments;

    // Helper: split a pipe-delimited string into a JSON array.
    auto pipeSplit = [](const std::string& s) {
        json arr = json::array();
        std::string tok;
        for (char c : s) {
            if (c == '|') { arr.push_back(tok); tok.clear(); }
            else tok += c;
        }
        arr.push_back(tok);
        return arr;
    };

    // 3. Identify Split Nodes
    std::set<int64_t> splitNodes;
    for (const auto& [nodeId, segments] : graph) {
        if (segments.size() != 2) {
            splitNodes.insert(nodeId);
            continue;
        }
        const auto& s1 = segments[0];
        const auto& s2 = segments[1];
        if (s1.lanesFwd != s2.lanesFwd || s1.lanesBwd != s2.lanesBwd) {
            splitNodes.insert(nodeId);
            continue;
        }

        IntersectionType t = allNodes[nodeId].type;
        if (t == IntersectionType::Stop) {
            splitNodes.insert(nodeId);
        }

        int64_t n1 = (s1.from == nodeId) ? s1.to : s1.from;
        int64_t n2 = (s2.from == nodeId) ? s2.to : s2.from;
        ProjectedPos pNode = worldNodes[nodeId];
        ProjectedPos p1 = worldNodes[n1];
        ProjectedPos p2 = worldNodes[n2];

        double dx1 = p1.x - pNode.x, dy1 = p1.y - pNode.y;
        double dx2 = p2.x - pNode.x, dy2 = p2.y - pNode.y;
        double len1 = std::sqrt(dx1*dx1 + dy1*dy1);
        double len2 = std::sqrt(dx2*dx2 + dy2*dy2);

        if (len1 > 0.1 && len2 > 0.1) {
            dx1 /= len1; dy1 /= len1;
            dx2 /= len2; dy2 /= len2;
            double dot = dx1*dx2 + dy1*dy2;
            if (dot > -0.70) splitNodes.insert(nodeId);  // split only if bend > ~45°
        }
    }

    // 4. Stitching
    json mapJson;
    mapJson["nodes"] = json::array();
    mapJson["ways"] = json::array();
    std::set<int64_t> exportedNodes;
    std::set<std::pair<int64_t, int64_t>> processed;

    for (const auto& startSeg : allSegments) {
        int64_t u = std::min(startSeg.from, startSeg.to);
        int64_t v = std::max(startSeg.from, startSeg.to);
        if (processed.count({u,v})) continue;

        std::deque<int64_t> wayNodes;
        wayNodes.push_back(startSeg.from);
        wayNodes.push_back(startSeg.to);
        processed.insert({u,v});

        int lanesF = startSeg.lanesFwd;
        int lanesB = startSeg.lanesBwd;

        // FWD — track the last absorbed segment's wayId so its turn:lanes apply at the
        // forward-end intersection rather than startSeg's (which may be mid-road).
        int64_t fwdTlWayId = startSeg.originalWayId;
        int64_t curr = startSeg.to;
        int64_t prev = startSeg.from;
        while (splitNodes.find(curr) == splitNodes.end()) {
            const auto& adj = graph[curr];
            bool found = false;
            for (const auto& nextSeg : adj) {
                int64_t nextNode = (nextSeg.from == curr) ? nextSeg.to : nextSeg.from;
                if (nextNode != prev) {
                    int64_t nu = std::min(nextSeg.from, nextSeg.to);
                    int64_t nv = std::max(nextSeg.from, nextSeg.to);
                    processed.insert({nu,nv});
                    fwdTlWayId = nextSeg.originalWayId;
                    wayNodes.push_back(nextNode);
                    prev = curr;
                    curr = nextNode;
                    found = true;
                    break;
                }
            }
            if (!found) break;
        }

        // BWD — track the last absorbed segment's wayId so its turn:lanes apply at the
        // backward-end intersection.
        int64_t bwdTlWayId = startSeg.originalWayId;
        curr = startSeg.from;
        prev = startSeg.to;
        while (splitNodes.find(curr) == splitNodes.end()) {
            const auto& adj = graph[curr];
            bool found = false;
            for (const auto& nextSeg : adj) {
                int64_t nextNode = (nextSeg.from == curr) ? nextSeg.to : nextSeg.from;
                if (nextNode != prev) {
                    int64_t nu = std::min(nextSeg.from, nextSeg.to);
                    int64_t nv = std::max(nextSeg.from, nextSeg.to);
                    processed.insert({nu,nv});
                    bwdTlWayId = nextSeg.originalWayId;
                    wayNodes.push_front(nextNode);
                    prev = curr;
                    curr = nextNode;
                    found = true;
                    break;
                }
            }
            if (!found) break;
        }

        json wayObj;
        wayObj["id"] = nextWayId++;
        wayObj["nodes"] = wayNodes;
        wayObj["lanesForward"] = lanesF;
        wayObj["lanesBackward"] = lanesB;
        wayObj["type"] = (int)startSeg.roadType;

        // Turn:lanes come from whichever OSM way sits at each end of the merged road:
        // forward TL from the last FWD-absorbed segment, backward TL from the last BWD-absorbed
        // segment. When no extension happened in a direction, that remains startSeg.originalWayId.
        {
            auto fwdIt = wayTurnLanes.find(fwdTlWayId);
            if (fwdIt != wayTurnLanes.end() && !fwdIt->second.first.empty())
                wayObj["turnLanesForward"] = pipeSplit(fwdIt->second.first);
            auto bwdIt = wayTurnLanes.find(bwdTlWayId);
            if (bwdIt != wayTurnLanes.end() && !bwdIt->second.second.empty())
                wayObj["turnLanesBackward"] = pipeSplit(bwdIt->second.second);
        }

        // Attach parking lane data from the originating OSM way (if any).
        auto plIt = wayParkingLanes.find(startSeg.originalWayId);
        if (plIt != wayParkingLanes.end()) {
            if (!plIt->second.first.empty())
                wayObj["parkingLeft"]  = plIt->second.first;
            if (!plIt->second.second.empty())
                wayObj["parkingRight"] = plIt->second.second;
        }

        // Attach road name from the originating OSM way (if any).
        auto nmIt = wayNames.find(startSeg.originalWayId);
        if (nmIt != wayNames.end())
            wayObj["name"] = nmIt->second;

        // Attach speed limit from the originating OSM way (if any).
        auto slIt = waySpeedLimits.find(startSeg.originalWayId);
        if (slIt != waySpeedLimits.end())
            wayObj["speedLimitKph"] = slIt->second;

        auto lyIt = wayLayers.find(startSeg.originalWayId);
        if (lyIt != wayLayers.end())
            wayObj["layer"] = lyIt->second;

        mapJson["ways"].push_back(wayObj);

        for (int64_t id : wayNodes) exportedNodes.insert(id);
    }

    // 5. Export Nodes
    for (int64_t id : exportedNodes) {
        ProjectedPos p = worldNodes[id];
        mapJson["nodes"].push_back({
            {"id", id},
            {"pos", {{"x", p.x}, {"y", p.y}}},
            {"type", static_cast<int>(allNodes[id].type)}
        });
    }

    // --- 6. AUTO-GENERATE SPAWN REGIONS ---
    mapJson["spawnRegions"] = json::array();

    // Build map of Node -> Connected Ways
    struct Conn { json* way; bool isStart; };
    std::map<int64_t, std::vector<Conn>> connectivity;

    for (auto& way : mapJson["ways"]) {
        if (way["nodes"].empty()) continue;
        int64_t startId = way["nodes"].front();
        int64_t endId = way["nodes"].back();

        connectivity[startId].push_back({&way, true});
        connectivity[endId].push_back({&way, false});
    }

    // Identify Edge Nodes (Degree 1)
    int spawnCount = 0;
    for (const auto& [nodeId, conns] : connectivity) {
        if (conns.size() == 1) {
            const auto& c = conns[0];
            bool canSpawn = false;

            int fwd = c.way->value("lanesForward", 0);
            int bwd = c.way->value("lanesBackward", 0);

            // Check if traffic can flow FROM this node INTO the map
            if (c.isStart) {
                // If node is START, we need lanes going Forward (Start->End)
                if (fwd > 0) canSpawn = true;
            } else {
                // If node is END, we need lanes going Backward (End->Start)
                if (bwd > 0) canSpawn = true;
            }

            if (canSpawn) {
                ProjectedPos p = worldNodes[nodeId];
                // When the OSM file has an explicit <bounds> element, skip degree-1
                // nodes strictly inside it — those are interior dead-ends, not real
                // network entry points.  Without <bounds> we cannot make this
                // distinction, so all degree-1 nodes are treated as edge stubs.
                if (hasBoundsTag &&
                    p.x > boundsMinW.x && p.x < boundsMaxW.x &&
                    p.y > boundsMinW.y && p.y < boundsMaxW.y) {
                    continue;
                }
                RoadType wayRoadType = static_cast<RoadType>(c.way->value("type", (int)RoadType::Unknown));
                double spawnRate = 0.3 * roadTypeSpawnWeight(wayRoadType);

                json region;
                region["regionId"] = "spawn_edge_" + std::to_string(nodeId);
                // 10x10m box around the node
                region["bounds"]["min"] = {{"x", p.x - 5.0}, {"y", p.y - 5.0}};
                region["bounds"]["max"] = {{"x", p.x + 5.0}, {"y", p.y + 5.0}};
                // Config
                region["spawnRatePerSec"] = spawnRate;
                region["vehicleTypeId"] = "default_car";
                region["routePolicy"] = "random_global";

                mapJson["spawnRegions"].push_back(region);
                spawnCount++;
            }
        }
    }

    // --- ADD BUILDINGS EXPORT ---
    mapJson["buildings"] = json::array();
    auto processRings = [&worldNodes](const std::vector<std::vector<int64_t>>& rings) {
        json ringsArr = json::array();
        for (const auto& ring : rings) {
            json ringPts = json::array();
            for (int64_t nid : ring) {
                if (worldNodes.count(nid)) {
                    ringPts.push_back({{"x", worldNodes[nid].x}, {"y", worldNodes[nid].y}});
                }
            }
            if (!ringPts.empty()) ringsArr.push_back(ringPts);
        }
        return ringsArr;
    };

    for (const auto& b : buildings) {
        json bJson;
        bJson["id"] = b.id;
        bJson["outer"] = processRings(b.outers);
        bJson["inner"] = processRings(b.inners);
        mapJson["buildings"].push_back(bJson);
    }

    std::cout << "[OSM Loader] Processed. Exporting " << mapJson["ways"].size() << " roads." << std::endl;
    std::cout << "[OSM Loader] Exported " << buildings.size() << " buildings." << std::endl;
    std::cout << "[OSM Loader] Generated " << spawnCount << " spawn regions at map edges." << std::endl;

    // --- 7. EXPORT RAW OSM DATA FOR DEBUGGING ---
    json rawOsm;
    rawOsm["nodes"] = json::object();
    for (const auto& [id, pos] : worldNodes) {
        rawOsm["nodes"][std::to_string(id)] = {
            {"x", pos.x},
            {"y", pos.y},
            {"type", (int)allNodes[id].type}
        };
    }

    rawOsm["segments"] = json::array();
    for (const auto& seg : allSegments) {
        rawOsm["segments"].push_back({
            {"from", std::to_string(seg.from)},
            {"to", std::to_string(seg.to)},
            {"wayId", std::to_string(seg.originalWayId)},
            {"roadType", (int)seg.roadType}
        });
    }

    rawOsm["wayNames"] = json::object();
    for (const auto& [id, name] : wayNames)
        rawOsm["wayNames"][std::to_string(id)] = name;

    mapJson["rawOsm"] = rawOsm;
    mapJson["mapOrigin"] = {{"lat", refLat}, {"lon", refLon}};

    // --- 8. EXPORT MAP BOUNDS (world coordinates) ---
    mapJson["mapBounds"] = {
        {"minX", boundsMinW.x}, {"minY", boundsMinW.y},
        {"maxX", boundsMaxW.x}, {"maxY", boundsMaxW.y}
    };

    return mapJson.dump();
}