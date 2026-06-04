#include "../include/ParkingSystem.hpp"
#include "../include/SimulationLane.hpp"
#include "../include/SimulationRoad.hpp"
#include "../include/MathHelpers.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

// Walk a polyline to get position at given progress distance.
// Returns the world point and fills tangentOut with the direction at that point.
static WorldPosition pointAtProgress(const std::vector<WorldPosition>& pts,
                                     double progress,
                                     WorldPosition& tangentOut)
{
    if (pts.size() < 2) {
        tangentOut = {1.0, 0.0};
        return pts.empty() ? WorldPosition{0,0} : pts[0];
    }

    double acc = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        WorldPosition seg = pts[i+1] - pts[i];
        double segLen = length(seg);
        if (segLen < 1e-9) continue;
        if (acc + segLen >= progress) {
            double t = (progress - acc) / segLen;
            tangentOut = normalize(seg);
            return pts[i] + seg * t;
        }
        acc += segLen;
    }
    // At end
    size_t n = pts.size();
    tangentOut = normalize(pts[n-1] - pts[n-2]);
    return pts[n-1];
}

void ParkingSystem::generateSpotsForLane(SimulationLane* lane,
                                          ParkingOrientation orientation,
                                          bool rightSide,
                                          double laneWidth)
{
    const auto& pts = lane->getTrimmedCenterline();
    if (pts.size() < 2) return;

    double totalLen = lane->getTrimmedCenterlineLength();
    if (totalLen <= 0.0) return;

    const double MARGIN = 10.0; // don't place within 10m of lane start/end FIXME: Derive something better then fixed margin

    // Spacing between spot centers along the lane
    double interval = 0.0;
    double depthOffset = 0.0;

    switch (orientation) {
        case ParkingOrientation::Parallel:
            interval    = PARALLEL_SPOT_LENGTH + SPOT_GAP;
            depthOffset = PARALLEL_SPOT_DEPTH / 2.0;
            break;
        case ParkingOrientation::Diagonal: {
            // Spot footprint width along the road = DIAGONAL_SPOT_WIDTH / cos(45deg)
            double widthAlongRoad = DIAGONAL_SPOT_WIDTH / std::cos(DIAGONAL_ANGLE_RAD);
            interval    = widthAlongRoad + SPOT_GAP;
            depthOffset = DIAGONAL_SPOT_DEPTH / 2.0;
            break;
        }
        case ParkingOrientation::Perpendicular:
            interval    = PERP_SPOT_WIDTH + SPOT_GAP;
            depthOffset = PERP_SPOT_DEPTH / 2.0;
            break;
        default:
            return;
    }

    for (double progress = MARGIN; progress <= totalLen - MARGIN; progress += interval) {
        WorldPosition tangent{1.0, 0.0};
        WorldPosition lanePoint = pointAtProgress(pts, progress, tangent);

        // Right-perpendicular (clockwise 90 deg from direction of travel)
        WorldPosition perpRight = {tangent.y, -tangent.x};

        // Which side to offset to
        WorldPosition sideDir = rightSide ? perpRight : WorldPosition{-perpRight.x, -perpRight.y};

        // Spot center offset: half lane width + some gap + depth
        double totalOffset = (laneWidth / 2.0) + depthOffset;
        WorldPosition spotCenter = lanePoint + sideDir * totalOffset;

        // Compute heading
        double heading = 0.0;
        switch (orientation) {
            case ParkingOrientation::Parallel:
                heading = std::atan2(tangent.y, tangent.x);
                break;
            case ParkingOrientation::Diagonal:
                // 45 degrees from lane direction, angled toward curb
                if (rightSide)
                    heading = std::atan2(tangent.y, tangent.x) - DIAGONAL_ANGLE_RAD;
                else
                    heading = std::atan2(tangent.y, tangent.x) + DIAGONAL_ANGLE_RAD;
                break;
            case ParkingOrientation::Perpendicular:
                // 90 degrees from lane direction
                if (rightSide)
                    heading = std::atan2(tangent.y, tangent.x) - (M_PI / 2.0);
                else
                    heading = std::atan2(tangent.y, tangent.x) + (M_PI / 2.0);
                break;
            default:
                break;
        }

        ParkingSpot spot;
        spot.id           = nextSpotId_++;
        spot.position     = spotCenter;
        spot.heading      = heading;
        spot.orientation  = orientation;
        spot.adjacentLane = lane;
        spot.laneProgress = progress;
        spot.occupantId   = -1;

        spots_.push_back(spot);
        spotById_[spot.id] = &spots_.back();
    }
}

void ParkingSystem::generateSpots(const std::vector<SimulationRoad*>& roads, double laneWidth)
{
    spots_.clear();
    spotById_.clear();
    parkingLanes_.clear();
    nextSpotId_ = 0;

    std::unordered_set<SimulationLane*> seenLanes;

    for (SimulationRoad* road : roads) {
        if (!road) continue;

        ParkingOrientation parkLeft  = road->getParkingLeft();
        ParkingOrientation parkRight = road->getParkingRight();

        // Fallback: residential and service roads get parallel parking on both sides
        // when no explicit OSM parking tag is present (most maps lack these tags!).
        if (parkLeft == ParkingOrientation::None && parkRight == ParkingOrientation::None) {
            RoadType rt = road->getRoadType();
            if (rt == RoadType::Residential || rt == RoadType::Service) {
                parkLeft  = ParkingOrientation::Parallel;
                parkRight = ParkingOrientation::Parallel;
            }
        }

        // Right side of road = forward lanes (isForward==true), laneIndex 0 (outermost/curb lane)
        if (parkRight != ParkingOrientation::None) {
            SimulationLane* curbLane = nullptr;
            for (SimulationLane* lane : road->getLanes()) {
                if (lane->isForward() && lane->getLaneIndex() == 0) {
                    curbLane = lane;
                    break;
                }
            }
            if (curbLane && seenLanes.find(curbLane) == seenLanes.end()) {
                generateSpotsForLane(curbLane, parkRight, /*rightSide=*/true, laneWidth);
                seenLanes.insert(curbLane);
                parkingLanes_.push_back(curbLane);
            }
        }

        // Left side of road = backward lanes (isForward==false), laneIndex 0
        // When traveling backward, laneIndex 0 is on the right side of travel
        // which corresponds to the left side of the overall road.
        if (parkLeft != ParkingOrientation::None) {
            SimulationLane* curbLane = nullptr;
            for (SimulationLane* lane : road->getLanes()) {
                if (!lane->isForward() && lane->getLaneIndex() == 0) {
                    curbLane = lane;
                    break;
                }
            }
            if (curbLane && seenLanes.find(curbLane) == seenLanes.end()) {
                generateSpotsForLane(curbLane, parkLeft, /*rightSide=*/true, laneWidth);
                seenLanes.insert(curbLane);
                parkingLanes_.push_back(curbLane);
            }
        }
    }
}

void ParkingSystem::setBuildingCentroids(std::vector<WorldPosition> centroids)
{
    buildingCentroids_ = std::move(centroids);
}

SimulationLane* ParkingSystem::findNearestParkingLane(WorldPosition pos, double maxRadius) const
{
    SimulationLane* best = nullptr;
    double bestDist2 = maxRadius * maxRadius;

    for (SimulationLane* lane : parkingLanes_) {
        const auto& pts = lane->getTrimmedCenterline();
        if (pts.empty()) continue;

        // Use midpoint of trimmed centerline as centroid
        size_t mid = pts.size() / 2;
        WorldPosition centroid = pts[mid];

        double dx = centroid.x - pos.x;
        double dy = centroid.y - pos.y;
        double d2 = dx*dx + dy*dy;
        if (d2 < bestDist2) {
            bestDist2 = d2;
            best = lane;
        }
    }
    return best;
}

std::vector<ParkingSpot*> ParkingSystem::findFreeSpots(WorldPosition pos, double radius) const
{
    double r2 = radius * radius;
    std::vector<ParkingSpot*> result;

    for (auto& spot : spots_) {
        if (spot.isOccupied()) continue;
        double dx = spot.position.x - pos.x;
        double dy = spot.position.y - pos.y;
        double d2 = dx*dx + dy*dy;
        if (d2 <= r2) {
            result.push_back(const_cast<ParkingSpot*>(&spot));
        }
    }

    // Sort by distance
    std::sort(result.begin(), result.end(), [&](ParkingSpot* a, ParkingSpot* b) {
        double da = (a->position.x - pos.x)*(a->position.x - pos.x) +
                    (a->position.y - pos.y)*(a->position.y - pos.y);
        double db = (b->position.x - pos.x)*(b->position.x - pos.x) +
                    (b->position.y - pos.y)*(b->position.y - pos.y);
        return da < db;
    });

    return result;
}

bool ParkingSystem::claimSpot(int64_t spotId, int64_t agentId)
{
    auto it = spotById_.find(spotId);
    if (it == spotById_.end()) return false;
    ParkingSpot* spot = it->second;
    if (spot->isOccupied()) return false;
    spot->occupantId = agentId;
    return true;
}

void ParkingSystem::releaseSpot(int64_t spotId)
{
    auto it = spotById_.find(spotId);
    if (it != spotById_.end()) {
        it->second->occupantId = -1;
    }
}

ParkingSpot* ParkingSystem::getSpot(int64_t spotId)
{
    auto it = spotById_.find(spotId);
    return (it != spotById_.end()) ? it->second : nullptr;
}

int ParkingSystem::getOccupancy() const
{
    int count = 0;
    for (const auto& spot : spots_) {
        if (spot.isOccupied()) ++count;
    }
    return count;
}
