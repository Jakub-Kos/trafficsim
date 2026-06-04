#pragma once
#include "SimulationTypes.hpp"
#include <vector>
#include <deque>
#include <unordered_map>
#include <cstdint>

class SimulationLane;
class SimulationRoad;

/**
 * @struct ParkingSpot
 * @brief A single parking space alongside a lane.
 */
struct ParkingSpot {
    int64_t id;
    WorldPosition position;
    double heading;                  ///< Radians; direction a parked car faces.
    ParkingOrientation orientation;
    SimulationLane* adjacentLane;    ///< Lane this spot is next to.
    double laneProgress;             ///< Metres from lane start (trimmed centreline).
    int64_t occupantId = -1;
    [[nodiscard]] bool isOccupied() const { return occupantId != -1; }
};

/**
 * @class ParkingSystem
 * @brief Generates and manages the full set of parking spots for a loaded map.
 *
 * Spots are generated once from OSM road data and stored in a deque for
 * stable addresses. Agents interact with the system via claimSpot /
 * releaseSpot; all other accessors are read-only.
 */
class ParkingSystem {
public:
    // ---- Spot geometry constants (metres unless noted) ----------------------
    static constexpr double PARALLEL_SPOT_LENGTH = 6.0;
    static constexpr double PARALLEL_SPOT_DEPTH  = 2.2;
    static constexpr double DIAGONAL_SPOT_WIDTH  = 2.5;
    static constexpr double DIAGONAL_SPOT_DEPTH  = 5.0;
    static constexpr double DIAGONAL_ANGLE_RAD   = 0.7854; ///< 45 degrees.
    static constexpr double PERP_SPOT_WIDTH      = 2.5;
    static constexpr double PERP_SPOT_DEPTH      = 5.0;
    static constexpr double SPOT_GAP             = 0.5;

    /**
     * @brief Generate spots from all roads that have parking configured.
     * @param laneWidth The simulation's default lane width (metres).
     */
    void generateSpots(const std::vector<SimulationRoad*>& roads, double laneWidth);

    void setBuildingCentroids(std::vector<WorldPosition> centroids);
    [[nodiscard]] const std::vector<WorldPosition>& getBuildingCentroids() const { return buildingCentroids_; }

    /** @brief Return the nearest parking-eligible lane to pos, or null if none within maxRadius. */
    [[nodiscard]] SimulationLane* findNearestParkingLane(WorldPosition pos, double maxRadius = 500.0) const;

    /** @brief Return all free spots within radius of pos, sorted by distance. */
    [[nodiscard]] std::vector<ParkingSpot*> findFreeSpots(WorldPosition pos, double radius = 100.0) const;

    /** @brief Atomically mark a spot as occupied by agentId. Returns false if already taken. */
    bool claimSpot(int64_t spotId, int64_t agentId);
    void releaseSpot(int64_t spotId);

    [[nodiscard]] ParkingSpot*                    getSpot(int64_t spotId);
    [[nodiscard]] const std::deque<ParkingSpot>&  getSpots()       const { return spots_; }
    [[nodiscard]] int                             getTotalSpots()  const { return static_cast<int>(spots_.size()); }
    [[nodiscard]] int                             getOccupancy()   const;
    [[nodiscard]] const std::vector<SimulationLane*>& getParkingLanes() const { return parkingLanes_; }

private:
    std::deque<ParkingSpot>              spots_;            ///< Deque used for stable spot addresses.
    std::unordered_map<int64_t, ParkingSpot*> spotById_;
    int64_t                              nextSpotId_ = 0;
    std::vector<WorldPosition>           buildingCentroids_;
    std::vector<SimulationLane*>         parkingLanes_;

    void generateSpotsForLane(SimulationLane* lane, ParkingOrientation orientation,
                               bool rightSide, double laneWidth);
};
