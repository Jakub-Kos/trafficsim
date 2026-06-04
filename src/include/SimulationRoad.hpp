#pragma once

#include "SimulationTypes.hpp"
#include <vector>
#include <memory>
#include <string>

// Forward declarations
class SimulationNode;
class SimulationLane;

/**
 * @class SimulationRoad
 * @brief An OSM way: owns its lanes and defines the centreline geometry shared by all of them.
 *
 * Lanes are created externally and transferred via addLane(). The road stores them as
 * unique_ptrs and maintains a raw-pointer cache for O(1) getLanes() access.
 */
class SimulationRoad {
public:
    /**
     * @brief Construct a road from an OSM ID and a sequence of node pointers.
     *
     * Nodes are non-owning references into SimulationWorld’s node store.
     * No lanes are created here; call addLane() during the graph-build phase.
     *
     * @param id    Unique string ID for this road.
     * @param nodes Vector of pointers to SimulationNode representing the road geometry.
     */
    SimulationRoad(int64_t id, const std::vector<SimulationNode*>& nodes);

    // --- Getters ---
    [[nodiscard]] int64_t                             getId()           const { return id_; }
    [[nodiscard]] const std::vector<SimulationNode*>& getNodes()        const { return nodes_; }
    [[nodiscard]] const std::vector<SimulationLane*>& getLanes()        const;
    [[nodiscard]] RoadType                            getRoadType()     const { return roadType_; }
    [[nodiscard]] ParkingOrientation                  getParkingLeft()  const { return parkingLeft_; }
    [[nodiscard]] ParkingOrientation                  getParkingRight() const { return parkingRight_; }
    [[nodiscard]] bool                                isClosed()        const { return isClosed_; }

    /**
     * @brief Effective speed limit in m/s.
     *
     * Returns the explicit OSM maxspeed when set; otherwise falls back to a
     * type-based default (motorway 130, trunk 100, primary 80, secondary 70,
     * tertiary 50, residential/other 30 km/h).
     */
    [[nodiscard]] double getSpeedLimitMps() const {
        if (speedLimitMps_ > 0.0) return speedLimitMps_;
        switch (roadType_) {
            case RoadType::Motorway:    return 130.0 / 3.6;
            case RoadType::Trunk:       return 100.0 / 3.6;
            case RoadType::Primary:     return  80.0 / 3.6;
            case RoadType::Secondary:   return  70.0 / 3.6;
            case RoadType::Tertiary:    return  50.0 / 3.6;
            case RoadType::Residential: return  30.0 / 3.6;
            default:                    return  30.0 / 3.6;
        }
    }

    /** @brief Node positions in order — used by SimulationLane::computeCenterline(). */
    [[nodiscard]] std::vector<WorldPosition> getCenterline() const;

    // --- Setters ---
    void setRoadType(RoadType rt)              { roadType_ = rt; }
    void setSpeedLimitMps(double mps)          { speedLimitMps_ = mps; }
    void setParkingLeft(ParkingOrientation po)  { parkingLeft_  = po; }
    void setParkingRight(ParkingOrientation po) { parkingRight_ = po; }
    void setIsClosed(bool c)                   { isClosed_ = c; }

    /** @brief Transfer ownership of a lane to this road and append it to the cache. */
    void addLane(std::unique_ptr<SimulationLane> lane);

private:
    int64_t  id_;                               ///< Unique int64 identifier for this road (e.g., "12345").
    RoadType roadType_     = RoadType::Unknown; ///< Road classification derived from OSM highway tag.
    double   speedLimitMps_ = 0.0;              ///<  Explicit OSM maxspeed in m/s; 0 = use type-based default.

    std::vector<SimulationNode*>              nodes_;          ///<  Non-owning; geometry nodes in OSM order.
    std::vector<std::unique_ptr<SimulationLane>> lanes_;       ///<  Owned lanes.
    mutable std::vector<SimulationLane*>      lanePtrsCache_;  ///<  Raw-pointer view of lanes_; rebuilt by addLane().

    ParkingOrientation parkingLeft_  = ParkingOrientation::None;  ///<  Parking configuration on the left kerb.
    ParkingOrientation parkingRight_ = ParkingOrientation::None;  ///<  Parking configuration on the right kerb.
    bool isClosed_ = false;                                        ///<  When true, agents reroute around this road.
};