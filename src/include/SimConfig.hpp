#pragma once
#include "SimulationTypes.hpp"
#include <string>
#include <vector>

/**
 * @struct SimConfig
 * @brief Top-level configuration loaded from simulation.json at startup.
 */
struct SimConfig {
    std::string mapFile          = "src/data/map.osm";
    std::string defaultProject   = "";   ///< If set, this project is auto-loaded at startup.
    int         httpPort         = 9001;
    int         wsPort           = 9002;
    double      initialTimeScale = 0.0;

    GlobalMapParameters mapParams = {
        /* defaultLaneWidth              */ 3.5,
        /* defaultSpeedLimit             */ 50.0 / 3.6,
        /* defaultIntersectionCurbRadius */ 8.0,
        /* defaultTurnCurvatureWeight    */ 0.5
    };

    SimulationTuning tuning;

    std::vector<VehicleTypeConfig>  vehicleTypes;
    std::vector<RouteDefinition>    routes;
    std::vector<SpawnRegionConfig>  spawnRegions;
};
