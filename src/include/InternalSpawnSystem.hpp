#pragma once

#include "SimulationTypes.hpp"
#include <vector>
#include <string>
#include <random>

// Forward declarations
class SimulationWorld;
class SimulationLane;
class SimulationNode;

/**
 * @class InternalSpawnSystem
 * @brief Manages spawn points and timing for creating new vehicles in the City.
 *
 * Manages all logic for creating new agents, both from specific
 * single-lane spawn points and from general spawn regions.
 */
class InternalSpawnSystem {
public:
    /**
     * @param world A non-owning pointer to the parent SimulationWorld,
     * used to call back to create agents.
     * @param seed RNG seed for deterministic spawn-timer staggering.
     */
    InternalSpawnSystem(SimulationWorld* world, uint64_t seed);

    /** @brief Re-seed the internal RNG (must be called before addSpawnRegion). */
    void setSeed(const uint64_t seed) { rng_.seed(seed); }

    /**
     * @brief Adds a legacy-style, single-lane spawn point.
     *
     * @param lane The specific SimulationLane to spawn on.
     * @param spawnRate The rate in agents per second.
     */
    void addSpawnPoint(SimulationLane* lane, double spawnRate);

    /**
     * @brief Adds a new, generalized spawn region.
     * This is the new API method for supporting the
     * SpawnRegionConfig struct from the SimulationEngine.
     *
     * @param config The region's configuration.
     */
    void addSpawnRegion(const SpawnRegionConfig& config);

    /**
     * @brief Updates all spawn points and regions.
     * This will run the logic from legacy SpawnSystem::update
     * for the legacy points, as well as new logic for the spawn regions.
     *
     * @param dt Time delta in seconds.
     */
    void update(double dt);

    /**
     * @brief Generates the next available agent ID.
     * Migrated from SpawnSystem::nextVehId_.
     *
     * @return A new, unique 64-bit agent ID.
     */
    int64_t getNextAgentId();

private:
    /**
     * @struct InternalSpawnPoint
     * @brief Decoupled replacement for the internal SpawnPoint struct.
     *
     * This holds the state for a single-lane spawner.
     */
    struct InternalSpawnPoint {
        SimulationLane* lane;
        double spawnRate;
        double timer;
    };

    /**
     * @struct ActiveSpawnRegion
     * @brief Internal struct for managing the region-based spawners.
     */
    struct ActiveSpawnRegion {
        SpawnRegionConfig config;
        std::vector<double> laneTimers;
        std::vector<SimulationLane*> validLanes;
        int64_t spawnCount = 0;  ///< Total agents spawned from this region.
    };

    SimulationWorld* world_;
    std::mt19937 rng_;
    std::vector<InternalSpawnPoint> legacyPoints_;
    std::vector<ActiveSpawnRegion> regions_;
    int64_t nextAgentId_ = 0;
    bool enabled_ = true;

public:
    void setEnabled(const bool e) { enabled_ = e; }
    [[nodiscard]] bool isEnabled() const { return enabled_; }
    void scaleAllRates(double factor) {
        for (auto& r : regions_) r.config.spawnRatePerSec *= factor;
    }

    struct RegionInfo {
        std::string id;
        int         laneCount;
        double      ratePerLane;
        double      totalRate;
        double      x, y;          ///< World-coordinate center of the region bounds.
        int64_t     spawnCount;    ///< Total agents spawned from this region since sim start.
    };
    [[nodiscard]] std::vector<RegionInfo> getRegionInfo() const {
        std::vector<RegionInfo> out;
        for (const auto& r : regions_) {
            double cx = (r.config.bounds.min.x + r.config.bounds.max.x) * 0.5;
            double cy = (r.config.bounds.min.y + r.config.bounds.max.y) * 0.5;
            out.push_back({r.config.regionId,
                           static_cast<int>(r.validLanes.size()),
                           r.config.spawnRatePerSec,
                           r.config.spawnRatePerSec * r.validLanes.size(),
                           cx, cy,
                           r.spawnCount});
        }
        return out;
    }
    [[nodiscard]] double getTotalSpawnRate() const {
        double t = 0.0;
        for (const auto& sp : legacyPoints_) t += sp.spawnRate;
        for (const auto& r  : regions_)      t += r.config.spawnRatePerSec * r.validLanes.size();
        return t;
    }
};