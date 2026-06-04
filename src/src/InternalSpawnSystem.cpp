#include "../include/InternalSpawnSystem.hpp"
#include "../include/SimulationWorld.hpp"
#include "../include/SimulationLane.hpp"
#include "../include/SimulationAgent.hpp"
#include <iostream>

// Constructor — seed the per-instance RNG so parallel headless sims are deterministic
InternalSpawnSystem::InternalSpawnSystem(SimulationWorld* world, uint64_t seed)
    : world_(world), rng_(seed), nextAgentId_(0)
{}

void InternalSpawnSystem::addSpawnPoint(SimulationLane* lane, double spawnRate) {
    legacyPoints_.push_back({lane, spawnRate, 0.0});
}

void InternalSpawnSystem::addSpawnRegion(const SpawnRegionConfig& config) {
    std::vector<SimulationLane*> validLanes = world_->getStartLanesInBounds(config.bounds);

    if (!validLanes.empty()) {
        const size_t numLanes = validLanes.size();

        // Create an independent timer for each lane
        std::vector<double> initialTimers(numLanes, 0.0);

        // Optional: Stagger the initial timers randomly so cars don't spawn exactly parallel
        std::uniform_real_distribution<double> offsetDist(0.0, 1.0 / config.spawnRatePerSec);
        for(size_t i = 0; i < numLanes; ++i) {
            initialTimers[i] = offsetDist(rng_);
        }

        // Move into the struct
        regions_.push_back({config, std::move(initialTimers), std::move(validLanes)});

        std::cout << "Spawn region '" << config.regionId << "' activated with "
                  << numLanes << " valid start lanes." << std::endl;
    } else {
        std::cout << "WARNING: Spawn region '" << config.regionId
                  << "' created, but no valid start lanes were found in its bounds." << std::endl;
    }
}

int64_t InternalSpawnSystem::getNextAgentId() {
    return nextAgentId_++;
}

void InternalSpawnSystem::update(double dt) {
    if (!enabled_) return;

    // --- 1. Process legacy, single-lane spawn points ---
    for (auto& sp : legacyPoints_) {
        sp.timer += dt;
        double interval = 1.0 / sp.spawnRate;

        while (sp.timer >= interval) {
            sp.timer -= interval;

            // 1) Spacing check delegated to SimulationWorld
            // We use a default MIN_GAP, which should be a constant from SimulationAgent.hpp
            if (!world_->canSpawn(sp.lane, 5.0 /*MIN_GAP*/)) {
                break; // Lane is blocked, stop trying for this frame
            }

            // 2) OK to spawn - let SimulationWorld create the agent
            world_->spawnNewAgent(sp.lane, getNextAgentId());
        }
    }

    // --- 2. Process new, region-based spawn points ---
    for (auto& region : regions_) {
        if (region.validLanes.empty()) {
            continue; // Nothing to do
        }

        // The rate is now treated as "per-lane"
        double interval = 1.0 / region.config.spawnRatePerSec;

        // Iterate through EVERY lane in this region
        for (size_t i = 0; i < region.validLanes.size(); ++i) {
            region.laneTimers[i] += dt;

            while (region.laneTimers[i] >= interval) {
                region.laneTimers[i] -= interval;

                SimulationLane* lane = region.validLanes[i];

                // Check if this specific lane is clear
                if (world_->canSpawn(lane, 5.0 /*MIN_GAP*/)) {
                    // Spawn immediately on this lane
                    world_->spawnNewAgent(lane, getNextAgentId());
                    region.spawnCount++;
                } else {
                    // Lane is blocked by traffic. We break the while loop
                    // so we don't build up an infinite backlog of instant spawns.
                    break;
                }
            }
        }
    }
}