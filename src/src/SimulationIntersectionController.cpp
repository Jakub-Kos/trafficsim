#include "../include/SimulationIntersectionController.hpp"
#include "../include/SimulationIntersectionPath.hpp"
#include "../include/SimulationIntersection.hpp"
#include <algorithm>

// Define constants
static const double YELLOW_DURATION = 3.0;

// Linear search over a flat vector beats unordered_map for the small N (≤25)
// typical of intersection conflict IDs.

bool SimulationIntersectionController::isConflictLocked(int conflictId, int64_t requestingAgentId) const {
    for (const auto& [cId, aId] : locks_) {
        if (cId == conflictId) {
            return aId != requestingAgentId; // locked by someone else
        }
    }
    return false; // not locked
}

void SimulationIntersectionController::lockConflict(int conflictId, int64_t agentId) {
    for (auto& [cId, aId] : locks_) {
        if (cId == conflictId) { aId = agentId; return; } // update existing
    }
    locks_.push_back({conflictId, agentId}); // new entry
}

void SimulationIntersectionController::unlockConflict(int conflictId, int64_t agentId) {
    for (auto it = locks_.begin(); it != locks_.end(); ++it) {
        if (it->first == conflictId && it->second == agentId) {
            locks_.erase(it);
            return;
        }
    }
}

void SimulationIntersectionController::releaseAllForAgent(int64_t agentId) {
    std::erase_if(locks_, [agentId](const auto& p) { return p.second == agentId; });
}

void SimulationIntersectionController::setPhases(const std::vector<TrafficLightPhase>& phases) {
    phases_ = phases;
    currentPhaseIdx_ = 0;
    phaseTimer_ = 0.0;
    isYellow_ = false;
}

void SimulationIntersectionController::update(double dt) {
    if (phases_.empty()) return;

    phaseTimer_ += dt;

    if (isYellow_) {
        if (phaseTimer_ >= YELLOW_DURATION) {
            isYellow_ = false;
            phaseTimer_ = 0.0;
            currentPhaseIdx_ = (currentPhaseIdx_ + 1) % phases_.size();
        }
    } else {
        if (phaseTimer_ >= phases_[currentPhaseIdx_].duration) {
            isYellow_ = true;
            phaseTimer_ = 0.0;
        }
    }
}

LightState SimulationIntersectionController::getLightState(int pathIndex) const {
    if (phases_.empty()) return LightState::Green;

    const auto& greenPaths = phases_[currentPhaseIdx_].greenPathIndices;
    bool isGreenPath = std::find(greenPaths.begin(), greenPaths.end(), pathIndex) != greenPaths.end();

    if (isYellow_ && isGreenPath) return LightState::Yellow;
    if (isGreenPath) return LightState::Green;
    return LightState::Red;
}