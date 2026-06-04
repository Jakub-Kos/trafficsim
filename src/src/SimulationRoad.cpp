#include "../include/SimulationRoad.hpp"
#include "../include/SimulationNode.hpp"
#include "../include/SimulationLane.hpp"
#include <vector>

SimulationRoad::SimulationRoad(const int64_t id, const std::vector<SimulationNode*>& nodes)
    : id_(id), nodes_(nodes) {
}

std::vector<WorldPosition> SimulationRoad::getCenterline() const {
    std::vector<WorldPosition> path;
    path.reserve(nodes_.size());
    for (const auto* node : nodes_) {
        path.push_back(node->getPosition());
    }
    return path;
}

void SimulationRoad::addLane(std::unique_ptr<SimulationLane> lane) {
    lanePtrsCache_.push_back(lane.get());
    lanes_.push_back(std::move(lane));
}

const std::vector<SimulationLane*>& SimulationRoad::getLanes() const {
    return lanePtrsCache_;
}
