#include "../include/SimulationEngine.hpp"
#include "../include/SimulationWorld.hpp"
#include "../include/MetricsCollector.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>

#include "OSMMapLoader.hpp"

// SimulationEngine serves only as a facade for the SimulationWorld

// --- Constructor & Destructor ---
SimulationEngine::SimulationEngine()
    : pimpl_(std::make_unique<SimulationWorld>()) {
}
SimulationEngine::~SimulationEngine() noexcept = default;

// --- Move Semantics ---
SimulationEngine::SimulationEngine(SimulationEngine&&) noexcept = default;
SimulationEngine& SimulationEngine::operator=(SimulationEngine&&) noexcept = default;

// --- Phase 1: Setup & Initialization ---
void SimulationEngine::loadMapFromOSM(const std::string& osmData) {
    // 1. Use the dedicated loader to convert XML -> JSON string
    std::string mapJson = OSMMapLoader::parseOSMtoJSON(osmData);

    // 2. Initialize the world with the generated JSON
    pimpl_->initialize(mapJson);
}

void SimulationEngine::setGlobalMapParameters(const GlobalMapParameters& params) {
    pimpl_->setGlobalMapParameters(params);
}

void SimulationEngine::setSimulationTuning(const SimulationTuning& tuning) {
    pimpl_->setSimulationTuning(tuning);
}

[[nodiscard]] GlobalMapParameters SimulationEngine::getGlobalMapParameters() const {
    return pimpl_->getGlobalMapParameters();
}

void SimulationEngine::initialize() {
    // legacy way of doing things, now it's not hardcoded
    const std::string mapFilePath = "src/data/3x3_grid.json";

    std::ifstream ifs(mapFilePath);
    if (!ifs) {
        throw std::runtime_error("Failed to open map file: " + mapFilePath);
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    std::string jsonData = buffer.str();

    pimpl_->initialize(jsonData);
}

// --- Phase 2: Post-Init Map Tinkering ---
void SimulationEngine::updateNodePosition(int64_t nodeId, const WorldPosition& newPosition) {
    pimpl_->updateNodePosition(nodeId, newPosition);
}

void SimulationEngine::updateWayProperties(int64_t wayId, const WayConfig& config) {
    pimpl_->updateWayProperties(wayId, config);
}

void SimulationEngine::updateIntersectionTurn(int64_t intersectionId,
                                            int64_t fromWayId,
                                            int64_t toWayId,
                                            const TurnPathConfig& config) {
    pimpl_->updateIntersectionTurn(intersectionId, fromWayId, toWayId, config);
}

// --- Phase 3: Simulation Setup ---
void SimulationEngine::addVehicleType(const VehicleTypeConfig& config) {
    pimpl_->addVehicleType(config);
}

void SimulationEngine::defineRoute(const RouteDefinition& route) {
    pimpl_->defineRoute(route);
}

void SimulationEngine::addSpawnRegion(const SpawnRegionConfig& config) {
    pimpl_->addSpawnRegion(config);
}

void SimulationEngine::scaleSpawnRates(double factor) {
    pimpl_->scaleSpawnRates(factor);
}

void SimulationEngine::addPublicTransportLine(const PublicTransportLineConfig& config) {
    pimpl_->addPublicTransportLine(config);
}

// --- Phase 4: Simulation Control & Execution ---
void SimulationEngine::step(double deltaTime) {
    pimpl_->step(deltaTime);
}

// --- Phase 5: Data-Out (Querying State) ---

[[nodiscard]] std::string SimulationEngine::getMapDataJSON() const {
    return pimpl_->getMapDataJSON();
}

[[nodiscard]] std::string SimulationEngine::getIntersectionPositionsJSON() const {
    return pimpl_->getIntersectionPositionsJSON();
}

[[nodiscard]] SimulationMetrics SimulationEngine::getMetrics() const {
    return pimpl_->getMetrics();
}

[[nodiscard]] std::vector<VehicleState> SimulationEngine::getVehicleStatesInBounds(
    const BoundingBox& bounds) const {
    return pimpl_->getVehicleStatesInBounds(bounds);
}

[[nodiscard]] std::optional<DetailedVehicleState> SimulationEngine::getDetailedVehicleState(
    int64_t vehicleId) const {
    return pimpl_->getDetailedVehicleState(vehicleId);
}

[[nodiscard]] std::vector<HeatmapCell> SimulationEngine::getVehicleHeatmap(
    const BoundingBox& bounds,
    int gridResolutionX,
    int gridResolutionY) const {
    return pimpl_->getVehicleHeatmap(bounds, gridResolutionX, gridResolutionY);
}

[[nodiscard]] std::vector<LaneTrafficStat> SimulationEngine::getLaneTrafficStats() const {
    return pimpl_->getLaneTrafficStats();
}

// --- Phase 6: Runtime Interaction ---
void SimulationEngine::setVehicleRoute(int64_t vehicleId, const std::string& routeId) {
    pimpl_->setVehicleRoute(vehicleId, routeId);
}

void SimulationEngine::setRoadClosed(int64_t wayId, bool isClosed) {
    pimpl_->setRoadClosed(wayId, isClosed);
}

void SimulationEngine::setLaneClosed(int64_t wayId, int laneIdx, bool isClosed) {
    pimpl_->setLaneClosed(wayId, laneIdx, isClosed);
}

void SimulationEngine::setSeed(uint64_t seed) {
    pimpl_->setSeed(seed);
}

[[nodiscard]] std::vector<IntersectionStateData> SimulationEngine::getIntersectionStates() const {
    return pimpl_->getIntersectionStates();
}

[[nodiscard]] std::string SimulationEngine::getParkingSpotsJSON() const {
    return pimpl_->getParkingSpotsJSON();
}

void SimulationEngine::setSpawningEnabled(bool e) { pimpl_->setSpawningEnabled(e); }
bool SimulationEngine::isSpawningEnabled() const  { return pimpl_->isSpawningEnabled(); }
std::vector<InternalSpawnSystem::RegionInfo> SimulationEngine::getSpawnRegionInfo() const {
    return pimpl_->getSpawnRegionInfo();
}

void SimulationEngine::setMetricsCollector(MetricsCollector* collector,
                                            const std::string& runId) {
    pimpl_->setMetricsCollector(collector, runId);
}

int64_t SimulationEngine::getTotalSpawned() const {
    return pimpl_->getMetrics().totalSpawned;
}

double SimulationEngine::getTotalSpawnedFreeflowSum() const {
    return pimpl_->getTotalSpawnedFreeflowSum();
}

double SimulationEngine::getTotalExcessSeconds() const {
    return pimpl_->getTotalExcessSeconds();
}

double SimulationEngine::getTotalCompletedTravelSum() const {
    return pimpl_->getTotalCompletedTravelSum();
}

int SimulationEngine::getReachableDestinationCount() const {
    return pimpl_->getReachableDestinationCount();
}

void SimulationEngine::setTripLogRecording(TripLog* log) { pimpl_->setTripLogRecording(log); }
void SimulationEngine::setTripLogReplay(const TripLog* log) { pimpl_->setTripLogReplay(log); }
int32_t SimulationEngine::getExcludedCount() const { return pimpl_->getExcludedCount(); }