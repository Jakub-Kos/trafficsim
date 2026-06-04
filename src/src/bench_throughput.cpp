// Headless simulation throughput benchmark.
// Builds as a separate executable; run from the project root:
//   ./build/bin/bench_throughput [path/to/map.osm]
// Default OSM path: src/data/map.osm

#include "SimulationEngine.hpp"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

static std::string readFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) { std::fprintf(stderr, "Cannot open: %s\n", path.c_str()); std::exit(1); }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

struct Result { double multiplier; double avgAgents; double simPerWall; };

static Result runLevel(const std::string& osmData, double multiplier) {
    constexpr double STEP        = 0.066;   // sim-seconds per tick
    constexpr double WARMUP_S    = 300.0;   // sim-seconds discarded before measurement
    constexpr double MEASURE_S   = 1200.0;  // sim-seconds measured

    SimulationEngine eng;
    // Disable parking so only actively-driving agents are counted (pure throughput benchmark)
    SimulationTuning tuning;
    tuning.parkingDestinationProbability = 0.0;
    eng.setSimulationTuning(tuning);
    eng.loadMapFromOSM(osmData);
    // Must match simulation.json vehicleTypes — spawn system silently no-ops without this
    eng.addVehicleType({"default_car", 4.5, 15.0, 2.5, 4.0});
    if (multiplier != 1.0) eng.scaleSpawnRates(multiplier);

    // Warm-up (not timed)
    for (double t = 0.0; t < WARMUP_S; t += STEP) eng.step(STEP);

    double agentSum = 0.0;
    int agentSamples = 0;
    int stepNum = 0;

    // Measured run
    auto wallStart = std::chrono::steady_clock::now();
    for (double t = 0.0; t < MEASURE_S; t += STEP) {
        eng.step(STEP);
        if (++stepNum % 100 == 0) {              // sample every ~6.6 sim-s
            agentSum += eng.getMetrics().totalVehicles;
            ++agentSamples;
        }
    }
    double wallSec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - wallStart).count();

    return {
        multiplier,
        agentSamples > 0 ? agentSum / agentSamples : 0.0,
        MEASURE_S / wallSec
    };
}

int main(int argc, char** argv) {
    const std::string osmPath = argc > 1 ? argv[1] : "src/data/map.osm";
    std::string osmData = readFile(osmPath);

    // Multipliers chosen to sweep across ~100-2000 concurrent agents
    // Actual agent counts are measured and printed
    const std::vector<double> multipliers = {0.25, 1.0, 2.5, 5.0};

    std::printf("\n%-10s  %-14s  %-22s  %-14s\n",
                "Multiplier", "Avg agents", "Sim-s / wall-s", "Real-time factor");
    std::printf("%-10s  %-14s  %-22s  %-14s\n",
                "----------", "--------------", "----------------------", "--------------");

    for (double m : multipliers) {
        std::printf("Running %.2fx ... ", m);
        std::fflush(stdout);
        auto r = runLevel(osmData, m);
        std::printf("\r%-10.2f  %-14.0f  %-22.1f  %-14.1f\n",
                    r.multiplier, r.avgAgents, r.simPerWall, r.simPerWall);
    }

    std::printf("\nNote: real-time factor == sim-s/wall-s (1.0 = exact real time, higher = faster).\n");
    std::printf("Pick the four rows closest to 100 / 500 / 1000 / 2000\n");
    return 0;
}