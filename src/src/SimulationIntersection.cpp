/*
 * Note: Gemini (Gemini 3.1 Pro) was used to assist with commenting this file —
 * clarifying what the code physically does and documenting non-obvious decisions.
 * The implementation logic was written by the author.
 */
#include "../include/SimulationIntersection.hpp"
#include "../include/SimulationIntersectionPath.hpp"
#include "../include/SimulationLane.hpp"
#include "../include/SimulationRoad.hpp"
#include "../include/MathHelpers.hpp"
#include <vector>
#include <algorithm>
#include <unordered_map>

const std::vector<ConflictPoint> SimulationIntersection::kEmptyConflicts_{};

// --- SimulationIntersection ---

SimulationIntersection::SimulationIntersection(int64_t id, WorldPosition pos, double curbRadius)
    : id_(id),
      position_(pos),
      curbRadius_(curbRadius)
{
    controller_.setParent(this);
}

// --- SimulationIntersectionPath ---

SimulationIntersectionPath::SimulationIntersectionPath(SimulationLane* fromLane, SimulationLane* toLane, TurnDirection dir)
    : fromLane_(fromLane), toLane_(toLane), direction_(dir)
{}

/**
 * @brief Samples a cubic Bézier curve: P0, P1, P2, P3.
 * Provides G2-continuous turns when P1 = P0 + vIn*h and P2 = P3 - vOut*h.
 */
static std::vector<WorldPosition> sampleCubicBezier(
    const WorldPosition& P0,
    const WorldPosition& P1,
    const WorldPosition& P2,
    const WorldPosition& P3,
    const int steps)
{
    std::vector<WorldPosition> pts;
    pts.reserve(steps + 1);
    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const double u = 1.0 - t;
        // B(t) = u³·P0 + 3u²t·P1 + 3ut²·P2 + t³·P3
        WorldPosition B = (u*u*u)        * P0
                        + (3.0*u*u*t)   * P1
                        + (3.0*u*t*t)   * P2
                        + (t*t*t)       * P3;
        pts.push_back(B);
    }
    return pts;
}

void SimulationIntersectionPath::setupPathCurve(const WorldPosition& p0,
                                                const WorldPosition& v_in,
                                                const WorldPosition& p3,
                                                const WorldPosition& v_out,
                                                const int steps)
{
    // Handle lengths at 40% of chord — gives natural-looking curves at any scale.
    const double h = length(p3 - p0) * 0.4;
    const WorldPosition P1 = p0 + v_in  * h;        // tangent handle leaving p0
    const WorldPosition P2 = p3 - v_out * h;        // tangent handle approaching p3

    pathPoints_ = sampleCubicBezier(p0, P1, P2, p3, steps);

    // Cache total arc length
    pathLength_ = 0.0;
    for (size_t i = 1; i < pathPoints_.size(); ++i)
        pathLength_ += length(pathPoints_[i] - pathPoints_[i - 1]);
}
// --- HELPER: Get total length of a path ---
static double getPathLength(const std::vector<WorldPosition>& pts) {
    double d = 0.0;
    if (pts.empty()) return 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        d += length(pts[i+1] - pts[i]);
    }
    return d;
}

void SimulationIntersection::computeConflictPoints() {
    conflicts_.clear();
    int cId = 0;

    // Compare every path against every other path
    for (size_t i = 0; i < paths_.size(); ++i) {
        for (size_t j = i + 1; j < paths_.size(); ++j) {
            const auto& pathA = paths_[i];
            const auto& pathB = paths_[j];

            // 1. ARTIFICIAL CONFLICTS (Entry/Exit)

            // DIVERGING: Share FromLane -> Conflict at start (dist=0)
            if (pathA.getFromLane() == pathB.getFromLane()) {
                if (!pathA.getPathPoints().empty()) {
                    conflicts_.push_back({
                        cId++,
                        pathA.getPathPoints().front(),
                        &pathA, &pathB,
                        0.0, 0.0
                    });
                }
            }

            // MERGING: Share ToLane -> Conflict at end (dist=totalLength)
            if (pathA.getToLane() == pathB.getToLane()) {
                double lenA = getPathLength(pathA.getPathPoints());
                double lenB = getPathLength(pathB.getPathPoints());

                if (!pathA.getPathPoints().empty()) {
                    conflicts_.push_back({
                        cId++,
                        pathA.getPathPoints().back(), // Visual position
                        &pathA, &pathB,
                        lenA, lenB
                    });
                }
            }

            // 2.A DETERMINE PRIORITY RULE
            // 0=None, 1=A is Priority, 2=B is Priority
            int rule = 0;

            // Road-type hierarchy: lower enum value = higher-priority road.
            // A path from a Primary road always beats one from a Residential road,
            // regardless of turn direction. Fall back to turn-direction only when
            // both paths come from the same road-type tier.
            {
                int rtA = static_cast<int>(pathA.getFromLane()->getParentRoad()->getRoadType());
                int rtB = static_cast<int>(pathB.getFromLane()->getParentRoad()->getRoadType());

                if (rtA < rtB) {
                    rule = 1; // A is on the higher-priority road
                } else if (rtB < rtA) {
                    rule = 2; // B is on the higher-priority road
                } else {
                    // Same road-type tier: fall back to turn-direction rule (RHT).
                    // Straight/Right > Left.
                    bool aIsLeft     = (pathA.getDirection() == TurnDirection::Left);
                    bool bIsLeft     = (pathB.getDirection() == TurnDirection::Left);
                    bool aIsStraight = (pathA.getDirection() == TurnDirection::Straight);
                    bool bIsStraight = (pathB.getDirection() == TurnDirection::Straight);

                    if      (aIsLeft && bIsStraight) rule = 2; // B wins
                    else if (bIsLeft && aIsStraight) rule = 1; // A wins
                }
            }

            // 2. GEOMETRIC CONFLICTS (Crossing)
            const auto& ptsA = pathA.getPathPoints();
            const auto& ptsB = pathB.getPathPoints();

            for (size_t ia = 0; ia + 1 < ptsA.size(); ++ia) {
                for (size_t ib = 0; ib + 1 < ptsB.size(); ++ib) {
                    WorldPosition intersect;
                    if (getSegmentIntersection(ptsA[ia], ptsA[ia+1], ptsB[ib], ptsB[ib+1], intersect)) {

                        // Calculate distance along path to this point
                        double distA = 0;
                        for(size_t k=0; k<ia; ++k) distA += length(ptsA[k+1]-ptsA[k]);
                        distA += length(intersect - ptsA[ia]);

                        double distB = 0;
                        for(size_t k=0; k<ib; ++k) distB += length(ptsB[k+1]-ptsB[k]);
                        distB += length(intersect - ptsB[ib]);

                        conflicts_.push_back({
                            cId++, intersect,
                            &pathA, &pathB,
                            distA, distB,
                            rule
                        });

                        // Optimization: if these segments cross, we record the conflict and
                        // stop checking the rest of pathB for this specific pair of paths.
                        goto next_pair;
                    }
                }
            }
            next_pair:;
        }
    }

    // Build per-path conflict cache directly on path objects (eliminates unordered_map lookup per tick)
    // First clear any stale data
    for (auto& p : paths_) {
        p.setCachedConflicts({});
    }

    // Temporary per-pointer accumulator (only used during build, not at tick time)
    std::unordered_map<SimulationIntersectionPath*, std::vector<ConflictPoint>> tmp;
    for (const auto& c : conflicts_) {
        auto* pA = static_cast<SimulationIntersectionPath*>(const_cast<void*>(c.pathA));
        auto* pB = static_cast<SimulationIntersectionPath*>(const_cast<void*>(c.pathB));
        tmp[pA].push_back(c);
        ConflictPoint swapped = c;
        swapped.pathA   = c.pathB;
        swapped.pathB   = c.pathA;
        swapped.distOnA = c.distOnB;
        swapped.distOnB = c.distOnA;
        tmp[pB].push_back(swapped);
    }
    for (auto& [path, vec] : tmp) {
        std::sort(vec.begin(), vec.end(), [](const ConflictPoint& a, const ConflictPoint& b){
            return a.distOnA < b.distOnA;
        });
        path->setCachedConflicts(std::move(vec));
    }

    // Cache path indices directly on path objects
    for (size_t i = 0; i < paths_.size(); ++i)
        paths_[i].setPathIndex(static_cast<int>(i));
}

const std::vector<ConflictPoint>& SimulationIntersection::getConflictsForPath(const SimulationIntersectionPath* path) const {
    if (!path) return kEmptyConflicts_;
    return path->getCachedConflicts();
}

void SimulationIntersection::update(double dt) {
    controller_.update(dt);
}

int SimulationIntersection::getPathIndex(const SimulationIntersectionPath* path) const {
    if (!path) return -1;
    return path->getPathIndex();
}