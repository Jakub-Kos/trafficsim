/*
 * Note: Gemini (Gemini 3.1 Pro) was used to assist with commenting this file —
 * clarifying what the code physically does and documenting non-obvious decisions.
 * The implementation logic was written by the author.
 */
#include "../include/SimulationLane.hpp"
#include "../include/SimulationRoad.hpp"
#include "../include/SimulationAgent.hpp"
#include "../include/SimulationIntersection.hpp"
#include "../include/MathHelpers.hpp"
#include <cmath>
#include <algorithm>

// --- CLASS IMPLEMENTATION ---

SimulationLane::SimulationLane(const int64_t id,
                               SimulationRoad* parent,
                               const bool isForward,
                               const double lateralOffset,
                               const int laneIndex,
                               const double curbRadius)
    : id_(id),
      parentRoad_(parent),
      isForward_(isForward),
      lateralOffset_(lateralOffset),
      laneIndex_(laneIndex),
      curbRadius_(curbRadius),
      startIntersection_(nullptr),
      exitIntersection_(nullptr)
{
}

void SimulationLane::computeCenterline() {
    const auto center = parentRoad_->getCenterline();
    const size_t N = center.size();
    centerline_.clear();
    if (N < 2) return;
    centerline_.resize(N);

    std::vector<WorldPosition> normals(N);

    // compute per-vertex normals
    for (size_t i = 0; i < N; ++i) {
        WorldPosition n{};
        if (i == 0) {
            auto d = center[1] - center[0];
            n = normalize(perp(d));
        } else if (i == N - 1) {
            auto d = center[N - 1] - center[N - 2];
            n = normalize(perp(d));
        } else {
            auto d1 = center[i] - center[i - 1];
            auto d2 = center[i + 1] - center[i];
            auto n1 = normalize(perp(d1));
            auto n2 = normalize(perp(d2));
            n = normalize(n1 + n2);
        }
        normals[i] = n;
    }

    // apply lateral offset
    for (size_t i = 0; i < N; ++i)
        centerline_[i] = center[i] + (normals[i] * lateralOffset_);

    // reverse direction if needed
    if (!isForward_)
        std::reverse(centerline_.begin(), centerline_.end());
}

void SimulationLane::computeTrimmedCenterline() {
    if (centerline_.empty()) {
        trimmedCenterline_.clear();
        return;
    }

    // Default: use the full centerline
    trimmedCenterline_ = centerline_;

    // If we have an intersection, we need the road direction to define the clipping plane
    const auto& roadPts = parentRoad_->getCenterline();
    if (roadPts.size() < 2) return;

    // --- 1. TRIM START (Leaving Intersection) ---
    if (startIntersection_) {
        // For exiting lanes, we generally just use the standard radius
        // (Cars don't stop at traffic lights when LEAVING an intersection)
        const auto& C = startIntersection_->getPosition();
        double R = startIntersection_->getCurbRadius(); // Use intersection radius

        WorldPosition roadDir;
        if (isForward_) roadDir = normalize(roadPts[1] - roadPts[0]);
        else roadDir = normalize(roadPts[roadPts.size()-2] - roadPts[roadPts.size()-1]); // N to N-1

        std::vector<WorldPosition> nextPts;
        bool cut = false;
        for (size_t i = 0; i < trimmedCenterline_.size(); ++i) {
            WorldPosition P = trimmedCenterline_[i];
            // Valid if P is "in front" of the radius line
            double dist = dot(P - C, roadDir);
            if (dist >= R) {
                if (!cut && i > 0) {
                    // Interpolate
                    WorldPosition P_prev = trimmedCenterline_[i-1];
                    double dist_prev = dot(P_prev - C, roadDir);
                    if (std::abs(dist - dist_prev) > 1e-5) {
                        double t = (R - dist_prev) / (dist - dist_prev);
                        nextPts.push_back(P_prev + (P - P_prev) * t);
                    }
                }
                nextPts.push_back(P);
                cut = true;
            }
        }
        if (nextPts.size() >= 2) trimmedCenterline_ = nextPts;
    }

    // --- 2. TRIM END (Entering Intersection / Stop Line) ---
    if (exitIntersection_) {
        // Logic:
        // A. Is there a Custom Stop Position (Traffic Light Node)? -> Use that.
        // B. Else -> Use Intersection Curb Radius.

        WorldPosition clipPoint;
        WorldPosition clipNormal;

        // Define direction pointing INTO the intersection (End of road)
        WorldPosition roadDirIn;
        if (isForward_) {
            size_t n = roadPts.size();
            roadDirIn = normalize(roadPts[n-1] - roadPts[n-2]);
        } else {
            roadDirIn = normalize(roadPts[0] - roadPts[1]);
        }

        if (customStopPos_.has_value()) {
            // OPTION A: Trim at specific Traffic Light Node
            clipPoint = customStopPos_.value();
            clipNormal = roadDirIn;
        } else {
            // OPTION B: Trim at Intersection Radius
            WorldPosition center = exitIntersection_->getPosition();
            double R = exitIntersection_->getCurbRadius();
            // The "Stop Point" on the center axis is Center - (Dir * R)
            clipPoint = center - (roadDirIn * R);
            clipNormal = roadDirIn;
        }

        std::vector<WorldPosition> nextPts;
        bool terminated = false;

        for (size_t i = 0; i < trimmedCenterline_.size(); ++i) {
            WorldPosition P = trimmedCenterline_[i];

            // Plane Equation: (P - ClipPoint) dot Normal <= 0
            // We want points BEFORE the line.
            double val = dot(P - clipPoint, clipNormal);

            if (val <= 0.001) { // Epsilon
                nextPts.push_back(P);
            } else {
                // Crossed the line
                if (i > 0) {
                    WorldPosition P_prev = trimmedCenterline_[i-1];
                    double val_prev = dot(P_prev - clipPoint, clipNormal);
                    // Solve for val = 0; only interpolate forward (t >= 0).
                    // If val_prev > 0 (kept by epsilon but slightly past the clip
                    // plane), t would be negative — extrapolating backward and
                    // reversing the last segment direction, which corrupts vIn.
                    if (std::abs(val - val_prev) > 1e-5) {
                        double t = (0.0 - val_prev) / (val - val_prev);
                        if (t >= 0.0)
                            nextPts.push_back(P_prev + (P - P_prev) * t);
                    }
                }
                terminated = true;
                break;
            }
        }
        if (nextPts.size() >= 2) trimmedCenterline_ = nextPts;
    }

    // Recalculate length
    trimmedLength_ = 0.0;
    for (size_t i = 1; i < trimmedCenterline_.size(); ++i) {
        trimmedLength_ += length(trimmedCenterline_[i] - trimmedCenterline_[i-1]);
    }
}


// call when agent enters this lane
void SimulationLane::registerAgent(SimulationAgent* agent) {
    agentQueue_.enqueueTail(agent);
    // TODO add debug
}

// call when agent leaves this lane
void SimulationLane::deregisterAgent(SimulationAgent* agent) {
    agentQueue_.remove(agent);
}

SimulationLane* SimulationLane::getAdjacentLane(int indexOffset) const {
    int targetIdx = laneIndex_ + indexOffset;
    if (targetIdx < 0) return nullptr;
    for (SimulationLane* lane : parentRoad_->getLanes()) {
        if (lane != this && lane->isForward() == isForward_ && lane->getLaneIndex() == targetIdx)
            return lane;
    }
    return nullptr;
}

// --- Setters for Graph Building ---
void SimulationLane::setStartIntersection(SimulationIntersection* intersection) {
    startIntersection_ = intersection;
}

void SimulationLane::setExitIntersection(SimulationIntersection* intersection) {
    exitIntersection_ = intersection;
}