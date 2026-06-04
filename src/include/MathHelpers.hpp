#pragma once

#include "SimulationTypes.hpp"
#include <cmath>

/**
 * @file MathHelpers.hpp
 * @brief Provides inline helper functions for 2D vector math
 * using the WorldPosition struct.
 */

// --- Vector Operations ---

/**
 * @brief Calculates the squared length of a vector.
 */
inline double lengthSq(const WorldPosition& v) {
    return v.x * v.x + v.y * v.y;
}

/**
 * @brief Calculates the length (magnitude) of a vector.
 */
inline double length(const WorldPosition& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

/**
 * @brief Returns a normalized version of the vector (unit length).
 */
inline WorldPosition normalize(const WorldPosition& v) {
    const double len = length(v);
    return (len > 0.0) ? WorldPosition{v.x / len, v.y / len} : WorldPosition{0.0, 0.0};
}

/**
 * @brief Returns a vector rotated 90 degrees counter-clockwise.
 */
inline WorldPosition perp(const WorldPosition& v) {
    return WorldPosition{-v.y, v.x};
}

/**
 * @brief Calculates the dot product of two vectors.
 */
inline double dot(const WorldPosition& a, const WorldPosition& b) {
    return a.x * b.x + a.y * b.y;
}

// --- Operator Overloads ---

inline WorldPosition operator+(const WorldPosition& a, const WorldPosition& b) {
    return {a.x + b.x, a.y + b.y};
}

inline WorldPosition operator-(const WorldPosition& a, const WorldPosition& b) {
    return {a.x - b.x, a.y - b.y};
}

inline WorldPosition operator*(const WorldPosition& v, const double s) {
    return {v.x * s, v.y * s};
}

inline WorldPosition operator*(const double s, const WorldPosition& v) {
    return {v.x * s, v.y * s};
}

inline WorldPosition operator/(const WorldPosition& v, const double s) {
    return {v.x / s, v.y / s};
}

inline WorldPosition& operator+=(WorldPosition& a, const WorldPosition& b) {
    a.x += b.x;
    a.y += b.y;
    return a;
}

inline WorldPosition& operator-=(WorldPosition& a, const WorldPosition& b) {
    a.x -= b.x;
    a.y -= b.y;
    return a;
}

/**
 * @brief Checks if segment p1-p2 intersects p3-p4.
 * @param intersection Point of intersection if found.
 * @return true if intersecting
 */
inline bool getSegmentIntersection(
    const WorldPosition& p1, const WorldPosition& p2,
    const WorldPosition& p3, const WorldPosition& p4,
    WorldPosition& intersection)
{
    WorldPosition s1 = p2 - p1;
    WorldPosition s2 = p4 - p3;

    double s = (-s1.y * (p1.x - p3.x) + s1.x * (p1.y - p3.y)) / (-s2.x * s1.y + s1.x * s2.y);
    double t = ( s2.x * (p1.y - p3.y) - s2.y * (p1.x - p3.x)) / (-s2.x * s1.y + s1.x * s2.y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
        intersection.x = p1.x + (t * s1.x);
        intersection.y = p1.y + (t * s1.y);
        return true;
    }
    return false;
}

/**
 * Simple Mercator projection for small areas (cities)
 * @param lat Latitude
 * @param lon Longitude
 * @param refLat Reference Latitude to calculate from
 * @param refLon Reference Longitude to calculate from
 * @return Relative x,y coordinates to reference
 */
inline WorldPosition latLonToWorld(const double lat, const double lon, const double refLat, const double refLon) {
    constexpr double R = 6378137.0; // Earth Radius
    constexpr double DEG2RAD = 3.1415926535 / 180.0;

    double x = (lon - refLon) * (DEG2RAD * R * std::cos(refLat * DEG2RAD));
    double y = (lat - refLat) * (DEG2RAD * R);

    return {x, y};
}