#pragma once

#include "SimulationTypes.hpp"
#include <string>

/**
 * @class SimulationNode
 * @brief An OSM node: a geographic point converted to world coordinates, with an intersection type tag.
 */
class SimulationNode {
public:
    SimulationNode(const int64_t id, const WorldPosition pos, const IntersectionType type = IntersectionType::None)
        : id_(id), position_(pos), type_(type) {}

    [[nodiscard]] int64_t          getId()      const { return id_; }
    [[nodiscard]] WorldPosition    getPosition() const { return position_; }
    [[nodiscard]] IntersectionType getType()    const { return type_; }

    void setPosition(WorldPosition pos) { position_ = pos; }
    void setType(IntersectionType t)    { type_ = t; }

private:
    int64_t          id_;        ///<  Unique OSM node ID.
    WorldPosition    position_;  ///<  World-coordinate position (converted from lat/lon).
    IntersectionType type_;      ///<  How this node behaves as an intersection (None, Priority, Signal, …).
};