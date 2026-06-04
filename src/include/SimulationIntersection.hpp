#pragma once

#include "SimulationTypes.hpp"
#include "SimulationIntersectionPath.hpp"
#include "SimulationIntersectionController.hpp"
#include <vector>
#include <string>

// Forward declaration
class SimulationLane;

/**
 * @class SimulationIntersection
 * @brief Represents a road intersection, containing lanes, turning paths, and control logic.
 *
 * An Intersection holds a unique ID, its position in world coordinates, lists of incoming
 * and outgoing lanes, all possible IntersectionPath objects (turning movements), and an
 * IntersectionController to manage path reservations. Also stores a curb radius used for
 * drawing or path generation.
 */
class SimulationIntersection {
public:
    /**
     * @brief Construct an intersection with a given ID, world position, and curb radius.
     *
     * Initializes the intersection's unique identifier, world position, and optional curb
     * radius. Incoming/outgoing lane lists and internal paths start empty.
     *
     * @param id         Unique int64 identifier for this intersection (e.g., "42").
     * @param pos        World‐coordinate position (x,y) of the intersection center.
     * @param curbRadius Optional curb radius for drawing or path‐generation (default = CURB_RADIUS).
     */
    SimulationIntersection(int64_t id, WorldPosition pos, double curbRadius);

    // --- Getters ---
    [[nodiscard]] int64_t getId() const { return id_; }

    /**
     * @brief Get the world position of the intersection center.
     * @return Constant reference to the WorldPosition representing the intersection position.
     */
    [[nodiscard]] WorldPosition getPosition() const { return position_; }

    /**
     * @brief Get the curb radius used for drawing or path calculations.
     * @return The double value of curbRadius_.
     */
    [[nodiscard]] double getCurbRadius() const { return curbRadius_; }

    /**
     * @brief Get the IntersectionController for this intersection.
     *
     * Allows calling canEnter(), reserve(), and release() on intersection paths.
     *
     * @return Reference to the IntersectionController instance.
     */
    SimulationIntersectionController& getController() { return controller_; }

    /**
     * @brief Get all turning paths within this intersection.
     *
     * Returns a non‐const reference to the vector of IntersectionPath objects
     * representing every valid from -> to movement. The caller may clear or iterate over this list.
     *
     * @return Reference to the vector of IntersectionPath.
     */
    [[nodiscard]] const std::vector<SimulationIntersectionPath>& getPaths() const { return paths_; }

    // --- Graph Building ---
    /**
     * @brief Add a new incoming lane to this intersection.
     *
     * Appends the given Lane pointer to entryLanes_, representing a lane whose end‐node
     * is this intersection.
     *
     * @param lane Pointer to the Lane to be added as incoming.
     */
    void addEntryLane(SimulationLane* lane) { entryLanes_.push_back(lane); }

    /**
     * @brief Add a new outgoing lane to this intersection.
     *
     * Appends the given Lane pointer to exitLanes_, representing a lane whose start‐node
     * is this intersection.
     *
     * @param lane Pointer to the Lane to be added as outgoing.
     */
    void addExitLane(SimulationLane* lane) { exitLanes_.push_back(lane); }

    /**
     * @brief Add a new turning path to this intersection.
     *
     * Appends the given IntersectionPath object (by move) into the internal paths_ vector.
     * The path should have its from/to lanes and pathPoints_ already set.
     *
     * @param path IntersectionPath object to add (by value). Moved into paths_.
     */
    void addPath(SimulationIntersectionPath path) { paths_.push_back(std::move(path)); }

    /**
     * @brief Get the list of incoming lanes (lanes that end at this intersection).
     *
     * Returns a non‐const reference to the vector, allowing iteration or addition/removal.
     *
     * @return Reference to the vector of incoming Lane pointers.
     */
    [[nodiscard]] const std::vector<SimulationLane*>& getEntryLanes() const { return entryLanes_; }

    /**
     * @brief Get the list of outgoing lanes (lanes that start at this intersection).
     *
     * Returns a non‐const reference to the vector, allowing iteration or addition/removal.
     *
     * @return Reference to the vector of outgoing Lane pointers.
     */
    [[nodiscard]] const std::vector<SimulationLane*>& getExitLanes() const { return exitLanes_; }

    /**
     * @brief Remove all entry and exit lane lists.
     *
     * Clears both entryLanes_ and exitLanes_ vectors. Use before rebuilding lane connections.
     */
    void clearLanes() { entryLanes_.clear(); exitLanes_.clear(); }

    /**
     * @brief Remove all existing turning paths from this intersection.
     *
     * Clears the internal vector of IntersectionPath. Any Agents still holding references
     * to old paths will become invalid, so call only when no Agents remain in the intersection.
     */
    void clearPaths() { paths_.clear(); }

    // --- Conflict points ---
    void computeConflictPoints();  ///< Derive all path-crossing conflict points from the current path set.
    [[nodiscard]] const std::vector<ConflictPoint>& getConflictPoints() const { return conflicts_; }
    /** @brief Returns the subset of conflict points that involve the given path; empty vector if path is null. */
    [[nodiscard]] const std::vector<ConflictPoint>& getConflictsForPath(const SimulationIntersectionPath* path) const;

    /** @brief Advance signal-controller state by dt seconds. */
    void update(double dt);
    /** @brief Returns the index of path inside paths_, or -1 if not found. */
    [[nodiscard]] int getPathIndex(const SimulationIntersectionPath* path) const;

    void setSignalControlled(bool v) { isSignalControlled_ = v; }
    [[nodiscard]] bool isSignalControlled() const { return isSignalControlled_; }
private:
    int64_t       id_;          ///<  Unique OSM node ID.
    WorldPosition position_;    ///<  World-coordinate centre of the intersection.
    double        curbRadius_;  ///<  Curb radius used for path generation and rendering.

    bool isSignalControlled_ = false;

    std::vector<SimulationLane*> entryLanes_;        ///< Non-owning; lanes whose end-node is this intersection.
    std::vector<SimulationLane*> exitLanes_;         ///< Non-owning; lanes whose start-node is this intersection.

    std::vector<SimulationIntersectionPath> paths_;  ///< Owned; all turning paths (from → to) for this intersection.
    SimulationIntersectionController controller_;    ///< Owned; manages which paths are currently reserved.

    std::vector<ConflictPoint> conflicts_;                         ///< All pairwise conflict points, computed once after graph build.
    static const std::vector<ConflictPoint> kEmptyConflicts_;     ///< Returned by getConflictsForPath() when path is null.
};