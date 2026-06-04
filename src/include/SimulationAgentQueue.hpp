#pragma once

#include "SimulationAgent.hpp"
#include <cstddef>

/**
 * @class SimulationAgentQueue
 * @brief Manages a doubly‐linked list representing a queue of SimulationAgents in back‐to‐front order.
 *
 * Each SimulationAgent in the queue has pointers to its predecessor and successor (not shown here,
 * but assumed to be part of the SimulationAgent). The queue maintains a pointer to the head
 * (front) and tail (back) SimulationAgents, as well as its current size.
 */
class SimulationAgentQueue {
public:
    SimulationAgentQueue() = default;

    // --- Getters ---
    [[nodiscard]] SimulationAgent* getHead() const { return head_; }
    [[nodiscard]] SimulationAgent* getTail() const { return tail_; }
    [[nodiscard]] std::size_t getSize() const { return size_; }

    /**
     * @brief Update head/tail when an agent moves to a new memory address.
     *
     * Called after a swap-and-pop in the agent pool to keep the queue
     * consistent without a full traversal.
     */
    void replacePointer(SimulationAgent* oldPtr, SimulationAgent* newPtr) {
        if (head_ == oldPtr) head_ = newPtr;
        if (tail_ == oldPtr) tail_ = newPtr;
    }

    // --- Queue Operations ---

    /**
     * @brief Insert an Agent at the tail (back) of the queue.
     *
     * Adds the given Agent pointer to the end of the linked list, updates head/tail
     * pointers and increases size by one.
     *
     * @param agent Pointer to the Agent to enqueue at the back. Must not be nullptr.
     */
    void enqueueTail(SimulationAgent* agent);

    /**
     * @brief Insert an Agent at the head (front) of the queue.
     *
     * Adds the given Agent pointer to the front of the linked list, updates head/tail
     * pointers and increases size by one.
     *
     * @param agent Pointer to the Agent to enqueue at the front. Must not be nullptr.
     */
    void enqueueHead(SimulationAgent* agent);

    /**
     * @brief Insert an Agent immediately after a specified Agent in the queue.
     *
     * If `prev` is nullptr, the new Agent is inserted at the head (same as enqueueHead).
     * Otherwise, it is inserted directly after the specified `prev` Agent, updates
     * successor/predecessor pointers accordingly, and increases size by one.
     *
     * @param agent Pointer to the Agent to insert. Must not be nullptr.
     * @param prev  Pointer to the Agent after which to insert `a`. If nullptr, insertion
     *             happens at the head of the queue.
     */
    void insertAfter(SimulationAgent* agent, SimulationAgent* prev);

    /**
     * @brief Remove an Agent from whatever position it occupies in the queue.
     *
     * Detaches the specified Agent from the linked list (updating its neighbors' pointers),
     * updates head/tail pointers if necessary, and decreases size by one.
     *
     * @param agent Pointer to the Agent to remove from the queue. Must not be nullptr
     *          and must already be in this queue.
     */
    void remove(SimulationAgent* agent);

    /**
     * @brief Pop and return the Agent at the head (front) of the queue.
     *
     * Removes the front Agent from the queue, updates head/tail pointers and size,
     * and returns a pointer to the removed Agent. If the queue is empty, returns nullptr.
     *
     * @return Pointer to the removed head Agent, or nullptr if the queue was empty.
     */
    SimulationAgent* popHead();

    /**
     * @brief Pop and return the Agent at the tail (back) of the queue.
     *
     * Removes the back Agent from the queue, updates head/tail pointers and size,
     * and returns a pointer to the removed Agent. If the queue is empty, returns nullptr.
     *
     * @return Pointer to the removed tail Agent, or nullptr if the queue was empty.
     */
    SimulationAgent* popTail();

private:
    SimulationAgent* head_ = nullptr;   /// Pointer to the front (head) Agent in the queue, or nullptr if empty.
    SimulationAgent* tail_ = nullptr;   /// Pointer to the back (tail) Agent in the queue, or nullptr if empty.
    std::size_t size_ = 0;              /// Number of Agents currently in the queue.
};