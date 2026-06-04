#include "../include/SimulationAgentQueue.hpp"
#include "../include/SimulationAgent.hpp"

// The way traffic flows can make this feel counter intuitive, we take the look from head, so the second car is the next car of the first
// TAIL        nextInLane   a   prevInLane          HEAD
// <---------------O--------O-------O-------------------|

// Insert at tail
void SimulationAgentQueue::enqueueTail(SimulationAgent* agent) {
    agent->setPrevInLane(tail_);
    agent->setNextInLane(nullptr);      // next in lane is noone
    if (tail_) {
        tail_->setNextInLane(agent);
    } else {
        head_ = agent;
    }
    tail_ = agent;
    ++size_;
}

// Insert at head
void SimulationAgentQueue::enqueueHead(SimulationAgent* agent) {
    agent->setNextInLane(head_);
    agent->setPrevInLane(nullptr);
    if (head_) {
        head_->setPrevInLane(agent);
    } else {
        tail_ = agent;
    }
    head_ = agent;
    ++size_;
}

// Insert a after prev (or at head if prev==nullptr)
void SimulationAgentQueue::insertAfter(SimulationAgent* agent, SimulationAgent* prev) {
    if (!prev) {
        enqueueHead(agent);
    } else if (prev == tail_) {
        enqueueTail(agent);
    } else {
        SimulationAgent* nxt = prev->getNextInLane();
        agent->setPrevInLane(prev);
        agent->setNextInLane(nxt);
        prev->setNextInLane(agent);
        if (nxt) {
            nxt->setPrevInLane(agent);
        }
        ++size_;
    }
}

// Remove a from wherever it is
void SimulationAgentQueue::remove(SimulationAgent* agent) {
    SimulationAgent* p = agent->getPrevInLane();
    SimulationAgent* n = agent->getNextInLane();
    if (p) {
        p->setNextInLane(n);
    } else {
        head_ = n;
    }
    if (n) {
        n->setPrevInLane(p);
    } else {
        tail_ = p;
    }
    agent->setPrevInLane(nullptr);
    agent->setNextInLane(nullptr);
    --size_;
}

// Pop the head
SimulationAgent* SimulationAgentQueue::popHead() {
    SimulationAgent* agent = head_;
    if (agent) {
        remove(agent);
    }
    return agent;
}

// Pop the tail
SimulationAgent* SimulationAgentQueue::popTail() {
    SimulationAgent* agent = tail_;
    if (agent) {
        remove(agent);
    }
    return agent;
}