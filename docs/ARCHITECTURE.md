# Architecture

## System topology

```mermaid
graph TD
    subgraph Backend ["C++ Backend Process"]
        T1["Thread 1: Simulation loop\nSimulationEngine::step(dt)\nFixed 16.6ms tick"]
        T2["Thread 2: Broadcast loop\n30 fps · JSON snapshot → WS clients"]
        T3["Thread 3: HTTP server\ncpp-httplib · REST API"]
        T4["Thread 4+: Headless runners\nPer-run SimulationEngine\nNo WS, no 30fps cap"]
        Mutex["engineMutex\ngSnapshotMutex"]
        T1 --> Mutex
        T2 --> Mutex
        T3 --> Mutex
    end

    subgraph Frontend ["Frontend  :8080  (Vanilla JS + WebGL 2.0)"]
        idx["index.html — project selector"]
        app["app.html — simulation shell"]
        analytics["analytics.html — analysis dashboard"]
    end

    subgraph DB ["PostgreSQL 16  :5432 (optional)"]
        tables["simulation_runs\nagent_trips\nintersection_snapshots\nanalyses / analysis_sessions"]
    end

    Backend -->|"WebSocket JSON · 30 fps"| Frontend
    Backend -->|"HTTP REST on demand"| Frontend
    Backend -->|"libpq (SQL)"| DB
```

---

## Backend source layout

```
src/
  include/   ← all .hpp headers
  src/        ← all .cpp implementations
```

**Entry point:** `src/src/main.cpp` — wires up threads, registers all HTTP routes, owns the global `SimulationEngine` instance.

### Core classes

| Class | Role |
|---|---|
| `SimulationEngine` | Public API — PIMPL wrapper. All external code calls this. |
| `SimulationWorld` | Owns all state: nodes, roads, lanes, intersections, agents, spawner, planner, parking |
| `SimulationAgent` | Vehicle: position, heading, speed, IDM physics, active controller |
| `SimulationRoad` | OSM way → multiple directed `SimulationLane`s with lateral offsets |
| `SimulationLane` | Ordered agent queue, centerline geometry, capacity |
| `SimulationIntersection` | Shared node, all turning paths, controller |
| `SimulationIntersectionController` | Reservation-based conflict avoidance |
| `AgentControllers` (6 impls) | Strategy pattern: high-level agent decisions |
| `InternalRoutePlanner` | Dijkstra on lane graph, congestion-aware costs, route cache |
| `InternalSpawnSystem` | Bounding-box spawn regions, rate-based vehicle creation |
| `ParkingSystem` | Spot generation from OSM tags, occupancy tracking |
| `OSMMapLoader` | Parses `.osm` XML → structured map JSON consumed by `SimulationEngine` |
| `MetricsCollector` | Writes trip/intersection data to PostgreSQL |

---

## Data flow: OSM → simulation

```mermaid
flowchart LR
    osm[".osm file"] --> loader["OSMMapLoader\nparseOSMtoJSON()"]
    loader -->|"filter highway=*\nextract lanes, oneway, maxspeed\nparking:lane:*\nbuilding centroids"| json["Structured JSON"]
    json --> engine["SimulationEngine\nloadMapFromOSM()"]
    engine --> world["SimulationWorld\nbuildGraph()"]
    world --> nodes["nodes\nunordered_map<int64_t, SimulationNode>"]
    world --> roads["roads\nunordered_map<int64_t, SimulationRoad>"]
    world --> isect["intersections\ndetected at shared nodes"]
    world --> parking["parking spots\nfrom road parking config"]
```

---

## Data flow: simulation tick

```mermaid
flowchart TD
    tick["SimulationWorld::step(dt)"]
    tick --> spawn["1. InternalSpawnSystem::tick()\nemit new vehicles at configured rate"]
    spawn --> agents["2. For each agent:\nIAgentController::update()\nSimulationAgent::stepPhysics() ← IDM"]
    agents --> isect["3. SimulationIntersectionController::tick()\nprocess reservations"]
    isect --> cleanup["4. Remove completed/dead agents\nswap-and-pop + fixMovedAgent()"]
    cleanup --> metrics["5. MetricsCollector::recordTrip()\nfor any completed trips"]
```

---

## Data flow: render frame

```mermaid
sequenceDiagram
    participant Sim as Simulation loop (T1)
    participant Bcast as Broadcast loop (T2)
    participant Client as Browser (60 fps)

    Sim->>Bcast: update world state (engineMutex)
    loop Every ~33ms
        Bcast->>Client: JSON snapshot over WebSocket
    end
    loop Every frame (16ms)
        Client->>Client: interpolate vehicle positions
        Client->>Client: WebGL instanced draw
    end
```

---

## Agent controller state machine

```mermaid
stateDiagram-v2
    [*] --> LaneDriving : spawned
    LaneDriving --> IntersectionEntry : end of lane
    IntersectionEntry --> IntersectionCrossing : path reserved
    IntersectionCrossing --> LaneDriving : exit intersection
    LaneDriving --> ParkingSearch : parking route
    ParkingSearch --> ParkingApproach : spot claimed
    ParkingApproach --> ParkingDeparture : parked (timer set)
    ParkingDeparture --> LaneDriving : departure timer expired
    LaneDriving --> [*] : route complete
```

---

## Agent pool memory model

Agents are stored as **value objects** in a pre-allocated `std::vector` (capacity 16,384 by default, configurable in `simulation.json`).

- Lane queues hold **raw non-owning pointers** to agents
- Addresses are stable because the vector never reallocates past its initial capacity
- Removal uses **swap-and-pop** followed by `fixMovedAgent()`, which updates all lane queue pointers to the swapped agent's new address
- Every new removal path must call `fixMovedAgent()` — failure produces dangling pointers

---

## Headless simulation (optimization use case)

`POST /analysis/start` spawns N worker threads, each running:

1. A fresh `SimulationEngine` loaded with the same OSM map
2. Optionally with modified road closures applied
3. Stepped as fast as possible (no WS broadcast, no 30 fps cap)
4. Metrics flushed to PostgreSQL on completion

Multiple headless runs compare baseline vs. modified scenarios to compute fitness scores. See [Analytics](ANALYTICS.md) for the scoring formula.