# Project Overview

## Purpose

This project is a **road traffic simulation engine** developed as a bachelor's thesis at [Charles University, Faculty of Mathematics and Physics
](https://www.mff.cuni.cz/en) by Jakub Kos. It models vehicle movement on real road networks (OpenStreetMap data) and is designed to serve as a **fitness function for road network optimization**: given a set of candidate road closures or construction phases, the engine runs headless parallel simulations and compares congestion metrics to identify configurations that minimize traffic delay.

The project is fully deployable via Docker and includes a WebGL frontend for interactive exploration of simulation results.

---

## Architecture

Three independently deployable services communicate over a local network:

| Service   | Technology                                | Port                          |
|-----------|-------------------------------------------|-------------------------------|
| Backend   | C++23, CMake, cpp-httplib, IXWebSocket    | `:9001` (HTTP) · `:9002` (WS) |
| Frontend  | Vanilla JS + WebGL 2.0, served by nginx   | `:8080`                       |
| Database  | PostgreSQL 16 (optional — analytics only) | `:5432`                       |

In Docker mode, nginx on `:8080` serves the frontend and proxies all HTTP API calls to the backend internally; only WebSocket `:9002` is exposed directly.

See [Architecture](ARCHITECTURE.md) for the threading model, data-flow diagrams, and full class descriptions.

---

## Simulation Model

- **World representation:** OSM graph → Nodes → Ways → Roads → directed Lanes → Intersections
- **Car-following:** Intelligent Driver Model (IDM) — continuous acceleration based on gap to leader and relative speed
- **Intersection handling:** reservation-based conflict avoidance (precomputed conflict graph, no traffic lights required)
- **Routing:** Dijkstra on the lane graph with congestion-aware edge costs, route cache (TTL-limited), and random-walk fallback
- **Parking:** spots generated from OSM `parking:lane:*` tags; agents park, wait a random duration, and re-merge into traffic
- **Spawning:** bounding-box spawn regions with configurable rates and vehicle type profiles

See [Simulation internals](SIMULATION.md) for the full model description, IDM parameters, and controller state machine.

---

## Optimization System

Road network optimization runs **headless** — no UI, no WebSocket broadcast, no 30 fps cap. The simulation engine is treated as a black-box fitness function:

1. Define a set of candidate road IDs and a phase count (which roads to close in which phase)
2. The optimizer generates candidate phase assignments and evaluates each by running N parallel headless simulation instances
3. The fitness metric is **DANCE** (Demand-Adjusted Network Cost with Exclusions): average congestion delay per spawned vehicle, with a penalty term for vehicles that never complete their journey
4. The algorithm searches for the assignment that minimizes DANCE subject to a per-phase constraint threshold

Available algorithms: hill climbing, simulated annealing, evolutionary algorithm. A sensitivity analysis mode ranks individual roads by their marginal impact on DANCE, useful as a pre-filter before running full optimization.

See [Analytics & Metrics](ANALYTICS.md) for the DANCE formula, scoring, and PostgreSQL schema.

---

## Implementation Status

| Component | Status |
|---|---|
| OSM loading → lane graph build | Done |
| IDM car-following physics | Done |
| Intersection reservation system | Done |
| Congestion-aware Dijkstra routing | Done |
| Parking system (approach / park / depart) | Done |
| WebGL frontend — agents / density / congestion views | Done |
| Project management (create / save / load / delete) | Done |
| Road editor (close roads, adjust speeds) | Done |
| Analytics DB + run lifecycle (PostgreSQL) | Done |
| Optimizer (hill climbing, SA, EA) | Done |
| Sensitivity analysis | Done |
| Docker deployment (3 images + compose + nginx) | Done |
| CI/CD pipeline (GitLab → Docker Hub on tag) | Done |

---

## Key Files

| File | Purpose |
|---|---|
| `CMakeLists.txt` | Build config: C++23, FetchContent deps, optional TBB for parallel agent updates |
| `simulation.json` | Default map path, ports, vehicle physics, routing parameters, agent pool size |
| `db/init.sql` | Full PostgreSQL schema: analytics runs, trips, intersections, optimizer sessions |
| `projects/<Name>/meta.json` | Project metadata (name, description, timestamps) |
| `projects/<Name>/map.osm` | OSM map file saved for a project |
| `docker-compose.yml` | Orchestrates backend, frontend (nginx), and database services |
| `Dockerfile` | Backend: Ubuntu 24.04 multi-stage build — compiles with CMake, strips to runtime image |
| `Dockerfile.frontend` | Frontend: copies `frontend/` into nginx:alpine |
| `Dockerfile.db` | Database: PostgreSQL 16 with `init.sql` pre-applied |
| `nginx.conf` | Serves static files; falls back to proxying API requests to backend |
| `.gitlab-ci.yml` | CI: Kaniko builds all three images to Docker Hub on each git tag |

---

## Design Decisions

For tradeoff analysis and early design choices (why IDM, why OSM,...) see [AnalysisOfApproaches.md](AnalysisOfApproaches.md).