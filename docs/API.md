# API Reference

**HTTP base:** `http://localhost:9001`  
**WebSocket:** `ws://localhost:9002/`

Ports are configurable in `simulation.json` via `httpPort` and `wsPort`.

---

## HTTP Endpoints

### Map

| Method | Path | Body | Description |
|---|---|---|---|
| GET | `/map` | — | Full static map JSON (roads, lanes, intersections) |
| POST | `/map/reset` | — | Reload default map from `simulation.json` |
| POST | `/map/reload` | raw OSM XML | Reinitialize engine with new OSM data |

### Simulation control

| Method | Path | Body | Description |
|---|---|---|---|
| GET | `/simulation/project` | — | Currently loaded project name |
| GET | `/simulation/status` | — | `{simTime, timeScale, metrics}` |
| PUT | `/simulation/speed` | `{value: 0.0–100.0}` | Set global time scale (0 = paused) |
| POST | `/simulation/speed` | `{value: 0.0–100.0}` | Legacy alias for PUT |

### Vehicles

| Method | Path | Params/Body | Description |
|---|---|---|---|
| GET | `/vehicles` | `?minX&minY&maxX&maxY` | All vehicle states in bounding box |
| GET | `/vehicle/detail` | `?id=<vehicleId>` | Full state + debug hitbox + position history |
| PUT | `/vehicle/route` | `{vehicleId, routeId}` | Reassign route to specific vehicle |

### Road management

| Method | Path | Body | Description |
|---|---|---|---|
| PUT | `/road/close` | `{wayId, isClosed}` | Close or reopen a road (triggers rerouting) |
| GET | `/road/closed` | — | Array of currently closed OSM way IDs |

### Metrics

| Method | Path | Params | Description |
|---|---|---|---|
| GET | `/metrics` | — | Aggregated counts + speed stats |
| GET | `/heatmap` | `?minX&minY&maxX&maxY&resX&resY` | KDE vehicle density grid |
| GET | `/heatmap/lanes` | — | Per-lane `{id, count, stuck, speed}` |
| GET | `/spawning` | — | Spawn enabled flag + per-region info |
| POST | `/spawning` | `{enabled: bool}` | Toggle vehicle spawning |
| GET | `/parking/spots` | — | All parking spot positions + occupancy |
| GET | `/intersections/positions` | — | `[{id, x, y}]` for all intersections |

### Analytics runs (DB required)

| Method | Path | Body | Description |
|---|---|---|---|
| POST | `/runs/start` | `{runType, label}` | Create run record, start metrics tracking |
| POST | `/runs/:id/finish` | — | Finalize run, compute fitness score |
| POST | `/runs/:id/fail` | — | Mark run as failed |
| GET | `/runs/active` | — | Active run ID (if any) |
| GET | `/projects/:name/runs` | — | All runs for a project |

### Analyses — comparison studies (DB required)

| Method | Path | Body | Description |
|---|---|---|---|
| POST | `/analyses` | `{name}` | Create analysis group |
| GET | `/analyses/:id/sessions` | — | List sessions in analysis |
| DELETE | `/analyses/:id` | — | Delete analysis + cancel running sessions |
| GET | `/sessions/:id/hotspots` | — | Top congestion intersections |
| GET | `/sessions/:id/trips` | — | Trip travel-time histogram |
| POST | `/analysis/start` | `{analysisId, label, runCount, simDurationS, trafficMode}` | Start N parallel headless runs |
| GET | `/analysis/:id/status` | — | Session progress `[{runId, progress: 0.0–1.0}]` |
| DELETE | `/analysis/:id` | — | Cancel running session |

### Projects

| Method | Path | Body | Description |
|---|---|---|---|
| GET | `/projects` | — | List all projects (name, description, dates) |
| POST | `/projects` | `{name, description}` | Create project directory |
| GET | `/projects/:name` | — | Project metadata |
| PATCH | `/projects/:name` | `{name, description}` | Update metadata |
| DELETE | `/projects/:name` | — | Delete project directory |
| POST | `/projects/:name/osm` | raw OSM XML | Save OSM map to project |
| GET | `/projects/:name/osm/raw` | — | Serve raw OSM XML |
| POST | `/projects/:name/load` | — | Load project map into running simulation |

---

## WebSocket Protocol

### Server → Client (every ~33ms)

```jsonc
{
  "simTime": 123.45,
  "timeScale": 2.0,
  "stepMs": 5.2,          // actual ms per sim step (performance indicator)
  "maxTimeScale": 4.5,    // auto-throttled max based on step time

  // --- One of three payload modes (switched via client cmd) ---

  // MODE: "agents" (default)
  "vehicles": [
    {
      "id": 1,
      "x": 100.5, "y": 200.3,
      "h": 0.785,           // heading in radians
      "speed": 5.2,
      "acceleration": 0.5,
      "leaderId": 2,        // agent ahead in lane (-1 if none)
      "gap": 8.3,           // gap to leader in meters
      "parked": false,
      "timeUntilDeparture": 0.0
    }
  ],

  // MODE: "density"
  "density": {
    "minX": -500, "minY": -500, "maxX": 500, "maxY": 500,
    "resX": 80, "resY": 60,
    "cells": [0, 2, 1, ...]   // vehicle count per grid cell, row-major
  },

  // MODE: "congestion"
  "laneStats": [
    { "id": 123, "count": 5, "stuck": 2, "speed": 3.5 }
  ],

  // Always present:
  "intersections": [
    {
      "id": 12345,
      "locks": [{ "c": 0, "a": 1 }],       // conflict_id, agent_id
      "lights": [{ "p": 0, "s": "green" }]  // path_index, signal_state
    }
  ],
  "metrics": {
    "totalVehicles": 150,
    "vehiclesInQueue": 95,
    "vehiclesInIntersection": 20,
    "yieldingVehicles": 5,
    "avgSpeed": 8.2,
    "maxSpeed": 15.0,
    "avgGapToLeader": 12.5,
    "totalSpawned": 500,
    "totalCompleted": 300,
    "throughputPerMinute": 18.5,
    "avgTravelTime": 240.0
  }
}
```

### Client → Server commands

```jsonc
// Change simulation speed
{ "cmd": "setSpeed", "value": 1.5 }

// Set viewport for server-side culling (reduces bandwidth)
{ "cmd": "setViewport", "minX": -1000, "minY": -1000, "maxX": 1000, "maxY": 1000 }

// Switch stream mode
{ "cmd": "setStreamMode", "mode": "agents" }   // or "density" or "congestion"

// Close/open a road
{ "cmd": "setRoadClosed", "wayId": 12345, "isClosed": true }

// Reassign vehicle route
{ "cmd": "setVehicleRoute", "vehicleId": 5, "routeId": "some_route_id" }
```