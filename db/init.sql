/*
 * Note: Claude (Anthropic) was used to assist with commenting this file.
 * The db definitions were written by the author.
 */

-- Traffic Simulation Analytics Schema
-- Project isolation: every row carries project_name so queries never bleed across projects.
-- Run isolation:     every row carries run_id (UUID) so runs never bleed into each other.

CREATE EXTENSION IF NOT EXISTS "pgcrypto";  -- for gen_random_uuid()

-- ── simulation_runs ──────────────────────────────────────────────────────────
-- One row per simulation run.  Created when the user clicks "Start Run",
-- finalised when the sim ends or the user clicks "Finish".
CREATE TABLE simulation_runs (
    run_id          TEXT        PRIMARY KEY DEFAULT gen_random_uuid()::TEXT,
    project_name    TEXT        NOT NULL,
    run_type        TEXT        NOT NULL CHECK (run_type IN ('baseline', 'scenario', 'optimization')),
    label           TEXT        NOT NULL DEFAULT '',
    created_at      TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    status          TEXT        NOT NULL DEFAULT 'running'
                                CHECK (status IN ('running', 'completed', 'failed')),

    -- Which roads were closed for this run (array of OSM way IDs)
    closed_road_ids JSONB       NOT NULL DEFAULT '[]',

    -- Populated on completion
    sim_duration_s          FLOAT,
    vehicles_spawned        INT,
    vehicles_completed      INT,
    completion_rate         FLOAT,   -- vehicles_completed / vehicles_spawned

    -- The core metric: sum of (actual - freeflow) travel times, in hours
    total_excess_vehicle_hours  FLOAT,
    avg_excess_time_per_trip_s  FLOAT,

    -- DANC (Demand-Adjusted Network Cost): accounts for unserved agents.
    -- danc_score = (excess_s + unserved * T_penalty) / spawned  [seconds per agent]
    -- T_penalty = max(300, avg_freeflow_all_spawned_s * 2)
    avg_freeflow_all_spawned_s  FLOAT,   -- Σfreeflow over ALL spawned agents / spawned_count
    danc_score                  FLOAT,

    -- Fitness relative to baseline (NULL for baseline runs, 1.0 = same as baseline)
    -- fitness_score > 1.0 means worse than baseline, < 1.0 means better
    baseline_run_id TEXT REFERENCES simulation_runs(run_id),
    fitness_score   FLOAT
);

CREATE INDEX idx_runs_project  ON simulation_runs(project_name);
CREATE INDEX idx_runs_type     ON simulation_runs(project_name, run_type);
CREATE INDEX idx_runs_created  ON simulation_runs(project_name, created_at DESC);

-- ── agent_trips ───────────────────────────────────────────────────────────────
-- One row per completed vehicle trip.
-- Used to compute total_excess_vehicle_hours on finalization,
-- and for per-trip analysis (histograms, O/D breakdown, etc.).
CREATE TABLE agent_trips (
    id                   BIGSERIAL PRIMARY KEY,
    run_id               TEXT  NOT NULL REFERENCES simulation_runs(run_id) ON DELETE CASCADE,
    agent_id             BIGINT NOT NULL,
    spawn_time_s         FLOAT  NOT NULL,
    complete_time_s      FLOAT  NOT NULL,
    actual_travel_time_s FLOAT  NOT NULL,
    freeflow_estimate_s  FLOAT  NOT NULL,   -- Σ(lane_length / speed_limit) at spawn
    excess_time_s        FLOAT  NOT NULL,   -- max(0, actual - freeflow)
    reroute_count        INT    NOT NULL DEFAULT 0
);

CREATE INDEX idx_trips_run     ON agent_trips(run_id);
CREATE INDEX idx_trips_excess  ON agent_trips(run_id, excess_time_s DESC);

-- ── intersection_snapshots ────────────────────────────────────────────────────
-- Coarse periodic samples (every ~60 sim-seconds) of intersection stress.
-- Only written when locked_conflicts >= 2 to keep the table lean.
-- Used for post-run "where did congestion form?" analysis.
CREATE TABLE intersection_snapshots (
    id               BIGSERIAL PRIMARY KEY,
    run_id           TEXT   NOT NULL REFERENCES simulation_runs(run_id) ON DELETE CASCADE,
    sim_time_s       FLOAT  NOT NULL,
    intersection_id  BIGINT NOT NULL,
    locked_conflicts INT    NOT NULL DEFAULT 0
);

CREATE INDEX idx_isect_run  ON intersection_snapshots(run_id);
CREATE INDEX idx_isect_time ON intersection_snapshots(run_id, sim_time_s);

-- ── analyses ──────────────────────────────────────────────────────────────────
-- Top-level grouping: one Analysis contains multiple Simulations (sessions),
-- typically a baseline + several closure scenarios for comparison.
CREATE TABLE analyses (
    analysis_id  TEXT        PRIMARY KEY DEFAULT gen_random_uuid()::TEXT,
    project_name TEXT        NOT NULL,
    name         TEXT        NOT NULL,
    sim_seed     BIGINT      NOT NULL DEFAULT 0,  -- shared RNG seed for all sims in this analysis
    created_at   TIMESTAMPTZ NOT NULL DEFAULT NOW()
);
CREATE INDEX idx_analyses_project ON analyses(project_name, created_at DESC);

-- ── analysis_sessions ─────────────────────────────────────────────────────────
-- One simulation configuration (N parallel headless runs → averaged results).
-- Belongs to an Analysis. One session per analysis is the baseline.
CREATE TABLE analysis_sessions (
    session_id      TEXT        PRIMARY KEY DEFAULT gen_random_uuid()::TEXT,
    project_name    TEXT        NOT NULL,
    name            TEXT        NOT NULL DEFAULT '',
    traffic_mode    TEXT        NOT NULL DEFAULT 'medium'
                                CHECK (traffic_mode IN ('low', 'medium', 'high')),
    run_count       INT         NOT NULL DEFAULT 1,
    sim_duration_s  FLOAT       NOT NULL DEFAULT 3600.0,
    closed_road_ids JSONB       NOT NULL DEFAULT '[]',
    created_at      TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    status          TEXT        NOT NULL DEFAULT 'running'
                                CHECK (status IN ('running', 'completed', 'failed')),

    -- Which Analysis group this simulation belongs to
    analysis_id         TEXT REFERENCES analyses(analysis_id) ON DELETE CASCADE,
    -- True for the reference (no-closure) simulation within an analysis
    is_baseline         BOOLEAN NOT NULL DEFAULT FALSE,

    -- Baseline session this was compared against (for fitness score)
    baseline_session_id TEXT REFERENCES analysis_sessions(session_id),

    -- Averaged results (populated once all runs in the session complete)
    avg_fitness_score           FLOAT,
    avg_excess_vehicle_hours    FLOAT,
    avg_excess_time_per_trip_s  FLOAT,
    avg_completion_rate         FLOAT,
    avg_vehicles_spawned        FLOAT,
    avg_vehicles_completed      FLOAT,
    avg_danc_score              FLOAT,
    avg_freeflow_all_spawned_s  FLOAT,
    -- demand-normalised DANC: uses S_baseline as denominator for every session
    -- so values are directly comparable across baseline and scenarios
    avg_danc_comparable         FLOAT
);

CREATE INDEX idx_sessions_project ON analysis_sessions(project_name);
CREATE INDEX idx_sessions_created ON analysis_sessions(project_name, created_at DESC);

-- Add session linkage to simulation_runs (CASCADE so deleting a session cleans up runs)
ALTER TABLE simulation_runs
    ADD COLUMN session_id TEXT REFERENCES analysis_sessions(session_id) ON DELETE CASCADE;

-- ── Optimizer tables ──────────────────────────────────────────────────────────
CREATE TABLE optimization_problems (
    problem_id           TEXT PRIMARY KEY DEFAULT gen_random_uuid()::TEXT,
    project_name         TEXT  NOT NULL,
    name                 TEXT  NOT NULL DEFAULT '',
    candidate_road_ids   JSONB NOT NULL DEFAULT '[]',
    phase_count          INT   NOT NULL DEFAULT 2,
    sim_duration_s       FLOAT NOT NULL DEFAULT 3600.0,
    traffic_mode         TEXT  NOT NULL DEFAULT 'medium',
    scenario_type        TEXT  NOT NULL DEFAULT 'phased_construction',
    constraint_threshold FLOAT NOT NULL DEFAULT 1.15,
    created_at           TIMESTAMPTZ NOT NULL DEFAULT NOW()
);
CREATE INDEX idx_optprob_project ON optimization_problems(project_name, created_at DESC);

CREATE TABLE optimization_sessions (
    session_id           TEXT PRIMARY KEY DEFAULT gen_random_uuid()::TEXT,
    problem_id           TEXT NOT NULL REFERENCES optimization_problems(problem_id) ON DELETE CASCADE,
    project_name         TEXT NOT NULL,
    algorithm            TEXT NOT NULL DEFAULT 'hill_climbing',
    hyperparams          JSONB NOT NULL DEFAULT '{}',
    status               TEXT NOT NULL DEFAULT 'running'
                         CHECK (status IN ('running', 'paused', 'completed', 'failed')),
    created_at           TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    iterations_done      INT   NOT NULL DEFAULT 0,
    evals_done           INT   NOT NULL DEFAULT 0,
    total_evals          INT   NOT NULL DEFAULT 0,
    avg_sim_time_ms      FLOAT,
    best_fitness         FLOAT,
    baseline_danc        FLOAT,
    best_assignment      JSONB NOT NULL DEFAULT '[]',
    best_phase_fitnesses JSONB NOT NULL DEFAULT '[]',
    convergence_history  JSONB NOT NULL DEFAULT '[]',
    candidate_road_ids   JSONB NOT NULL DEFAULT '[]'
);
CREATE INDEX idx_optsess_problem ON optimization_sessions(problem_id);
