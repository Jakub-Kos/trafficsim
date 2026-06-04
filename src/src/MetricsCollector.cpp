/*
 * Note: Claude (Anthropic) was used under human supervision to assist with
 * implementing this file. The code is largely repetitive SQL boilerplate where
 * the risk of subtle mistakes (wrong column order, mismatched parameter counts)
 * outweighs the complexity of the logic itself — making it a practical candidate
 * for AI-assisted authoring with the author reviewing, testing, and owning the result.
 */
#include "../include/MetricsCollector.hpp"

#include <libpq-fe.h>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>

// ── helpers ──────────────────────────────────────────────────────────────────

static std::string escLit(PGconn* conn, const std::string& s) {
    char* e = PQescapeLiteral(conn, s.c_str(), s.size());
    std::string result(e);
    PQfreemem(e);
    return result;
}

static std::string dbl(double v) {
    if (!std::isfinite(v)) return "0";
    std::ostringstream ss;
    ss << v;
    return ss.str();
}

// ── MetricsCollector ──────────────────────────────────────────────────────────

MetricsCollector::MetricsCollector() {
    connect();
}

MetricsCollector::~MetricsCollector() {
    if (conn_) {
        PQfinish(conn_);
        conn_ = nullptr;
    }
}

void MetricsCollector::connect() {
    const char* envUrl = std::getenv("POSTGRES_URL");
    const std::string connStr = envUrl
        ? std::string(envUrl)
        : "postgresql://trafficsim:trafficsim@localhost:5432/trafficsim";

    conn_ = PQconnectdb(connStr.c_str());
    if (PQstatus(conn_) != CONNECTION_OK) {
        std::cerr << "[MetricsCollector] DB unavailable (" << PQerrorMessage(conn_)
                  << ") — metrics collection disabled.\n";
        PQfinish(conn_);
        conn_ = nullptr;
    } else {
        std::cout << "[MetricsCollector] Connected to Postgres.\n";
    }
}

bool MetricsCollector::isConnected() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return conn_ != nullptr && PQstatus(conn_) == CONNECTION_OK;
}

bool MetricsCollector::execSimple(const std::string& sql) {
    if (!conn_) return false;
    PGresult* res = PQexec(conn_, sql.c_str());
    ExecStatusType status = PQresultStatus(res);
    bool ok = (status == PGRES_COMMAND_OK || status == PGRES_TUPLES_OK);
    if (!ok) {
        std::cerr << "[MetricsCollector] SQL error: " << PQerrorMessage(conn_)
                  << "\nSQL: " << sql.substr(0, 200) << "\n";
    }
    PQclear(res);
    return ok;
}

// ── createAnalysis ────────────────────────────────────────────────────────────

std::string MetricsCollector::createAnalysis(const std::string& projectName,
                                              const std::string& name,
                                              uint64_t seed) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return "";

    std::ostringstream sql;
    sql << "INSERT INTO analyses (project_name, name, sim_seed) VALUES ("
        << escLit(conn_, projectName) << ","
        << escLit(conn_, name) << ","
        << static_cast<int64_t>(seed)
        << ") RETURNING analysis_id";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK || PQntuples(res) == 0) {
        std::cerr << "[MetricsCollector] createAnalysis failed: " << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return "";
    }
    std::string analysisId(PQgetvalue(res, 0, 0));
    PQclear(res);
    std::cout << "[MetricsCollector] Analysis created: " << analysisId << " (" << name << ", seed=" << seed << ")\n";
    return analysisId;
}

// ── queryAnalysisSeed ─────────────────────────────────────────────────────────

uint64_t MetricsCollector::queryAnalysisSeed(const std::string& analysisId) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_ || analysisId.empty()) return 0;

    std::ostringstream sql;
    sql << "SELECT sim_seed FROM analyses WHERE analysis_id=" << escLit(conn_, analysisId);
    PGresult* res = PQexec(conn_, sql.str().c_str());
    uint64_t seed = 0;
    if (PQresultStatus(res) == PGRES_TUPLES_OK && PQntuples(res) > 0) {
        seed = static_cast<uint64_t>(std::stoll(PQgetvalue(res, 0, 0)));
    }
    PQclear(res);
    return seed;
}

// ── deleteAnalysis ────────────────────────────────────────────────────────────

void MetricsCollector::deleteAnalysis(const std::string& analysisId) {
    if (analysisId.empty()) return;
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return;

    // Manual cascade: runs → sessions → analysis (agent_trips/snapshots cascade from runs)
    std::ostringstream delRuns;
    delRuns << "DELETE FROM simulation_runs WHERE session_id IN "
            << "(SELECT session_id FROM analysis_sessions WHERE analysis_id="
            << escLit(conn_, analysisId) << ")";
    execSimple(delRuns.str());

    std::ostringstream delSessions;
    delSessions << "DELETE FROM analysis_sessions WHERE analysis_id="
                << escLit(conn_, analysisId);
    execSimple(delSessions.str());

    std::ostringstream delAnalysis;
    delAnalysis << "DELETE FROM analyses WHERE analysis_id=" << escLit(conn_, analysisId);
    execSimple(delAnalysis.str());
    std::cout << "[MetricsCollector] Analysis deleted: " << analysisId << "\n";
}

// ── queryAnalysesForProject ───────────────────────────────────────────────────

std::string MetricsCollector::queryAnalysesForProject(const std::string& projectName) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return "[]";

    auto jStr = [](const std::string& s) -> std::string {
        std::string r = "\"";
        for (char c : s) {
            if (c == '"')  r += "\\\"";
            else if (c == '\\') r += "\\\\";
            else r += c;
        }
        r += '"';
        return r;
    };

    // Query 1: all analyses for project
    std::ostringstream aSql;
    aSql << "SELECT analysis_id, name, "
         << "to_char(created_at AT TIME ZONE 'UTC', 'YYYY-MM-DD\"T\"HH24:MI:SS\"Z\"') "
         << "FROM analyses WHERE project_name=" << escLit(conn_, projectName)
         << " ORDER BY created_at DESC";
    PGresult* aRes = PQexec(conn_, aSql.str().c_str());
    if (PQresultStatus(aRes) != PGRES_TUPLES_OK) {
        std::cerr << "[MetricsCollector] queryAnalysesForProject(analyses) failed: "
                  << PQerrorMessage(conn_) << "\n";
        PQclear(aRes);
        return "[]";
    }
    int aRows = PQntuples(aRes);

    // Query 2: all sessions for this project's analyses
    std::ostringstream sSql;
    sSql << "SELECT session_id, name, traffic_mode, run_count, sim_duration_s, "
         << "to_char(created_at AT TIME ZONE 'UTC', 'YYYY-MM-DD\"T\"HH24:MI:SS\"Z\"'), "
         << "status, is_baseline::TEXT, closed_road_ids::TEXT, "
         << "COALESCE(avg_fitness_score::TEXT,'null'), "
         << "COALESCE(avg_excess_vehicle_hours::TEXT,'null'), "
         << "COALESCE(avg_excess_time_per_trip_s::TEXT,'null'), "
         << "COALESCE(avg_completion_rate::TEXT,'null'), "
         << "COALESCE(avg_vehicles_spawned::TEXT,'null'), "
         << "COALESCE(avg_vehicles_completed::TEXT,'null'), "
         << "COALESCE(analysis_id,''), "
         << "COALESCE(baseline_session_id,''), "
         << "COALESCE(avg_danc_score::TEXT,'null'), "
         << "COALESCE(avg_freeflow_all_spawned_s::TEXT,'null'), "
         << "COALESCE(avg_danc_comparable::TEXT,'null') "
         << "FROM analysis_sessions "
         << "WHERE analysis_id IN "
         << "(SELECT analysis_id FROM analyses WHERE project_name=" << escLit(conn_, projectName) << ") "
         << "ORDER BY created_at ASC";
    PGresult* sRes = PQexec(conn_, sSql.str().c_str());
    if (PQresultStatus(sRes) != PGRES_TUPLES_OK) {
        std::cerr << "[MetricsCollector] queryAnalysesForProject(sessions) failed: "
                  << PQerrorMessage(conn_) << "\n";
        PQclear(aRes);
        PQclear(sRes);
        return "[]";
    }
    int sRows = PQntuples(sRes);

    // Build nested JSON
    std::ostringstream out;
    out << '[';
    for (int i = 0; i < aRows; ++i) {
        if (i > 0) out << ',';
        auto aCol = [&](int c) -> std::string {
            return PQgetisnull(aRes, i, c) ? "" : std::string(PQgetvalue(aRes, i, c));
        };
        const std::string analysisId = aCol(0);
        out << '{'
            << "\"analysisId\":"  << jStr(analysisId) << ','
            << "\"name\":"        << jStr(aCol(1))    << ','
            << "\"createdAt\":"   << jStr(aCol(2))    << ','
            << "\"sessions\":[";
        bool firstSess = true;
        for (int j = 0; j < sRows; ++j) {
            auto sCol = [&](int c) -> std::string {
                return PQgetisnull(sRes, j, c) ? "" : std::string(PQgetvalue(sRes, j, c));
            };
            if (sCol(15) != analysisId) continue; // filter by analysis_id
            if (!firstSess) out << ',';
            firstSess = false;
            out << '{'
                << "\"sessionId\":"            << jStr(sCol(0))  << ','
                << "\"name\":"                 << jStr(sCol(1))  << ','
                << "\"trafficMode\":"          << jStr(sCol(2))  << ','
                << "\"runCount\":"             << sCol(3)        << ','
                << "\"simDurationS\":"         << sCol(4)        << ','
                << "\"createdAt\":"            << jStr(sCol(5))  << ','
                << "\"status\":"               << jStr(sCol(6))  << ','
                << "\"isBaseline\":"           << (sCol(7) == "t" ? "true" : "false") << ','
                << "\"closedRoadIds\":"        << sCol(8)        << ','
                << "\"avgFitnessScore\":"      << sCol(9)        << ','
                << "\"avgExcessVehicleHours\":" << sCol(10)      << ','
                << "\"avgExcessTimeSec\":"     << sCol(11)       << ','
                << "\"avgCompletionRate\":"    << sCol(12)       << ','
                << "\"avgVehiclesSpawned\":"   << sCol(13)       << ','
                << "\"avgVehiclesCompleted\":" << sCol(14)       << ','
                << "\"analysisId\":"           << jStr(sCol(15)) << ','
                << "\"baselineSessionId\":"        << jStr(sCol(16)) << ','
                << "\"avgDancScore\":"             << sCol(17)       << ','
                << "\"avgFreeflowAllSpawnedS\":"   << sCol(18)       << ','
                << "\"avgDancComparable\":"         << sCol(19)
                << '}';
        }
        out << "]}";
    }
    out << ']';
    PQclear(aRes);
    PQclear(sRes);
    return out.str();
}

// ── querySessionsForAnalysis ──────────────────────────────────────────────────

std::string MetricsCollector::querySessionsForAnalysis(const std::string& analysisId) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_ || analysisId.empty()) return "[]";

    auto jStr = [](const std::string& s) -> std::string {
        std::string r = "\"";
        for (char c : s) {
            if (c == '"')  r += "\\\"";
            else if (c == '\\') r += "\\\\";
            else r += c;
        }
        r += '"';
        return r;
    };

    std::ostringstream sql;
    sql << "SELECT session_id, name, traffic_mode, run_count, sim_duration_s, "
        << "to_char(created_at AT TIME ZONE 'UTC', 'YYYY-MM-DD\"T\"HH24:MI:SS\"Z\"'), "
        << "status, is_baseline::TEXT, closed_road_ids::TEXT, "
        << "COALESCE(avg_fitness_score::TEXT,'null'), "
        << "COALESCE(avg_excess_vehicle_hours::TEXT,'null'), "
        << "COALESCE(avg_excess_time_per_trip_s::TEXT,'null'), "
        << "COALESCE(avg_completion_rate::TEXT,'null'), "
        << "COALESCE(avg_vehicles_spawned::TEXT,'null'), "
        << "COALESCE(avg_vehicles_completed::TEXT,'null'), "
        << "COALESCE(avg_danc_score::TEXT,'null'), "
        << "COALESCE(avg_freeflow_all_spawned_s::TEXT,'null') "
        << "FROM analysis_sessions WHERE analysis_id=" << escLit(conn_, analysisId)
        << " ORDER BY created_at ASC";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "[MetricsCollector] querySessionsForAnalysis failed: "
                  << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return "[]";
    }

    int rows = PQntuples(res);
    std::ostringstream out;
    out << '[';
    for (int i = 0; i < rows; ++i) {
        if (i > 0) out << ',';
        auto col = [&](int c) -> std::string {
            return PQgetisnull(res, i, c) ? "" : std::string(PQgetvalue(res, i, c));
        };
        out << '{'
            << "\"sessionId\":"            << jStr(col(0))  << ','
            << "\"name\":"                 << jStr(col(1))  << ','
            << "\"trafficMode\":"          << jStr(col(2))  << ','
            << "\"runCount\":"             << col(3)        << ','
            << "\"simDurationS\":"         << col(4)        << ','
            << "\"createdAt\":"            << jStr(col(5))  << ','
            << "\"status\":"               << jStr(col(6))  << ','
            << "\"isBaseline\":"           << (col(7) == "t" ? "true" : "false") << ','
            << "\"closedRoadIds\":"        << col(8)        << ','
            << "\"avgFitnessScore\":"      << col(9)        << ','
            << "\"avgExcessVehicleHours\":" << col(10)      << ','
            << "\"avgExcessTimeSec\":"     << col(11)       << ','
            << "\"avgCompletionRate\":"    << col(12)       << ','
            << "\"avgVehiclesSpawned\":"   << col(13)       << ','
            << "\"avgVehiclesCompleted\":"      << col(14) << ','
            << "\"avgDancScore\":"              << col(15) << ','
            << "\"avgFreeflowAllSpawnedS\":"    << col(16) << ','
            << "\"avgDancComparable\":"          << col(17)
            << '}';
    }
    out << ']';
    PQclear(res);
    return out.str();
}

// ── queryHotspotsForSession ───────────────────────────────────────────────────

std::string MetricsCollector::queryHotspotsForSession(const std::string& sessionId) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_ || sessionId.empty()) return "[]";

    std::ostringstream sql;
    sql << "SELECT iss.intersection_id, SUM(iss.locked_conflicts) AS total_conflicts "
        << "FROM intersection_snapshots iss "
        << "JOIN simulation_runs sr ON iss.run_id = sr.run_id "
        << "WHERE sr.session_id = " << escLit(conn_, sessionId)
        << " AND sr.status = 'completed' "
        << "GROUP BY iss.intersection_id "
        << "ORDER BY total_conflicts DESC LIMIT 20";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "[MetricsCollector] queryHotspotsForSession failed: "
                  << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return "[]";
    }

    int rows = PQntuples(res);
    std::ostringstream out;
    out << '[';
    for (int i = 0; i < rows; ++i) {
        if (i > 0) out << ',';
        auto col = [&](int c) -> std::string {
            return PQgetisnull(res, i, c) ? "0" : std::string(PQgetvalue(res, i, c));
        };
        out << "{\"intersectionId\":" << col(0)
            << ",\"conflictCount\":"   << col(1)
            << '}';
    }
    out << ']';
    PQclear(res);
    return out.str();
}

// ── queryTripsForSession ──────────────────────────────────────────────────────

std::string MetricsCollector::queryTripsForSession(const std::string& sessionId) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_ || sessionId.empty()) return "[]";

    std::ostringstream sql;
    sql << "SELECT at.actual_travel_time_s, at.excess_time_s, at.reroute_count "
        << "FROM agent_trips at "
        << "JOIN simulation_runs sr ON at.run_id = sr.run_id "
        << "WHERE sr.session_id = " << escLit(conn_, sessionId)
        << " AND sr.status = 'completed' "
        << "ORDER BY at.actual_travel_time_s "
        << "LIMIT 3000";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "[MetricsCollector] queryTripsForSession failed: "
                  << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return "[]";
    }

    int rows = PQntuples(res);
    std::ostringstream out;
    out << '[';
    for (int i = 0; i < rows; ++i) {
        if (i > 0) out << ',';
        auto col = [&](int c) -> std::string {
            return PQgetisnull(res, i, c) ? "0" : std::string(PQgetvalue(res, i, c));
        };
        out << "{\"actualTravelTimeSec\":" << col(0)
            << ",\"excessTimeSec\":"        << col(1)
            << ",\"rerouteCount\":"         << col(2)
            << '}';
    }
    out << ']';
    PQclear(res);
    return out.str();
}

// ── createSession ─────────────────────────────────────────────────────────────

std::string MetricsCollector::createSession(const std::string& projectName,
                                             const std::string& name,
                                             const std::string& trafficMode,
                                             int runCount,
                                             double simDurationS,
                                             const std::string& closedRoadIds,
                                             bool isBaseline,
                                             const std::string& analysisId,
                                             const std::string& baselineSessionId) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return "";

    const std::string safeRoads = closedRoadIds.empty() ? "[]" : closedRoadIds;

    std::ostringstream sql;
    sql << "INSERT INTO analysis_sessions "
        << "(project_name,name,traffic_mode,run_count,sim_duration_s,closed_road_ids,is_baseline";
    if (!analysisId.empty())        sql << ",analysis_id";
    if (!baselineSessionId.empty()) sql << ",baseline_session_id";
    sql << ") VALUES ("
        << escLit(conn_, projectName) << ","
        << escLit(conn_, name)        << ","
        << escLit(conn_, trafficMode) << ","
        << runCount                   << ","
        << dbl(simDurationS)          << ","
        << escLit(conn_, safeRoads)   << "::jsonb,"
        << (isBaseline ? "TRUE" : "FALSE");
    if (!analysisId.empty())
        sql << "," << escLit(conn_, analysisId);
    if (!baselineSessionId.empty())
        sql << "," << escLit(conn_, baselineSessionId);
    sql << ") RETURNING session_id";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK || PQntuples(res) == 0) {
        std::cerr << "[MetricsCollector] createSession failed: " << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return "";
    }
    std::string sessionId(PQgetvalue(res, 0, 0));
    PQclear(res);
    std::cout << "[MetricsCollector] Session created: " << sessionId << " (" << name << ")\n";
    return sessionId;
}

// ── finalizeSession ───────────────────────────────────────────────────────────

void MetricsCollector::finalizeSession(const std::string& sessionId) {
    if (sessionId.empty()) return;
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return;

    // ── Self-heal: link baseline_session_id if it was never set ──────────────
    // Happens when the scenario was created while the baseline was still running,
    // causing the creation-time lookup to miss it.
    {
        std::ostringstream hq;
        hq << "UPDATE analysis_sessions sc "
           << "SET baseline_session_id = bl.session_id "
           << "FROM analysis_sessions bl "
           << "WHERE sc.session_id = " << escLit(conn_, sessionId)
           << " AND sc.baseline_session_id IS NULL "
           << " AND sc.is_baseline = FALSE "
           << " AND bl.analysis_id = sc.analysis_id "
           << " AND bl.is_baseline = TRUE";
        execSimple(hq.str());
    }

    // ── Aggregate metrics from all completed runs in this session ─────────────
    std::ostringstream agg;
    agg << "SELECT "
        << "AVG(total_excess_vehicle_hours), "        // 0
        << "AVG(avg_excess_time_per_trip_s), "        // 1
        << "AVG(completion_rate), "                   // 2
        << "AVG(vehicles_spawned), "                  // 3
        << "AVG(vehicles_completed), "                // 4
        << "AVG(danc_score), "                        // 5
        << "AVG(avg_freeflow_all_spawned_s) "         // 6
        << "FROM simulation_runs "
        << "WHERE session_id = " << escLit(conn_, sessionId)
        << " AND status = 'completed'";

    PGresult* res = PQexec(conn_, agg.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK || PQntuples(res) == 0) {
        std::cerr << "[MetricsCollector] finalizeSession aggregation failed: "
                  << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return;
    }

    auto nullOrDbl = [&](int c) -> std::string {
        return PQgetisnull(res, 0, c) ? "NULL" : std::string(PQgetvalue(res, 0, c));
    };
    auto getOrZero = [&](int c) -> double {
        return PQgetisnull(res, 0, c) ? 0.0 : std::stod(PQgetvalue(res, 0, c));
    };

    std::string avgExcessH    = nullOrDbl(0);
    std::string avgExcessS    = nullOrDbl(1);
    std::string avgCompletion = nullOrDbl(2);
    std::string avgSpawned    = nullOrDbl(3);
    std::string avgCompleted  = nullOrDbl(4);
    std::string avgDanc       = nullOrDbl(5);
    std::string avgFreeflow   = nullOrDbl(6);

    // Raw doubles needed for fitness computation
    double thisExcessS    = getOrZero(0) * 3600.0;  // convert vh → seconds
    double thisCompleted  = getOrZero(4);
    PQclear(res);

    std::string avgFitness         = "NULL";
    std::string avgDancComparable  = "NULL";

    // Find baseline_session_id for this session
    std::string baselineSessionId;
    {
        std::ostringstream bsq;
        bsq << "SELECT baseline_session_id FROM analysis_sessions WHERE session_id = "
            << escLit(conn_, sessionId);
        PGresult* bsr = PQexec(conn_, bsq.str().c_str());
        if (PQresultStatus(bsr) == PGRES_TUPLES_OK && PQntuples(bsr) > 0
                && !PQgetisnull(bsr, 0, 0))
            baselineSessionId = PQgetvalue(bsr, 0, 0);
        PQclear(bsr);
    }

    if (!baselineSessionId.empty()) {
        // Fetch baseline's aggregated metrics
        std::ostringstream blq;
        blq << "SELECT "
            << "AVG(total_excess_vehicle_hours)*3600, "   // 0  excess_s
            << "AVG(vehicles_spawned), "                  // 1  S_baseline
            << "AVG(vehicles_completed), "                // 2  completed_baseline
            << "AVG(sim_duration_s) "                     // 3  sim duration
            << "FROM simulation_runs "
            << "WHERE session_id = " << escLit(conn_, baselineSessionId)
            << " AND status = 'completed'";
        PGresult* blr = PQexec(conn_, blq.str().c_str());

        if (PQresultStatus(blr) == PGRES_TUPLES_OK && PQntuples(blr) > 0
                && !PQgetisnull(blr, 0, 0)) {
            double blExcessS   = std::stod(PQgetvalue(blr, 0, 0));
            double blSpawned   = std::stod(PQgetvalue(blr, 0, 1));
            double blCompleted = std::stod(PQgetvalue(blr, 0, 2));
            double blSimDur    = PQgetisnull(blr, 0, 3) ? 3600.0
                                  : std::stod(PQgetvalue(blr, 0, 3));

            // T_penalty = sim_duration / 2
            // An unserved agent spawned uniformly over the sim period and never
            // completed; their expected wasted time is half the simulation length.
            // This is independent of congestion level and always >> avg excess delay,
            // ensuring denied trips are penalised more heavily than delayed ones.
            double T_penalty = blSimDur / 2.0;

            double unservedScenario = std::max(0.0, blSpawned - thisCompleted);
            double unservedBaseline = std::max(0.0, blSpawned - blCompleted);

            double numerator   = thisExcessS + unservedScenario * T_penalty;
            double denominator = blExcessS   + unservedBaseline * T_penalty;

            // avg_danc_comparable: absolute demand-normalised cost in s/agent
            // Uses S_baseline as denominator so ALL sessions are on the same scale
            if (blSpawned > 0.0)
                avgDancComparable = dbl(numerator / blSpawned);

            if (denominator > 0.0)
                avgFitness = dbl(numerator / denominator);

            std::cout << "[MetricsCollector] Fitness: T_pen=" << T_penalty
                      << " S_bl=" << blSpawned
                      << " danc_cmp=" << avgDancComparable
                      << " f=" << avgFitness << "\n";
        }
        PQclear(blr);
    } else {
        // Baseline session: compute danc_comparable using own data as reference
        // (same formula as scenario, but S and T_penalty come from self)
        double ownSpawned   = (avgSpawned   != "NULL") ? std::stod(avgSpawned)   : 0.0;
        double ownCompleted = (avgCompleted != "NULL") ? std::stod(avgCompleted) : 0.0;

        // Query own sim_duration_s for T_penalty
        double ownSimDur = 3600.0;
        {
            std::ostringstream sdq;
            sdq << "SELECT AVG(sim_duration_s) FROM simulation_runs "
                << "WHERE session_id = " << escLit(conn_, sessionId)
                << " AND status = 'completed'";
            PGresult* sdr = PQexec(conn_, sdq.str().c_str());
            if (PQresultStatus(sdr) == PGRES_TUPLES_OK && PQntuples(sdr) > 0
                    && !PQgetisnull(sdr, 0, 0))
                ownSimDur = std::stod(PQgetvalue(sdr, 0, 0));
            PQclear(sdr);
        }
        double T_penalty = ownSimDur / 2.0;
        double unservedOwn = std::max(0.0, ownSpawned - ownCompleted);
        double numerator   = thisExcessS + unservedOwn * T_penalty;
        if (ownSpawned > 0.0)
            avgDancComparable = dbl(numerator / ownSpawned);
    }

    std::ostringstream upd;
    upd << "UPDATE analysis_sessions SET "
        << "status = 'completed', "
        << "avg_excess_vehicle_hours = "   << avgExcessH    << ", "
        << "avg_excess_time_per_trip_s = " << avgExcessS    << ", "
        << "avg_completion_rate = "        << avgCompletion << ", "
        << "avg_vehicles_spawned = "       << avgSpawned    << ", "
        << "avg_vehicles_completed = "     << avgCompleted  << ", "
        << "avg_danc_score = "             << avgDanc            << ", "
        << "avg_freeflow_all_spawned_s = " << avgFreeflow        << ", "
        << "avg_danc_comparable = "        << avgDancComparable  << ", "
        << "avg_fitness_score = "          << avgFitness
        << " WHERE session_id = "          << escLit(conn_, sessionId);
    execSimple(upd.str());
    std::cout << "[MetricsCollector] Session finalised: " << sessionId
              << "  avg_danc=" << avgDanc
              << "  fitness=" << avgFitness << "\n";
}

// ── failSession ───────────────────────────────────────────────────────────────

void MetricsCollector::failSession(const std::string& sessionId) {
    if (sessionId.empty()) return;
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return;
    std::ostringstream sql;
    sql << "UPDATE analysis_sessions SET status='failed' WHERE session_id="
        << escLit(conn_, sessionId);
    execSimple(sql.str());
}

// ── deleteSession ─────────────────────────────────────────────────────────────

void MetricsCollector::deleteSession(const std::string& sessionId) {
    if (sessionId.empty()) return;
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return;
    // First delete runs (agent_trips and intersection_snapshots cascade from them)
    std::ostringstream delRuns;
    delRuns << "DELETE FROM simulation_runs WHERE session_id=" << escLit(conn_, sessionId);
    execSimple(delRuns.str());
    // Then delete the session
    std::ostringstream delSess;
    delSess << "DELETE FROM analysis_sessions WHERE session_id=" << escLit(conn_, sessionId);
    execSimple(delSess.str());
    std::cout << "[MetricsCollector] Session deleted: " << sessionId << "\n";
}

// ── cleanupStaleRuns ──────────────────────────────────────────────────────────

void MetricsCollector::cleanupStaleRuns() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return;
    bool r1 = execSimple("UPDATE simulation_runs SET status='failed' WHERE status='running'");
    bool r2 = execSimple("UPDATE analysis_sessions SET status='failed' WHERE status='running'");
    if (r1 || r2)
        std::cout << "[MetricsCollector] Cleaned up stale running records from previous session.\n";
}

// ── querySessionsForProject ───────────────────────────────────────────────────

std::string MetricsCollector::querySessionsForProject(const std::string& projectName) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return "[]";

    std::ostringstream sql;
    sql << "SELECT session_id, name, traffic_mode, run_count, sim_duration_s, "
        << "to_char(created_at AT TIME ZONE 'UTC', 'YYYY-MM-DD\"T\"HH24:MI:SS\"Z\"'), "
        << "status, "
        << "COALESCE(avg_fitness_score::TEXT,'null'), "
        << "COALESCE(avg_excess_vehicle_hours::TEXT,'null'), "
        << "COALESCE(avg_excess_time_per_trip_s::TEXT,'null'), "
        << "COALESCE(avg_completion_rate::TEXT,'null'), "
        << "COALESCE(avg_vehicles_spawned::TEXT,'null'), "
        << "COALESCE(avg_vehicles_completed::TEXT,'null'), "
        << "closed_road_ids::TEXT, "
        << "COALESCE(baseline_session_id,'') "
        << "FROM analysis_sessions "
        << "WHERE project_name = " << escLit(conn_, projectName)
        << " ORDER BY created_at DESC";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "[MetricsCollector] querySessionsForProject failed: "
                  << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return "[]";
    }

    auto jStr = [](const std::string& s) -> std::string {
        std::string r = "\"";
        for (char c : s) {
            if (c == '"')  r += "\\\"";
            else if (c == '\\') r += "\\\\";
            else r += c;
        }
        r += '"';
        return r;
    };

    int rows = PQntuples(res);
    std::ostringstream out;
    out << '[';
    for (int i = 0; i < rows; ++i) {
        if (i > 0) out << ',';
        auto col = [&](int c) -> std::string {
            return PQgetisnull(res, i, c) ? "" : std::string(PQgetvalue(res, i, c));
        };
        out << '{'
            << "\"sessionId\":"             << jStr(col(0))  << ','
            << "\"name\":"                  << jStr(col(1))  << ','
            << "\"trafficMode\":"           << jStr(col(2))  << ','
            << "\"runCount\":"              << col(3)        << ','
            << "\"simDurationS\":"          << col(4)        << ','
            << "\"createdAt\":"             << jStr(col(5))  << ','
            << "\"status\":"                << jStr(col(6))  << ','
            << "\"avgFitnessScore\":"       << col(7)        << ','
            << "\"avgExcessVehicleHours\":" << col(8)        << ','
            << "\"avgExcessTimeSec\":"      << col(9)        << ','
            << "\"avgCompletionRate\":"     << col(10)       << ','
            << "\"avgVehiclesSpawned\":"    << col(11)       << ','
            << "\"avgVehiclesCompleted\":"  << col(12)       << ','
            << "\"closedRoadIds\":"         << col(13)       << ','
            << "\"baselineSessionId\":"     << jStr(col(14))
            << '}';
    }
    out << ']';
    PQclear(res);
    return out.str();
}

// ── startRun ──────────────────────────────────────────────────────────────────

std::string MetricsCollector::startRun(const std::string& projectName,
                                        const std::string& runType,
                                        const std::string& label,
                                        const std::string& closedRoadIds,
                                        const std::string& baselineRunId,
                                        const std::string& sessionId) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return "";

    // Validate closed_road_ids is valid JSON (fall back to empty array)
    const std::string safeRoads = closedRoadIds.empty() ? "[]" : closedRoadIds;

    std::ostringstream sql;
    sql << "INSERT INTO simulation_runs "
        << "(project_name, run_type, label, closed_road_ids";
    if (!baselineRunId.empty()) sql << ", baseline_run_id";
    if (!sessionId.empty())     sql << ", session_id";
    sql << ") VALUES ("
        << escLit(conn_, projectName) << ", "
        << escLit(conn_, runType)     << ", "
        << escLit(conn_, label)       << ", "
        << escLit(conn_, safeRoads)   << "::jsonb";
    if (!baselineRunId.empty())
        sql << ", " << escLit(conn_, baselineRunId);
    if (!sessionId.empty())
        sql << ", " << escLit(conn_, sessionId);
    sql << ") RETURNING run_id";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK || PQntuples(res) == 0) {
        std::cerr << "[MetricsCollector] startRun failed: " << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return "";
    }
    std::string runId(PQgetvalue(res, 0, 0));
    PQclear(res);
    std::cout << "[MetricsCollector] Run started: " << runId << " (" << runType << ")\n";
    return runId;
}

// ── recordTrip ────────────────────────────────────────────────────────────────

void MetricsCollector::recordTrip(const std::string& runId,
                                   int64_t agentId,
                                   double spawnTimeS,
                                   double completeTimeS,
                                   double actualTravelTimeS,
                                   double freeflowEstimateS,
                                   int rerouteCount) {
    if (runId.empty()) return;

    double excess = std::max(0.0, actualTravelTimeS - freeflowEstimateS);

    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return;

    batch_.push_back({runId, agentId, spawnTimeS, completeTimeS,
                      actualTravelTimeS, freeflowEstimateS, excess, rerouteCount});

    if ((int)batch_.size() >= BATCH_SIZE) {
        // Flush without re-locking (already holding lock)
        if (batch_.empty()) return;

        std::ostringstream sql;
        sql << "INSERT INTO agent_trips "
            << "(run_id,agent_id,spawn_time_s,complete_time_s,"
            <<  "actual_travel_time_s,freeflow_estimate_s,excess_time_s,reroute_count) VALUES ";

        bool first = true;
        for (const auto& t : batch_) {
            if (!first) sql << ',';
            first = false;
            sql << '(' << escLit(conn_, t.runId)
                << ',' << t.agentId
                << ',' << dbl(t.spawnTimeS)
                << ',' << dbl(t.completeTimeS)
                << ',' << dbl(t.actualTravelTimeS)
                << ',' << dbl(t.freeflowEstimateS)
                << ',' << dbl(t.excessTimeS)
                << ',' << t.rerouteCount
                << ')';
        }
        execSimple(sql.str());
        batch_.clear();
    }
}

// ── recordIntersectionSnapshot ────────────────────────────────────────────────

void MetricsCollector::recordIntersectionSnapshot(const std::string& runId,
                                                   double simTimeS,
                                                   int64_t intersectionId,
                                                   int lockedConflicts) {
    if (runId.empty()) return;
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return;

    std::ostringstream sql;
    sql << "INSERT INTO intersection_snapshots "
        << "(run_id,sim_time_s,intersection_id,locked_conflicts) VALUES ("
        << escLit(conn_, runId) << ','
        << dbl(simTimeS) << ','
        << intersectionId << ','
        << lockedConflicts << ')';
    execSimple(sql.str());
}

// ── flush ─────────────────────────────────────────────────────────────────────

void MetricsCollector::flush() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_ || batch_.empty()) return;

    std::ostringstream sql;
    sql << "INSERT INTO agent_trips "
        << "(run_id,agent_id,spawn_time_s,complete_time_s,"
        <<  "actual_travel_time_s,freeflow_estimate_s,excess_time_s,reroute_count) VALUES ";

    bool first = true;
    for (const auto& t : batch_) {
        if (!first) sql << ',';
        first = false;
        sql << '(' << escLit(conn_, t.runId)
            << ',' << t.agentId
            << ',' << dbl(t.spawnTimeS)
            << ',' << dbl(t.completeTimeS)
            << ',' << dbl(t.actualTravelTimeS)
            << ',' << dbl(t.freeflowEstimateS)
            << ',' << dbl(t.excessTimeS)
            << ',' << t.rerouteCount
            << ')';
    }
    execSimple(sql.str());
    batch_.clear();
}

// ── finalizeRun ───────────────────────────────────────────────────────────────

void MetricsCollector::finalizeRun(const std::string& runId,
                                    double simDurationS,
                                    int64_t spawned,
                                    double spawnedFreeflowSumS) {
    if (runId.empty()) return;

    // Flush any remaining trips first (no lock held yet — flush() takes its own)
    flush();

    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return;

    // ── Aggregate completed-trip stats ────────────────────────────────────────
    std::ostringstream agg;
    agg << "SELECT COUNT(*), "
        << "COALESCE(SUM(excess_time_s),0), "
        << "COALESCE(AVG(excess_time_s),0) "
        << "FROM agent_trips WHERE run_id = " << escLit(conn_, runId);

    PGresult* res = PQexec(conn_, agg.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "[MetricsCollector] finalizeRun aggregation failed: "
                  << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return;
    }

    int64_t completed    = std::stoll(PQgetvalue(res, 0, 0));
    double  totalExcessS = std::stod(PQgetvalue(res, 0, 1));
    double  avgExcessS   = std::stod(PQgetvalue(res, 0, 2));
    PQclear(res);

    double totalExcessHours = totalExcessS / 3600.0;
    double completionRate   = (spawned > 0) ? (double)completed / (double)spawned : 0.0;

    // ── DANC: Demand-Adjusted Network Cost ────────────────────────────────────
    // T_penalty = max(300s, avgFreeflow_allSpawned * 2)
    // Rationale: an unserved agent paid at least twice its expected freeflow time
    // (once waiting, once not travelling). Floor of 300s avoids degenerate maps.
    double avgFreeflowAllSpawned = (spawned > 0) ? spawnedFreeflowSumS / (double)spawned : 0.0;
    double T_penalty  = std::max(300.0, avgFreeflowAllSpawned * 2.0);
    double unserved   = (double)(spawned - completed);
    double dancScore  = (totalExcessS + unserved * T_penalty) / std::max((int64_t)1, spawned);

    // ── Fitness: DANC(this) / DANC(baseline), only when baseline is linked ────
    std::string baselineId;
    {
        std::ostringstream bq;
        bq << "SELECT baseline_run_id FROM simulation_runs WHERE run_id = "
           << escLit(conn_, runId);
        PGresult* br = PQexec(conn_, bq.str().c_str());
        if (PQresultStatus(br) == PGRES_TUPLES_OK && PQntuples(br) > 0
                && !PQgetisnull(br, 0, 0))
            baselineId = PQgetvalue(br, 0, 0);
        PQclear(br);
    }

    std::string fitnessStr = "NULL";
    if (!baselineId.empty()) {
        std::ostringstream bfq;
        bfq << "SELECT danc_score FROM simulation_runs WHERE run_id = "
            << escLit(conn_, baselineId);
        PGresult* bfr = PQexec(conn_, bfq.str().c_str());
        if (PQresultStatus(bfr) == PGRES_TUPLES_OK && PQntuples(bfr) > 0
                && !PQgetisnull(bfr, 0, 0)) {
            double baseDanc = std::stod(PQgetvalue(bfr, 0, 0));
            if (baseDanc > 0.0)
                fitnessStr = dbl(dancScore / baseDanc);
        }
        PQclear(bfr);
    }

    // ── Persist ───────────────────────────────────────────────────────────────
    std::ostringstream upd;
    upd << "UPDATE simulation_runs SET "
        << "status = 'completed', "
        << "sim_duration_s = "                  << dbl(simDurationS)          << ", "
        << "vehicles_spawned = "                << spawned                    << ", "
        << "vehicles_completed = "              << completed                  << ", "
        << "completion_rate = "                 << dbl(completionRate)        << ", "
        << "total_excess_vehicle_hours = "      << dbl(totalExcessHours)      << ", "
        << "avg_excess_time_per_trip_s = "      << dbl(avgExcessS)            << ", "
        << "avg_freeflow_all_spawned_s = "      << dbl(avgFreeflowAllSpawned) << ", "
        << "danc_score = "                      << dbl(dancScore)             << ", "
        << "fitness_score = "                   << fitnessStr
        << " WHERE run_id = "                   << escLit(conn_, runId);
    execSimple(upd.str());

    std::cout << "[MetricsCollector] Run finalised: " << runId
              << "  trips=" << completed << "/" << spawned
              << "  danc=" << dancScore
              << "  fitness=" << fitnessStr
              << "\n";
}

// ── failRun ───────────────────────────────────────────────────────────────────

void MetricsCollector::failRun(const std::string& runId) {
    if (runId.empty()) return;
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return;
    std::ostringstream sql;
    sql << "UPDATE simulation_runs SET status='failed' WHERE run_id="
        << escLit(conn_, runId);
    execSimple(sql.str());
}

// ── findBaselineRun ───────────────────────────────────────────────────────────

std::string MetricsCollector::findBaselineRun(const std::string& projectName) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_ || projectName.empty()) return "";

    std::ostringstream sql;
    sql << "SELECT run_id FROM simulation_runs "
        << "WHERE project_name = " << escLit(conn_, projectName)
        << " AND run_type = 'baseline' AND status = 'completed'"
        << " ORDER BY created_at DESC LIMIT 1";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    std::string result;
    if (PQresultStatus(res) == PGRES_TUPLES_OK && PQntuples(res) > 0)
        result = PQgetvalue(res, 0, 0);
    PQclear(res);
    return result;
}

// ── queryTripsForRun ──────────────────────────────────────────────────────────

std::string MetricsCollector::queryTripsForRun(const std::string& runId) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_ || runId.empty()) return "[]";

    std::ostringstream sql;
    sql << "SELECT agent_id, spawn_time_s, complete_time_s, "
        << "actual_travel_time_s, freeflow_estimate_s, excess_time_s, reroute_count "
        << "FROM agent_trips WHERE run_id = " << escLit(conn_, runId)
        << " ORDER BY excess_time_s DESC LIMIT 5000";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "[MetricsCollector] queryTripsForRun failed: "
                  << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return "[]";
    }

    int rows = PQntuples(res);
    std::ostringstream out;
    out << '[';
    for (int i = 0; i < rows; ++i) {
        if (i > 0) out << ',';
        auto col = [&](int c) -> std::string {
            return PQgetisnull(res, i, c) ? "0" : std::string(PQgetvalue(res, i, c));
        };
        out << '{'
            << "\"agent_id\":"            << col(0) << ','
            << "\"spawn_time_s\":"        << col(1) << ','
            << "\"complete_time_s\":"     << col(2) << ','
            << "\"actual_travel_time_s\":"<< col(3) << ','
            << "\"freeflow_estimate_s\":" << col(4) << ','
            << "\"excess_time_s\":"       << col(5) << ','
            << "\"reroute_count\":"       << col(6)
            << '}';
    }
    out << ']';
    PQclear(res);
    return out.str();
}

// ── createOptimizationProblem ─────────────────────────────────────────────────

std::string MetricsCollector::createOptimizationProblem(const std::string& projectName,
                                                          const std::string& name,
                                                          const std::string& candidateRoadIds,
                                                          int phaseCount,
                                                          double simDurationS,
                                                          const std::string& trafficMode,
                                                          const std::string& scenarioType,
                                                          double constraintThreshold) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return "";

    const std::string safeRoads = candidateRoadIds.empty() ? "[]" : candidateRoadIds;
    const std::string safeScen  = scenarioType.empty() ? "phased_construction" : scenarioType;

    std::ostringstream sql;
    sql << "INSERT INTO optimization_problems "
        << "(project_name, name, candidate_road_ids, phase_count, sim_duration_s, "
        << "traffic_mode, scenario_type, constraint_threshold) VALUES ("
        << escLit(conn_, projectName) << ","
        << escLit(conn_, name)        << ","
        << escLit(conn_, safeRoads)   << "::jsonb,"
        << phaseCount                 << ","
        << dbl(simDurationS)          << ","
        << escLit(conn_, trafficMode) << ","
        << escLit(conn_, safeScen)    << ","
        << dbl(constraintThreshold)
        << ") RETURNING problem_id";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK || PQntuples(res) == 0) {
        std::cerr << "[MetricsCollector] createOptimizationProblem failed: " << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return "";
    }
    std::string problemId(PQgetvalue(res, 0, 0));
    PQclear(res);
    std::cout << "[MetricsCollector] OptimizationProblem created: " << problemId << "\n";
    return problemId;
}

// ── queryOptimizationProblems ─────────────────────────────────────────────────

std::string MetricsCollector::queryOptimizationProblems(const std::string& projectName) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return "[]";

    auto jStr = [](const std::string& s) -> std::string {
        std::string r = "\"";
        for (char c : s) {
            if (c == '"')  r += "\\\"";
            else if (c == '\\') r += "\\\\";
            else r += c;
        }
        r += '"';
        return r;
    };

    std::ostringstream sql;
    sql << "SELECT problem_id, name, candidate_road_ids::TEXT, phase_count, sim_duration_s, "
        << "traffic_mode, scenario_type, constraint_threshold, "
        << "to_char(created_at AT TIME ZONE 'UTC', 'YYYY-MM-DD\"T\"HH24:MI:SS\"Z\"') "
        << "FROM optimization_problems WHERE project_name=" << escLit(conn_, projectName)
        << " ORDER BY created_at DESC";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "[MetricsCollector] queryOptimizationProblems failed: " << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return "[]";
    }

    int rows = PQntuples(res);
    std::ostringstream out;
    out << '[';
    for (int i = 0; i < rows; ++i) {
        if (i > 0) out << ',';
        auto col = [&](int c) -> std::string {
            return PQgetisnull(res, i, c) ? "" : std::string(PQgetvalue(res, i, c));
        };
        std::string thresh = col(7); if (thresh.empty()) thresh = "1.15";
        out << '{'
            << "\"problemId\":"           << jStr(col(0)) << ','
            << "\"name\":"                << jStr(col(1)) << ','
            << "\"candidateRoadIds\":"    << col(2)       << ','
            << "\"phaseCount\":"          << col(3)       << ','
            << "\"simDurationS\":"        << col(4)       << ','
            << "\"trafficMode\":"         << jStr(col(5)) << ','
            << "\"scenarioType\":"        << jStr(col(6)) << ','
            << "\"constraintThreshold\":" << thresh       << ','
            << "\"createdAt\":"           << jStr(col(8))
            << '}';
    }
    out << ']';
    PQclear(res);
    return out.str();
}

// ── queryOptimizationProblemById ──────────────────────────────────────────────

std::string MetricsCollector::queryOptimizationProblemById(const std::string& problemId) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_ || problemId.empty()) return "";

    auto jStr = [](const std::string& s) -> std::string {
        std::string r = "\"";
        for (char c : s) {
            if (c == '"')  r += "\\\"";
            else if (c == '\\') r += "\\\\";
            else r += c;
        }
        r += '"';
        return r;
    };

    std::ostringstream sql;
    sql << "SELECT problem_id, name, candidate_road_ids::TEXT, phase_count, sim_duration_s, "
        << "traffic_mode, scenario_type, constraint_threshold, "
        << "to_char(created_at AT TIME ZONE 'UTC', 'YYYY-MM-DD\"T\"HH24:MI:SS\"Z\"') "
        << "FROM optimization_problems WHERE problem_id=" << escLit(conn_, problemId);

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK || PQntuples(res) == 0) {
        PQclear(res);
        return "";
    }
    auto col = [&](int c) -> std::string {
        return PQgetisnull(res, 0, c) ? "" : std::string(PQgetvalue(res, 0, c));
    };
    std::string thresh = col(7); if (thresh.empty()) thresh = "1.15";
    std::string scen   = col(6); if (scen.empty())   scen   = "phased_construction";
    std::ostringstream out;
    out << '{'
        << "\"problemId\":"           << jStr(col(0)) << ','
        << "\"name\":"                << jStr(col(1)) << ','
        << "\"candidateRoadIds\":"    << col(2)       << ','
        << "\"phaseCount\":"          << col(3)       << ','
        << "\"simDurationS\":"        << col(4)       << ','
        << "\"trafficMode\":"         << jStr(col(5)) << ','
        << "\"scenarioType\":"        << jStr(scen)   << ','
        << "\"constraintThreshold\":" << thresh       << ','
        << "\"createdAt\":"           << jStr(col(8))
        << '}';
    PQclear(res);
    return out.str();
}

// ── queryOptimizationSessionById ──────────────────────────────────────────────

std::string MetricsCollector::queryOptimizationSessionById(const std::string& sessionId) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_ || sessionId.empty()) return "";

    auto jStr = [](const std::string& s) -> std::string {
        std::string r = "\"";
        for (char c : s) {
            if (c == '"')  r += "\\\"";
            else if (c == '\\') r += "\\\\";
            else r += c;
        }
        r += '"';
        return r;
    };

    std::ostringstream sql;
    sql << "SELECT session_id, status, algorithm, "
        << "to_char(created_at AT TIME ZONE 'UTC', 'YYYY-MM-DD\"T\"HH24:MI:SS\"Z\"'), "
        << "best_fitness, best_assignment::TEXT, best_phase_fitnesses::TEXT, "
        << "iterations_done, evals_done, baseline_danc, total_evals, "
        << "convergence_history::TEXT, candidate_road_ids::TEXT "
        << "FROM optimization_sessions WHERE session_id=" << escLit(conn_, sessionId);

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK || PQntuples(res) == 0) {
        PQclear(res);
        return "";
    }
    auto col  = [&](int c) { return PQgetisnull(res,0,c) ? "" : std::string(PQgetvalue(res,0,c)); };
    std::string asgnRaw  = col(5); if (asgnRaw.empty())  asgnRaw  = "[]";
    std::string pfRaw    = col(6); if (pfRaw.empty())    pfRaw    = "[]";
    std::string convRaw  = col(11); if (convRaw.empty()) convRaw  = "[]";
    std::string candsRaw = col(12); if (candsRaw.empty()) candsRaw = "[]";
    std::string bfStr    = col(4);

    std::ostringstream out;
    out << '{'
        << "\"sessionId\":"          << jStr(col(0))    << ','
        << "\"status\":"             << jStr(col(1))    << ','
        << "\"algorithm\":"          << jStr(col(2))    << ','
        << "\"createdAt\":"          << jStr(col(3))    << ','
        << "\"bestFitness\":"        << (bfStr.empty() ? "null" : bfStr) << ','
        << "\"bestAssignment\":"     << asgnRaw          << ','
        << "\"phaseFitnesses\":"     << pfRaw            << ','
        << "\"itersDone\":"          << col(7)           << ','
        << "\"evalsDone\":"          << col(8)           << ','
        << "\"baselineDANC\":"       << (col(9).empty() ? "null" : col(9)) << ','
        << "\"totalEvals\":"         << col(10)          << ','
        << "\"convergenceHistory\":" << convRaw          << ','
        << "\"candidateRoadIds\":"   << candsRaw         << ','
        << "\"fromDB\":"             << "true"
        << '}';
    PQclear(res);
    return out.str();
}

// ── deleteOptimizationProblem ─────────────────────────────────────────────────

void MetricsCollector::deleteOptimizationProblem(const std::string& problemId) {
    if (problemId.empty()) return;
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return;
    std::ostringstream sql;
    sql << "DELETE FROM optimization_problems WHERE problem_id=" << escLit(conn_, problemId);
    execSimple(sql.str());
    std::cout << "[MetricsCollector] OptimizationProblem deleted: " << problemId << "\n";
}

// ── createOptimizationSession ─────────────────────────────────────────────────

std::string MetricsCollector::createOptimizationSession(const std::string& problemId,
                                                          const std::string& projectName,
                                                          const std::string& algorithm,
                                                          const std::string& hyperparamsJson) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return "";

    const std::string safeParams = hyperparamsJson.empty() ? "{}" : hyperparamsJson;

    std::ostringstream sql;
    sql << "INSERT INTO optimization_sessions "
        << "(problem_id, project_name, algorithm, hyperparams) VALUES ("
        << escLit(conn_, problemId)   << ","
        << escLit(conn_, projectName) << ","
        << escLit(conn_, algorithm)   << ","
        << escLit(conn_, safeParams)  << "::jsonb"
        << ") RETURNING session_id";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK || PQntuples(res) == 0) {
        std::cerr << "[MetricsCollector] createOptimizationSession failed: " << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return "";
    }
    std::string sessionId(PQgetvalue(res, 0, 0));
    PQclear(res);
    std::cout << "[MetricsCollector] OptimizationSession created: " << sessionId << "\n";
    return sessionId;
}

// ── finalizeOptimizationSession ───────────────────────────────────────────────

void MetricsCollector::finalizeOptimizationSession(const std::string& sessionId,
                                                     double bestFitness,
                                                     const std::string& bestAssignmentJson,
                                                     const std::string& phaseFitnessJson,
                                                     int itersDone,
                                                     int evalsDone) {
    if (sessionId.empty()) return;
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return;

    const std::string safeAsgn = bestAssignmentJson.empty() ? "[]" : bestAssignmentJson;
    const std::string safePf   = phaseFitnessJson.empty()   ? "[]" : phaseFitnessJson;

    std::ostringstream sql;
    sql << "UPDATE optimization_sessions SET "
        << "status='completed', "
        << "best_fitness="          << dbl(bestFitness)             << ", "
        << "best_assignment="       << escLit(conn_, safeAsgn)      << "::jsonb, "
        << "best_phase_fitnesses="  << escLit(conn_, safePf)        << "::jsonb, "
        << "iterations_done="       << itersDone                    << ", "
        << "evals_done="            << evalsDone
        << " WHERE session_id="     << escLit(conn_, sessionId);
    execSimple(sql.str());
    std::cout << "[MetricsCollector] OptimizationSession finalized: " << sessionId
              << "  best=" << bestFitness << "\n";
}

// ── checkpointOptimizationSession ────────────────────────────────────────────

void MetricsCollector::checkpointOptimizationSession(
        const std::string& sessionId,
        double bestFitness,
        const std::string& bestAssignmentJson,
        const std::string& phaseFitnessJson,
        int itersDone,
        int evalsDone,
        double baselineDANC,
        int totalEvals,
        const std::string& convergenceHistoryJson,
        const std::string& candidateRoadIdsJson) {
    if (sessionId.empty()) return;
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return;

    const std::string safeAsgn  = bestAssignmentJson.empty()      ? "[]" : bestAssignmentJson;
    const std::string safePf    = phaseFitnessJson.empty()         ? "[]" : phaseFitnessJson;
    const std::string safeConv  = convergenceHistoryJson.empty()   ? "[]" : convergenceHistoryJson;
    const std::string safeCands = candidateRoadIdsJson.empty()     ? "[]" : candidateRoadIdsJson;

    std::ostringstream sql;
    sql << "UPDATE optimization_sessions SET "
        << "best_fitness="          << dbl(bestFitness)                  << ", "
        << "best_assignment="       << escLit(conn_, safeAsgn)           << "::jsonb, "
        << "best_phase_fitnesses="  << escLit(conn_, safePf)             << "::jsonb, "
        << "iterations_done="       << itersDone                         << ", "
        << "evals_done="            << evalsDone                         << ", "
        << "baseline_danc="         << dbl(baselineDANC)                 << ", "
        << "total_evals="           << totalEvals                        << ", "
        << "convergence_history="   << escLit(conn_, safeConv)           << "::jsonb, "
        << "candidate_road_ids="    << escLit(conn_, safeCands)          << "::jsonb"
        << " WHERE session_id="     << escLit(conn_, sessionId);
    execSimple(sql.str());
}

// ── queryOptimizationSessionsForProblem ───────────────────────────────────────

std::string MetricsCollector::queryOptimizationSessionsForProblem(
        const std::string& problemId) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_ || problemId.empty()) return "[]";

    auto jStr = [](const std::string& s) -> std::string {
        std::string r = "\"";
        for (char c : s) {
            if (c == '"')  r += "\\\"";
            else if (c == '\\') r += "\\\\";
            else r += c;
        }
        r += '"';
        return r;
    };

    std::ostringstream sql;
    sql << "SELECT session_id, status, algorithm, "
        << "to_char(created_at AT TIME ZONE 'UTC', 'YYYY-MM-DD\"T\"HH24:MI:SS\"Z\"'), "
        << "best_fitness, best_assignment::TEXT, best_phase_fitnesses::TEXT, "
        << "iterations_done, evals_done, baseline_danc, total_evals, "
        << "convergence_history::TEXT, candidate_road_ids::TEXT "
        << "FROM optimization_sessions WHERE problem_id=" << escLit(conn_, problemId)
        << " ORDER BY created_at DESC LIMIT 20";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "[MetricsCollector] queryOptimizationSessionsForProblem failed: "
                  << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return "[]";
    }

    auto col  = [&](int r, int c) { return PQgetisnull(res,r,c) ? "" : std::string(PQgetvalue(res,r,c)); };
    auto dblV = [&](int r, int c) -> double {
        auto s = col(r, c);
        return s.empty() ? 0.0 : std::stod(s);
    };

    // Column offsets: 0=session_id 1=status 2=algorithm 3=created_at
    //   4=best_fitness 5=best_assignment 6=best_phase_fitnesses
    //   7=iterations_done 8=evals_done 9=baseline_danc 10=total_evals
    //   11=convergence_history 12=candidate_road_ids

    std::ostringstream out;
    out << "[";
    int n = PQntuples(res);
    for (int i = 0; i < n; i++) {
        if (i) out << ",";
        std::string asgnRaw  = col(i,  5); if (asgnRaw.empty())  asgnRaw  = "[]";
        std::string pfRaw    = col(i,  6); if (pfRaw.empty())    pfRaw    = "[]";
        std::string convRaw  = col(i, 11); if (convRaw.empty())  convRaw  = "[]";
        std::string candsRaw = col(i, 12); if (candsRaw.empty()) candsRaw = "[]";
        double bf = col(i,4).empty() ? 0.0 : dblV(i, 4);
        out << "{"
            << "\"sessionId\":"          << jStr(col(i,0))  << ","
            << "\"status\":"             << jStr(col(i,1))  << ","
            << "\"algorithm\":"          << jStr(col(i,2))  << ","
            << "\"createdAt\":"          << jStr(col(i,3))  << ","
            << "\"bestFitness\":"        << (col(i,4).empty() ? "null" : std::to_string(bf)) << ","
            << "\"bestAssignment\":"     << asgnRaw          << ","
            << "\"phaseFitnesses\":"     << pfRaw            << ","
            << "\"itersDone\":"          << col(i,7)         << ","
            << "\"evalsDone\":"          << col(i,8)         << ","
            << "\"baselineDANC\":"       << (col(i,9).empty() ? "null" : col(i,9)) << ","
            << "\"totalEvals\":"         << col(i,10)        << ","
            << "\"convergenceHistory\":" << convRaw          << ","
            << "\"candidateRoadIds\":"   << candsRaw
            << "}";
    }
    out << "]";
    PQclear(res);
    return out.str();
}

// ── findBaselineSession ───────────────────────────────────────────────────────

std::string MetricsCollector::findBaselineSession(const std::string& projectName) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_ || projectName.empty()) return "";

    // Baseline session: completed, no closed roads (empty JSON array)
    std::ostringstream sql;
    sql << "SELECT session_id FROM analysis_sessions "
        << "WHERE project_name = " << escLit(conn_, projectName)
        << " AND status = 'completed'"
        << " AND closed_road_ids = '[]'::jsonb"
        << " ORDER BY created_at DESC LIMIT 1";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    std::string result;
    if (PQresultStatus(res) == PGRES_TUPLES_OK && PQntuples(res) > 0)
        result = PQgetvalue(res, 0, 0);
    PQclear(res);
    return result;
}

// ── queryRunsForProject ───────────────────────────────────────────────────────

std::string MetricsCollector::queryRunsForProject(const std::string& projectName) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!conn_) return "[]";

    std::ostringstream sql;
    sql << "SELECT run_id, run_type, label, status, "
        << "to_char(created_at AT TIME ZONE 'UTC', 'YYYY-MM-DD\"T\"HH24:MI:SS\"Z\"'), "
        << "COALESCE(fitness_score::TEXT, 'null'), "
        << "COALESCE(total_excess_vehicle_hours::TEXT, 'null'), "
        << "COALESCE(avg_excess_time_per_trip_s::TEXT, 'null'), "
        << "COALESCE(completion_rate::TEXT, 'null'), "
        << "COALESCE(vehicles_spawned::TEXT, 'null'), "
        << "COALESCE(vehicles_completed::TEXT, 'null'), "
        << "closed_road_ids::TEXT, "
        << "COALESCE(baseline_run_id, ''), "
        << "COALESCE(danc_score::TEXT, 'null'), "
        << "COALESCE(avg_freeflow_all_spawned_s::TEXT, 'null') "
        << "FROM simulation_runs "
        << "WHERE project_name = " << escLit(conn_, projectName)
        << " ORDER BY created_at DESC";

    PGresult* res = PQexec(conn_, sql.str().c_str());
    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "[MetricsCollector] queryRunsForProject failed: "
                  << PQerrorMessage(conn_) << "\n";
        PQclear(res);
        return "[]";
    }

    int rows = PQntuples(res);
    // Build JSON manually to avoid pulling in nlohmann here
    std::ostringstream out;
    out << '[';
    for (int i = 0; i < rows; ++i) {
        if (i > 0) out << ',';
        auto col = [&](int c) -> std::string {
            return PQgetisnull(res, i, c) ? "" : std::string(PQgetvalue(res, i, c));
        };
        // Helper: wrap string value in quotes, escaping backslash and double-quote
        auto jStr = [](const std::string& s) -> std::string {
            std::string r = "\"";
            for (char c : s) {
                if (c == '"')  r += "\\\"";
                else if (c == '\\') r += "\\\\";
                else r += c;
            }
            r += '"';
            return r;
        };
        out << '{'
            << "\"runId\":"                   << jStr(col(0))  << ','
            << "\"runType\":"                 << jStr(col(1))  << ','
            << "\"label\":"                   << jStr(col(2))  << ','
            << "\"status\":"                  << jStr(col(3))  << ','
            << "\"createdAt\":"               << jStr(col(4))  << ','
            << "\"fitnessScore\":"            << col(5)        << ','
            << "\"totalExcessVehicleHours\":" << col(6)        << ','
            << "\"avgExcessTimeSec\":"        << col(7)        << ','
            << "\"completionRate\":"          << col(8)        << ','
            << "\"vehiclesSpawned\":"         << col(9)        << ','
            << "\"vehiclesCompleted\":"       << col(10)       << ','
            << "\"closedRoadIds\":"           << col(11)       << ','
            << "\"baselineRunId\":"           << jStr(col(12)) << ','
            << "\"dancScore\":"               << col(13)       << ','
            << "\"avgFreeflowAllSpawnedS\":"  << col(14)
            << '}';
    }
    out << ']';
    PQclear(res);
    return out.str();
}
