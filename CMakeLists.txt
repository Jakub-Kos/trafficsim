cmake_minimum_required(VERSION 3.27)
project(TrafficSimulation)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

include(FetchContent)

# nlohmann/json (header-only)
FetchContent_Declare(nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.3
        GIT_SHALLOW ON
        EXCLUDE_FROM_ALL
        SYSTEM)
FetchContent_MakeAvailable(nlohmann_json)

# cpp-httplib (header-only)
FetchContent_Declare(httplib
        GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
        GIT_TAG v0.15.3
        GIT_SHALLOW ON
        EXCLUDE_FROM_ALL
        SYSTEM)
FetchContent_MakeAvailable(httplib)

# IXWebSocket (WebSocket server/client)
# Notes:
# - USE_TLS OFF keeps things simple (no OpenSSL needed). Flip to ON later if you want WSS.
set(USE_TLS OFF CACHE BOOL "Build IXWebSocket with TLS")
FetchContent_Declare(ixwebsocket
        GIT_REPOSITORY https://github.com/machinezone/IXWebSocket.git
        GIT_TAG v11.4.6
        GIT_SHALLOW ON
        EXCLUDE_FROM_ALL
        SYSTEM)
FetchContent_MakeAvailable(ixwebsocket)

FetchContent_Declare(tinyxml2
        GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
        GIT_TAG 10.0.0
        GIT_SHALLOW ON
        EXCLUDE_FROM_ALL
        SYSTEM
)
FetchContent_MakeAvailable(tinyxml2)

find_package(Threads REQUIRED)
find_package(PostgreSQL REQUIRED)

find_library(TBB_LIB NAMES tbb PATHS /usr/lib64 /usr/local/lib64 NO_DEFAULT_PATH)
if(NOT TBB_LIB)
    foreach(_tbbver 12 2 "")
        set(_tbbcandidate "/usr/lib64/libtbb.so.${_tbbver}")
        if(EXISTS "${_tbbcandidate}")
            set(TBB_LIB "${_tbbcandidate}")
            break()
        endif()
    endforeach()
endif()
if(NOT TBB_LIB)
    message(WARNING "TBB not found — parallel agent update will be disabled.")
    add_compile_definitions(TRAFFIC_NO_PARALLEL)
else()
    message(STATUS "TBB found: ${TBB_LIB}")
endif()

# Shared engine sources (everything except main.cpp and server-only files)
set(ENGINE_SOURCES
        src/src/SimulationEngine.cpp
        src/src/SimulationWorld.cpp
        src/src/SimulationAgent.cpp
        src/src/SimulationAgentQueue.cpp
        src/src/SimulationIntersection.cpp
        src/src/SimulationIntersectionController.cpp
        src/src/SimulationLane.cpp
        src/src/SimulationRoad.cpp
        src/src/InternalRoutePlanner.cpp
        src/src/InternalSpawnSystem.cpp
        src/src/AgentControllers.cpp
        src/src/OSMMapLoader.cpp
        src/src/ParkingSystem.cpp
        src/src/MetricsCollector.cpp
)

# Throughput benchmark
add_executable(bench_throughput
        src/src/bench_throughput.cpp
        ${ENGINE_SOURCES}
)
target_include_directories(bench_throughput PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include
        ${PostgreSQL_INCLUDE_DIRS}
)
target_link_libraries(bench_throughput PRIVATE
        nlohmann_json::nlohmann_json
        tinyxml2
        Threads::Threads
        ${PostgreSQL_LIBRARIES}
        $<$<BOOL:${TBB_LIB}>:${TBB_LIB}>
)

# App sources
add_executable(${PROJECT_NAME}
        src/src/main.cpp
        ${ENGINE_SOURCES}
        src/src/OptimizerAlgorithms.cpp
        src/src/OptimizerRunner.cpp
        src/src/SensitivityRunner.cpp
)

# Includes for your headers + third-party headers
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include
        ${httplib_SOURCE_DIR}
        ${PostgreSQL_INCLUDE_DIRS}
)

# Link third-party libs.
# ixwebsocket target name differs depending on how it’s provided; handle both.
if (TARGET ixwebsocket::ixwebsocket)
    set(IXWS_TARGET ixwebsocket::ixwebsocket)
elseif (TARGET ixwebsocket)
    set(IXWS_TARGET ixwebsocket)
else ()
    message(FATAL_ERROR "IXWebSocket target not found after FetchContent.")
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE
        nlohmann_json::nlohmann_json
        tinyxml2
        Threads::Threads
        ${IXWS_TARGET}
        ${PostgreSQL_LIBRARIES}
        $<$<BOOL:${TBB_LIB}>:${TBB_LIB}>
)
