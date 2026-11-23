#pragma once
#include "graph.hpp"
#include <chrono>
#include <array>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>

/// Counts the operations performed on one search or repair queue.
struct QueueOps {
    int64_t insert = 0, pop = 0, change = 0, erase = 0, peek = 0;
};
/// Accumulates timings, expansion counts and optional trace data for a single planning query.
struct Metrics {
    /// Distinguishes ordinary search expansions, repair expansions and initial repair seeds in the trace.
    enum Phase { Search, Repair, Seed };
    bool tracing = false;
    /// Stores the encoded cell, state kind and phase, together with the occupancy counts before the event.
    struct TraceEvent {
        int64_t code;
        CellCounts before;
    };
    std::vector<TraceEvent> trace;
    // One representative primitive or pose per logical phase keeps walkthroughs bounded.
    struct PhaseSample {
        int count = 0, x = 0, y = 0, heading = 0, primitive = -1;
        Cost before{}, after{};
        void observe(const Graph &graph, Node s, int id, Cost old_value, Cost new_value) {
            if (count++ != 0) return;
            Point p = graph.point(s);
            x = p.x; y = p.y; heading = graph.type(s); primitive = id;
            before = old_value; after = new_value;
        }
    };
    bool walkthrough = false, astar_started = false;
    std::vector<int64_t> learned_poses; // row-major cell * 16 + heading
    PhaseSample opened, lowered, raised;
    int64_t reused_edges = 0;
    /// When tracing is enabled, locate node in graph and record its phase with the current occupancy counters.
    void record(const Graph &graph, Node node, Phase phase) {
        if (tracing) {
            trace.push_back({int64_t(graph.cell(node)) * 8 +
                             (graph.type(node) >= 16 ? 4 : 0) + phase, graph.grid.checks->counts});
            graph.grid.checks->begin_event(trace.size());
        }
    }
    using Clock = std::chrono::steady_clock;
    /// Return the elapsed wall-clock duration in milliseconds since the steady-clock timestamp started.
    static double since(Clock::time_point started) {
        return std::chrono::duration<double, std::milli>(Clock::now() - started).count();
    }
    double changed_edges_ms = 0, path_ms = 0;
    QueueOps open, repair;
    int64_t expanded = 0, repair_expanded = 0;
    int64_t batch_seeds = 0; // Вершины, с которых пакет запускает обновление.
    std::unordered_set<int> cells, repair_cells;
    /// Count a search expansion of node u, track its cell in graph and record the event if tracing is enabled.
    void expand(const Graph &graph, Node u) {
        ++expanded;
        cells.insert(graph.cell(u));
        record(graph, u, Search);
    }
};
using Key = std::pair<Cost, Cost>;
/// Lists row-major cells to block or clear together for one planning query.
struct Batch {
    std::vector<int> block, clear;
    /// Validate the requested occupancy transitions and apply them to grid, marking the edited cells with the batch version.
    void apply(Grid &grid);
};
/// Reports whether the goal was reached by the search and holds the ordered edges of a successful path.
struct Result {
    bool found = false;
    std::vector<Edge> path;
};
/// Defines the interface for repeated path queries on one graph with a fixed goal.
class Planner {
protected:
    Graph &graph;
    Node goal;
    std::vector<Edge> successors, predecessors;

public:
    /// Bind the planner to graph g and fix target as the goal node for subsequent queries.
    Planner(Graph &g, Node target) : graph(g), goal(target) {}
    /// Allow derived planners to release their resources when destroyed through the base interface.
    virtual ~Planner() = default;
    /// Find a path from start to the fixed goal using the map edits in the already-applied batch.
    /// Write timing, search counters and any requested trace into metrics.
    virtual Result plan(Node start, const Batch &batch, Metrics &metrics) = 0;
};
/// Construct a planner that borrows graph and answers successive queries for the fixed goal node.
using Factory = std::unique_ptr<Planner>(Graph &graph, Node goal);
/// Create a D* Lite planner that searches backward from goal on graph.
Factory dstar_lite;
/// Create an MPGAA* planner that reuses paths and heuristic estimates toward goal on graph.
Factory mpgaa;
/// Contract unbranched mesh chains in graph, then create a D* Lite planner for goal.
Factory dstar_lite_chain;
/// Contract unbranched mesh chains in graph, then create an MPGAA* planner for goal.
Factory mpgaa_chain;
/// Construct the reference A* planner for graph and the fixed goal node. On a mesh, use mesh_table
/// if supplied. Otherwise derive the mesh configurations from graph.
std::unique_ptr<Planner> paper_astar(Graph &graph, Node goal, const MeshInfo *mesh_table);
