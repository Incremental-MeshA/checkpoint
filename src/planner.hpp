#pragma once
#include "graph.hpp"
#include <chrono>
#include <array>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>

struct QueueOps {
    int64_t insert = 0, pop = 0, change = 0, erase = 0, peek = 0;
};
struct Metrics {
    enum Phase { Search, Repair, Seed };
    bool tracing = false;
    struct TraceEvent {
        int64_t code;
        CellCounts before;
    };
    std::vector<TraceEvent> trace;
    int64_t reused_edges = 0;
    void record(const Graph &graph, Node node, Phase phase) {
        if (tracing) {
            trace.push_back({int64_t(graph.cell(node)) * 8 +
                             (graph.type(node) >= 16 ? 4 : 0) + phase, graph.grid.checks->counts});
            graph.grid.checks->begin_event(trace.size());
        }
    }
    using Clock = std::chrono::steady_clock;
    static double since(Clock::time_point started) {
        return std::chrono::duration<double, std::milli>(Clock::now() - started).count();
    }
    double changed_edges_ms = 0, path_ms = 0;
    QueueOps open, repair;
    int64_t expanded = 0, repair_expanded = 0;
    int64_t batch_seeds = 0; // Вершины, с которых пакет запускает обновление.
    std::unordered_set<int> cells, repair_cells;
    void expand(const Graph &graph, Node u) {
        ++expanded;
        cells.insert(graph.cell(u));
        record(graph, u, Search);
    }
};
using Key = std::pair<Cost, Cost>;
class Open {
    using Item = std::pair<Key, Node>;
    std::set<Item> order;
    std::unordered_map<Node, Key> keys;

public:
    bool empty() const { return order.empty(); }
    Key top(QueueOps &m) const {
        ++m.peek;
        return empty() ? Key{Cost::inf(), Cost::inf()} : order.begin()->first;
    }
    void put(Node u, Key key, QueueOps &m) {
        auto [it, added] = keys.emplace(u, key);
        if (added)
            ++m.insert;
        else {
            if (it->second == key)
                return;
            ++m.change;
            order.erase({it->second, u});
            it->second = key;
        }
        order.insert({key, u});
    }
    void erase(Node u, QueueOps &m) {
        auto it = keys.find(u);
        if (it == keys.end())
            return;
        order.erase({it->second, u});
        keys.erase(it);
        ++m.erase;
    }
    Item pop(QueueOps &m) {
        Item item = *order.begin();
        order.erase(order.begin());
        keys.erase(item.second);
        ++m.pop;
        return item;
    }
};
struct Batch {
    std::vector<int> block, clear;
    void apply(Grid &grid);
};
struct Result {
    bool found = false;
    std::vector<Edge> path;
};
class Planner {
protected:
    Graph &graph;
    Node goal;
    std::vector<Edge> successors, predecessors;

public:
    Planner(Graph &g, Node target) : graph(g), goal(target) {}
    virtual ~Planner() = default;
    virtual Result plan(Node start, const Batch &batch, Metrics &metrics) = 0;
};
using Factory = std::unique_ptr<Planner>(Graph &, Node);
Factory astar, dstar_lite, mpgaa;
