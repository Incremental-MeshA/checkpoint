#pragma once
#include "planner.hpp"
#include <functional>

enum class WatchedEnd { source, target };
enum class EdgeChange { unchanged, opened, closed };

// Each watched endpoint belongs to one tile. A batch visits each candidate
// endpoint once, even when several edited cells intersect the same primitive.
class ChangedEdgeIndex {
    const Graph &graph;
    WatchedEnd watched;
    int tile_size, tiles_across;
    Point min_offset{}, max_offset{}; // from edited cell to watched endpoint
    using IncidentArcs = std::map<std::pair<int, int>, std::vector<int>>;
    std::vector<IncidentArcs> by_offset;
    std::unordered_map<int, std::unordered_set<Node>> tiles;

    int tile_of(Point p) const { return p.y / tile_size * tiles_across + p.x / tile_size; }
    EdgeChange availability(Point source, Point target, const Arc &arc) const;

public:
    ChangedEdgeIndex(const Graph &graph, WatchedEnd watched);
    void watch(Node node);
    void forget(Node node);
    void visit(const Batch &batch, const std::function<void(Node, Edge, EdgeChange)> &accept) const;
};
