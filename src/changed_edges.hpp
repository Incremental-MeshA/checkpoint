#pragma once
#include "planner.hpp"
#include <functional>

/// Selects the end of each forward arc registered in the index.
enum class WatchedEnd { source, target };
/// Describes the change in traversability of an entire arc across one batch of map edits.
enum class EdgeChange { unchanged, opened, closed };

// Each watched endpoint belongs to one tile. A batch visits each candidate
// endpoint once, even when several edited cells intersect the same primitive.
/// Finds changed transitions incident to watched nodes using spatial tiles and collision-trace offsets.
class ChangedEdgeIndex {
    const Graph &graph;
    const std::vector<Config> &configs;
    WatchedEnd watched;
    int tile_size, tiles_across;
    Point min_offset{}, max_offset{}; // from edited cell to watched endpoint
    using IncidentArcs = std::map<std::pair<int, int>, std::vector<int>>;
    std::vector<IncidentArcs> by_offset;
    std::unordered_map<int, std::unordered_set<Node>> tiles;

    /// Return the row-major tile number containing the grid coordinates p.
    int tile_of(Point p) const { return p.y / tile_size * tiles_across + p.x / tile_size; }
    /// Compare the traversability of arc from source to target before and after the most recent batch.
    EdgeChange availability(Point source, Point target, const Arc &arc) const;

public:
    /// Index arc footprints in graph around the endpoint selected by watched. When supplied, motion_configs
    /// replaces the graph's configuration table (for example, with full-motion arcs). Both inputs are borrowed.
    ChangedEdgeIndex(const Graph &graph, WatchedEnd watched, const std::vector<Config> *motion_configs = nullptr);
    /// Register node as an endpoint whose incident arcs should be checked after map edits.
    void watch(Node node);
    /// Stop watching node and release its tile entry if the tile has no other watched endpoints.
    void forget(Node node);
    /// Inspect the edits in the already-applied batch. Call accept with the forward source, outgoing edge and change.
    /// Source watches report openings. Target watches report both openings and closures.
    void visit(const Batch &batch, const std::function<void(Node, Edge, EdgeChange)> &accept) const;
};
