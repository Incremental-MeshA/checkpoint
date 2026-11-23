#include "changed_edges.hpp"

/// Precompute which arc traces in graph can touch an edited cell. watched chooses the source or target as origin.
/// Use the arcs in motion_configs when supplied, otherwise use the graph's native arcs.
ChangedEdgeIndex::ChangedEdgeIndex(const Graph &graph, WatchedEnd watched, const std::vector<Config> *motion_configs)
    : graph(graph), configs(motion_configs ? *motion_configs : graph.configurations()), watched(watched), tile_size(graph.mesh && !motion_configs ? 4 : 8),
      tiles_across((graph.grid.width + tile_size - 1) / tile_size), by_offset(configs.size()) {
    bool incoming = watched == WatchedEnd::target;
    for (size_t type = 0; type < configs.size(); ++type) {
        const Config &config = configs[type];
        const auto &arcs = incoming ? config.in : config.out;
        for (int i = 0; i < static_cast<int>(arcs.size()); ++i) {
            const Arc &arc = arcs[i];
            Point source = incoming ? arc.delta : Point{}; // relative to watched endpoint
            /// Index the collision-trace offset cell, measured from the forward source, under the watched endpoint.
            auto include = [&](Point cell) {
                Point offset = Point{} - (source + cell);
                min_offset.x = std::min(min_offset.x, offset.x);
                min_offset.y = std::min(min_offset.y, offset.y);
                max_offset.x = std::max(max_offset.x, offset.x);
                max_offset.y = std::max(max_offset.y, offset.y);
                by_offset[type][{offset.x, offset.y}].push_back(i);
            };
            if (arc.trace_id >= 0) {
                for (Point cell : graph.chain_traces[arc.trace_id])
                    include(cell);
            } else if (graph.mesh && arc.trace_id == -1) {
                include({});
                include(incoming ? Point{} - arc.delta : arc.delta);
            } else {
                for (Point cell : graph.primitives[arc.primitive].trace)
                    include(cell);
            }
        }
    }
}

/// Place node in the tile containing its watched endpoint so nearby edits can find it.
void ChangedEdgeIndex::watch(Node node) {
    tiles[tile_of(graph.point(node))].insert(node);
}

/// Remove node from its spatial tile and discard the tile if its last watched endpoint was removed.
void ChangedEdgeIndex::forget(Node node) {
    auto tile = tiles.find(tile_of(graph.point(node)));
    if (tile == tiles.end())
        return;
    tile->second.erase(node);
    if (tile->second.empty())
        tiles.erase(tile);
}

/// Compare the forward footprint of arc from source to target in the old and current maps using per-cell edit stamps.
EdgeChange ChangedEdgeIndex::availability(Point source, Point target, const Arc &arc) const {
    const Grid &grid = graph.grid;
    bool was_free = true, now_free = true;
    /// Include grid position p in both footprint checks, reconstructing its previous occupancy if this batch edited it.
    auto check = [&](Point p) {
        if (!grid.contains(p)) {
            was_free = now_free = false;
        } else {
            int cell = grid.cell(p);
            bool free = !grid.occupied(cell);
            bool changed = grid.edit_version[cell] == grid.version;
            was_free &= free != changed;
            now_free &= free;
        }
    };
    if (graph.mesh && arc.trace_id == -1) {
        check(source);
        check(target);
    } else {
        const auto &trace = arc.trace_id >= 0 ? graph.chain_traces[arc.trace_id] : graph.primitives[arc.primitive].trace;
        for (Point offset : trace) {
            check(source + offset);
            if (!was_free && !now_free)
                break;
        }
    }
    if (was_free == now_free)
        return EdgeChange::unchanged;
    return now_free ? EdgeChange::opened : EdgeChange::closed;
}

/// Use the changed cells in batch to find candidate arcs. Pass each reportable change to accept
/// as a forward source node, an outgoing edge and its change in availability.
void ChangedEdgeIndex::visit(const Batch &batch,
                             const std::function<void(Node, Edge, EdgeChange)> &accept) const {
    if (tiles.empty())
        return;
    /// Inclusive grid-coordinate bounds for candidate watched endpoints.
    struct Bounds { int left, top, right, bottom; };
    /// Groups a tile's candidate endpoint bounds with the edited cells that may affect it.
    struct Region { Bounds bounds; std::vector<Point> edits; };
    std::unordered_map<int, Region> regions;
    const Grid &grid = graph.grid;
    /// Group the row-major indices in cells by tiles whose watched endpoints could have a footprint touching an edit.
    auto include_cells = [&](const std::vector<int> &cells) {
        for (int cell : cells) {
            Point p{cell % grid.width, cell / grid.width};
            Bounds box{std::max(0, p.x + min_offset.x), std::max(0, p.y + min_offset.y),
                       std::min(grid.width - 1, p.x + max_offset.x),
                       std::min(grid.height - 1, p.y + max_offset.y)};
            for (int y = box.top / tile_size; y <= box.bottom / tile_size; ++y) {
                for (int x = box.left / tile_size; x <= box.right / tile_size; ++x) {
                    int id = y * tiles_across + x;
                    if (!tiles.count(id))
                        continue;
                    auto [it, added] = regions.try_emplace(id);
                    it->second.edits.push_back(p);
                    if (added)
                        it->second.bounds = box;
                    else {
                        Bounds &b = it->second.bounds;
                        b.left = std::min(b.left, box.left);
                        b.top = std::min(b.top, box.top);
                        b.right = std::max(b.right, box.right);
                        b.bottom = std::max(b.bottom, box.bottom);
                    }
                }
            }
        }
    };
    bool incoming = watched == WatchedEnd::target;
    if (incoming)
        include_cells(batch.block);
    include_cells(batch.clear);

    std::vector<Node> last_checked; // one stamp per incident arc index
    for (const auto &[id, region] : regions) {
        const Bounds &box = region.bounds;
        for (Node node : tiles.at(id)) {
            Point p = graph.point(node);
            if (p.x < box.left || p.x > box.right || p.y < box.top || p.y > box.bottom)
                continue;
            int type = graph.type(node);
            const Config &config = configs[type];
            const auto &arcs = incoming ? config.in : config.out;
            /// Test the incident arc at this watched endpoint and report it if its availability changed in a tracked direction.
            auto examine = [&](const Arc &arc) {
                Point other = p + arc.delta;
                Point source = incoming ? other : p;
                Point target = incoming ? p : other;
                EdgeChange change = availability(source, target, arc);
                if (change == EdgeChange::unchanged || (!incoming && change == EdgeChange::closed))
                    return;
                Node from = incoming ? graph.at(source, arc.type) : node;
                Node to = incoming ? node : graph.at(target, arc.type);
                accept(from, {to, arc.cost, arc.primitive, arc.slot, arc.trace_id}, change);
            };
            // In a dense batch each incident arc is cheaper to visit once.
            if (arcs.size() <= region.edits.size()) {
                for (const Arc &arc : arcs)
                    examine(arc);
                continue;
            }
            // Sparse batch: only traces actually passing through an edited cell.
            if (last_checked.size() < arcs.size())
                last_checked.resize(arcs.size(), -1);
            for (Point edit : region.edits) {
                Point offset = p - edit;
                auto matches = by_offset[type].find({offset.x, offset.y});
                if (matches == by_offset[type].end())
                    continue;
                for (int i : matches->second) {
                    if (last_checked[i] == node)
                        continue;
                    last_checked[i] = node;
                    examine(arcs[i]);
                }
            }
        }
    }
}
