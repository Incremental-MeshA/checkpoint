#include "changed_edges.hpp"

ChangedEdgeIndex::ChangedEdgeIndex(const Graph &graph, WatchedEnd watched)
    : graph(graph), watched(watched), tile_size(graph.mesh ? 4 : 8),
      tiles_across((graph.grid.width + tile_size - 1) / tile_size), by_offset(graph.configs.size()) {
    bool incoming = watched == WatchedEnd::target;
    for (size_t type = 0; type < graph.configs.size(); ++type) {
        const Config &config = graph.configs[type];
        const auto &arcs = incoming ? config.in : config.out;
        for (int i = 0; i < static_cast<int>(arcs.size()); ++i) {
            const Arc &arc = arcs[i];
            Point source = incoming ? arc.delta : Point{}; // relative to watched endpoint
            auto include = [&](Point cell) {
                Point offset = Point{} - (source + cell);
                min_offset.x = std::min(min_offset.x, offset.x);
                min_offset.y = std::min(min_offset.y, offset.y);
                max_offset.x = std::max(max_offset.x, offset.x);
                max_offset.y = std::max(max_offset.y, offset.y);
                by_offset[type][{offset.x, offset.y}].push_back(i);
            };
            if (graph.mesh) {
                include({});
                include(incoming ? Point{} - arc.delta : arc.delta);
            } else {
                for (Point cell : graph.primitives[arc.primitive].trace)
                    include(cell);
            }
        }
    }
}

void ChangedEdgeIndex::watch(Node node) {
    tiles[tile_of(graph.point(node))].insert(node);
}

void ChangedEdgeIndex::forget(Node node) {
    auto tile = tiles.find(tile_of(graph.point(node)));
    if (tile == tiles.end())
        return;
    tile->second.erase(node);
    if (tile->second.empty())
        tiles.erase(tile);
}

EdgeChange ChangedEdgeIndex::availability(Point source, Point target, int primitive) const {
    const Grid &grid = graph.grid;
    bool was_free = true, now_free = true;
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
    if (graph.mesh) {
        check(source);
        check(target);
    } else {
        for (Point offset : graph.primitives[primitive].trace) {
            check(source + offset);
            if (!was_free && !now_free)
                break;
        }
    }
    if (was_free == now_free)
        return EdgeChange::unchanged;
    return now_free ? EdgeChange::opened : EdgeChange::closed;
}

void ChangedEdgeIndex::visit(const Batch &batch,
                             const std::function<void(Node, Edge, EdgeChange)> &accept) const {
    if (tiles.empty())
        return;
    struct Bounds { int left, top, right, bottom; };
    struct Region { Bounds bounds; std::vector<Point> edits; };
    std::unordered_map<int, Region> regions;
    const Grid &grid = graph.grid;
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
            const Config &config = graph.configs[type];
            const auto &arcs = incoming ? config.in : config.out;
            auto examine = [&](const Arc &arc) {
                Point other = p + arc.delta;
                Point source = incoming ? other : p;
                Point target = incoming ? p : other;
                EdgeChange change = availability(source, target, arc.primitive);
                if (change == EdgeChange::unchanged || (!incoming && change == EdgeChange::closed))
                    return;
                Node from = incoming ? graph.at(source, arc.type) : node;
                Node to = incoming ? node : graph.at(target, arc.type);
                accept(from, {to, arc.cost, arc.primitive, arc.slot}, change);
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
