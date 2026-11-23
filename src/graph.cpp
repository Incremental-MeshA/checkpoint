#include "graph.hpp"
#include "../algorithms/astar/paper/KC_structs.hpp"
#include <fstream>
#include <sstream>

/// Read dimensions and occupancy rows from filename. The octile symbols '.', 'G' and 'S' denote free cells.
Grid::Grid(const std::string &filename) {
    std::ifstream f(filename);
    std::string label, value;
    if (!(f >> label >> value) || label != "type" || value != "octile" || !(f >> label >> height) ||
        label != "height" || !(f >> label >> width) || label != "width" || !(f >> label) ||
        label != "map" || width <= 0 || height <= 0)
        throw std::runtime_error("Invalid map header: " + filename);
    for (int y = 0; y < height; ++y) {
        if (!(f >> value) || int(value.size()) != width)
            throw std::runtime_error("Invalid map row");
        for (char c : value)
            blocked.push_back(c != '.' && c != 'G' && c != 'S');
    }
}

/// Parse the control set in filename. Validate headings and traces and bound each length below by its endpoint displacement.
std::vector<Primitive> load_primitives(const std::string &filename) {
    std::ifstream f(filename);
    std::string line;
    std::vector<Primitive> all;
    Primitive p{};
    while (std::getline(f, line)) {
        auto fields = std::istringstream(line.substr(line.find(':') + 1));
        if (line.find("start heading") == 0)
            fields >> p.start;
        else if (line.find("goal state") == 0)
            fields >> p.delta.y >> p.delta.x >> p.end;
        else if (line.find("length is") == 0) {
            double length;
            if (!(fields >> length) || !std::isfinite(length) || length < 1)
                throw std::runtime_error("Invalid length");
            p.length = Cost::length(length);
        } else if (line.find("prim ID") == 0)
            fields >> p.id;
        else if (line.find("collision is") == 0) {
            while (std::getline(f, line) && line.find("---") != 0) {
                Point offset;
                if (!(std::istringstream(line) >> offset.y >> offset.x))
                    throw std::runtime_error("Invalid trace");
                p.trace.push_back(offset);
            }
        } else if (line.find("prim end") == 0) {
            if (p.trace.size() < 2 || !(p.trace.front() == Point{}) || !(p.trace.back() == p.delta) ||
                p.start < 0 || p.start >= 16 || p.end < 0 || p.end >= 16 || p.length.whole < 1)
                throw std::runtime_error("Invalid primitive");
            p.length = std::max(p.length, Cost::length(std::hypot(p.delta.x, p.delta.y)));
            all.push_back(p);
            p = {};
        }
    }
    if (all.empty())
        throw std::runtime_error("Missing primitives: " + filename);
    return all;
}

/// Record the transition from configuration from to configuration to in both adjacency lists. delta is the forward
/// displacement. prim is the completed primitive index, or -1 for a prefix that accrues only a step.
void Graph::add_arc(int from, int to, Point delta, int prim) {
    Cost cost = prim < 0 ? Cost{} : primitives[prim].length;
    cost.steps = 1;
    int slot = static_cast<int>(configs[from].out.size());
    configs[from].out.push_back({to, delta, prim, cost, slot});
    configs[to].in.push_back({from, Point{} - delta, prim, cost, slot});
}

/// Build the graph over map with the motions in controls. use_mesh selects shared prefixes instead of whole-motion arcs.
/// When use_mesh is true, table supplies precomputed transitions if available.
Graph::Graph(Grid map, std::vector<Primitive> controls, bool use_mesh, const MeshInfo *table)
    : mesh(use_mesh), configs(16), grid(std::move(map)), primitives(std::move(controls)) {
    grid.edit_version.resize(grid.blocked.size());
    // A prefix represents the set of (primitive, step) pairs with the same
    // swept cells. Terminating and continuing arcs may enter the same cell.
    std::map<std::tuple<int, int, int>, int> prefixes;
    for (int i = 0; i < int(primitives.size()); ++i) {
        const auto &p = primitives[i];
        if (!mesh) {
            add_arc(p.start, p.end, p.delta, i);
            continue;
        }
        if (table) continue; // The published table already contains every mesh edge.
        int parent = p.start;
        for (size_t k = 1; k < p.trace.size(); ++k) {
            Point step = p.trace[k] - p.trace[k - 1];
            if (k + 1 == p.trace.size()) {
                add_arc(parent, p.end, step, i);
                break;
            }
            auto key = std::make_tuple(parent, step.x, step.y);
            auto [it, added] = prefixes.emplace(key, int(configs.size()));
            if (added) {
                configs.emplace_back();
                configs.back().offset = p.trace[k];
                add_arc(parent, it->second, step, -1);
            }
            parent = it->second;
            configs[parent].completions.push_back(i);
        }
    }
    if (table && mesh) import_mesh(*table);
    if (mesh) {
        std::vector<int> queue;
        for (int h = 0; h < 16; ++h) queue.push_back(h);
        for (size_t k = 0; k < queue.size(); ++k) {
            const int parent = queue[k];
            for (const Arc &arc : configs[parent].out) {
                if (arc.primitive >= 0 || configs[arc.type].depth) continue;
                configs[arc.type].depth = configs[parent].depth + 1;
                queue.push_back(arc.type);
            }
        }
    }
}

// The incremental planners address boundary poses by heading (0..15).
// Remap the paper's configuration numbers, retaining its arcs and Finals.
/// Import the transitions and primitive completions from table, matching its primitive IDs to the loaded controls.
void Graph::import_mesh(const MeshInfo &table) {
    std::map<int,int> index;
    for (int i = 0; i < int(primitives.size()); ++i)
        index.emplace(primitives[i].id, i);
    std::vector<int> types(table.successors.size(), -1);
    for (int h = 0; h < 16; ++h)
        types.at(table.initial_config_by_theta.at(h)) = h;
    for (size_t id = 0; id < types.size(); ++id) {
        const auto &finals = table.primitives_in_config[id];
        if (types[id] >= 0 || finals.empty()) continue;
        types[id] = int(configs.size());
        configs.emplace_back();
        auto &cfg = configs.back();
        for (auto [heading,dy,dx,k,prim_id] : finals) {
            int i = index.at(prim_id);
            const auto &motion = primitives[i];
            Point offset = motion.delta - Point{dx,dy};
            if (motion.end != heading || !(motion.trace.at(k) == offset) ||
                (!cfg.completions.empty() && !(cfg.offset == offset)))
                throw std::runtime_error("Mesh Finals do not match the control set");
            cfg.offset = offset;
            cfg.completions.push_back(i);
        }
    }
    for (size_t id = 0; id < types.size(); ++id) {
        for (auto [dy,dx,to,prim_id] : table.successors[id]) {
            if (types[id] < 0 || types.at(to) < 0)
                throw std::runtime_error("Unknown mesh configuration");
            add_arc(types[id], types[to], {dx,dy}, prim_id < 0 ? -1 : index.at(prim_id));
        }
    }
}

/// Compute the Euclidean distance in cell units between positions a and b.
Cost Graph::distance(Point a, Point b) const {
    return Cost::length(std::hypot(a.x - b.x, a.y - b.y));
}
/// Estimate the cost from u to goal. At a mesh prefix, minimize over the motions that can complete that prefix.
Cost Graph::forward(Node u, Node goal) const {
    Point p = point(u), target = point(goal);
    if (type(u) < 16)
        return distance(p, target);
    const auto &c = configs[type(u)];
    Cost best = Cost::inf();
    for (int i : c.completions) {
        const auto &prim = primitives[i];
        best = std::min(best, prim.length + distance(p - c.offset + prim.delta, target));
    }
    return best;
}
/// Estimate the cost from start to the motion origin associated with u. For a full pose, that origin is its own cell.
Cost Graph::reverse(Node start, Node u) const {
    return distance(point(start), point(u) - configs[type(u)].offset);
}
/// Follow the completed primitives in path from start, checking their headings, footprints and arrival at goal.
/// Return indices into the graph's primitive array.
std::vector<int> Graph::decode(Node start, Node goal, const std::vector<Edge> &path) const {
    std::vector<int> route;
    Node pose = start;
    for (auto edge : path) {
        if (edge.primitive < 0)
            continue;
        const auto &p = primitives.at(edge.primitive);
        Point origin = point(pose);
        if (type(pose) != p.start || at(origin + p.delta, p.end) != edge.node)
            throw std::logic_error("Invalid primitive endpoints");
        for (Point offset : p.trace) {
            Point q = origin + offset;
            if (!grid.contains(q) || grid.blocked[grid.cell(q)])
                throw std::logic_error("Collision in result");
        }
        route.push_back(edge.primitive);
        pose = edge.node;
    }
    if (pose != goal)
        throw std::logic_error("Incomplete path");
    return route;
}
/// Check whether distinct endpoints have a usable departure from start and arrival at goal. Return true if either is missing.
bool Graph::isolated_endpoint(Node start, Node goal) const {
    if (start == goal)
        return false;
    bool can_leave = false, can_arrive = false;
    /// Return whether every cell swept by primitive p is free when its trace starts at origin.
    auto clear = [&](Point origin, const Primitive &p) {
        for (Point delta : p.trace) {
            Point cell = origin + delta;
            if (!grid.contains(cell) || grid.occupied(grid.cell(cell)))
                return false;
        }
        return true;
    };
    for (const auto &p : primitives) {
        if (!can_leave && p.start == type(start))
            can_leave = clear(point(start), p);
        if (!can_arrive && p.end == type(goal))
            can_arrive = clear(point(goal) - p.delta, p);
        if (can_leave && can_arrive)
            return false;
    }
    return true;
}

// Both directions use the same forward collision trace, anchored at its source.
/// Collect adjacent nodes of u in result, replacing earlier entries. backwards selects incoming arcs and reverses
/// which endpoint is returned. Collision checks always follow the arc from its forward source.
void Graph::neighbors(Node u, bool backwards, std::vector<Edge> &result) const {
    result.clear();
    Point p = point(u);
    if (grid.occupied(grid.cell(p)))
        return;
    const auto &cfg = configs[type(u)];
    const auto &arcs = backwards ? cfg.in : cfg.out;
    auto *checks = grid.checks;
    bool sample = checks && checks->sample_next_scan && !arcs.empty();
    if (sample)
        checks->sample_next_scan = false;
    result.reserve(arcs.size());
    for (const Arc &a : arcs) {
        Point q = p + a.delta;
        Point source = backwards ? q : p;
        Point target = backwards ? p : q;
        bool clear = traversable(source, target, a.primitive, a.trace_id);
        if (sample) {
            CellChecks::Candidate edge{source, target,
                a.primitive < 0 ? -1 : primitives[a.primitive].id, clear, checks->counts, source, {}};
            if (mesh) {
                edge.origin = source - configs[backwards ? a.type : type(u)].offset;
                if (a.primitive >= 0)
                    edge.motion_ids.push_back(edge.primitive);
                else
                    for (int i : configs[backwards ? type(u) : a.type].completions)
                        edge.motion_ids.push_back(primitives[i].id);
            }
            auto &sample_edges = checks->samples.back().edges;
            if (a.trace_id < 0) {
                sample_edges.push_back(std::move(edge));
            } else {
                // Playback draws adjacent swept cells, including inside a macro edge.
                // Use the grid directly here so replay instrumentation adds no reads.
                const auto &trace = chain_traces[a.trace_id];
                for (size_t k = 1; k < trace.size(); ++k) {
                    edge.source = source + trace[k-1];
                    edge.target = source + trace[k];
                    edge.clear = grid.contains(edge.source) && grid.contains(edge.target) &&
                                 !grid.blocked[grid.cell(edge.source)] && !grid.blocked[grid.cell(edge.target)];
                    sample_edges.push_back(edge);
                    if (!edge.clear) break;
                }
            }
        }
        if (!clear)
            continue;
        result.push_back({at(q, a.type), a.cost, a.primitive, a.slot, a.trace_id});
    }
}

/// Materialize the outgoing transition at slot for source using its configuration and cell coordinates.
Edge Graph::edge_from(Node source, int slot) const {
    const Arc &arc = configs[type(source)].out.at(slot);
    return {at(point(source) + arc.delta, arc.type), arc.cost, arc.primitive, slot, arc.trace_id};
}

/// Check whether the motion from source to target stays on free cells. trace_id selects a contracted footprint,
/// a native mesh step or a whole primitive. primitive identifies the motion for whole-trace checks.
bool Graph::traversable(Point source, Point target, int primitive, int trace_id) const {
    if (mesh && trace_id == -1)
        return grid.contains(source) && grid.contains(target) &&
               !grid.occupied(grid.cell(source)) && !grid.occupied(grid.cell(target));
    // The lattice trace already includes both endpoints.
    const auto &trace = trace_id >= 0 ? chain_traces[trace_id] : primitives[primitive].trace;
    for (Point offset : trace) {
        Point cell = source + offset;
        if (!grid.contains(cell) || grid.occupied(grid.cell(cell)))
            return false;
    }
    return true;
}

// A pose or a fork remains a search vertex. Interior vertices have one parent;
// an unbranched segment can only be entered at its start and left at its end.
/// Replace unbranched chains of mesh-prefix arcs with single arcs that retain the original cost, step count and footprint.
void Graph::contract_chains() {
    if (!mesh) throw std::invalid_argument("Chain contraction requires mesh");
    const auto original = configs;
    for (Config &config : configs) {
        config.out.clear();
        config.in.clear();
    }
    for (int from = 0; from < static_cast<int>(configs.size()); ++from) {
        if (from >= 16 && original[from].in.size() == 1 && original[from].out.size() == 1)
            continue;
        for (Arc arc : original[from].out) {
            std::vector<Point> trace{Point{}, arc.delta};
            while (arc.type >= 16 && original[arc.type].in.size() == 1 && original[arc.type].out.size() == 1) {
                const Arc &next = original[arc.type].out.front();
                arc.cost = arc.cost + next.cost; // includes each original step
                arc.delta = arc.delta + next.delta;
                arc.type = next.type;
                arc.primitive = next.primitive;
                trace.push_back(arc.delta);
            }
            if (trace.size() > 2) {
                arc.trace_id = static_cast<int>(chain_traces.size());
                chain_traces.push_back(std::move(trace));
            }
            arc.slot = static_cast<int>(configs[from].out.size());
            configs[from].out.push_back(arc);
            configs[arc.type].in.push_back({from, Point{} - arc.delta, arc.primitive,
                                            arc.cost, arc.slot, arc.trace_id});
        }
    }
}
