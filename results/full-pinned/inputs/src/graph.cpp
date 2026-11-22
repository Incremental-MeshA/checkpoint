#include "graph.hpp"
#include <fstream>
#include <sstream>

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
            all.push_back(p);
            p = {};
        }
    }
    if (all.empty())
        throw std::runtime_error("Missing primitives: " + filename);
    return all;
}

void Graph::add_arc(int from, int to, Point delta, int prim) {
    Cost cost = prim < 0 ? Cost{} : primitives[prim].length;
    cost.steps = 1;
    int slot = static_cast<int>(configs[from].out.size());
    configs[from].out.push_back({to, delta, prim, cost, slot});
    configs[to].in.push_back({from, Point{} - delta, prim, cost, slot});
}

Graph::Graph(Grid map, std::vector<Primitive> controls, bool use_mesh)
    : mesh(use_mesh), configs(16), grid(std::move(map)), primitives(std::move(controls)) {
    grid.edit_version.resize(grid.blocked.size());
    // A prefix represents the set of (primitive, step) pairs with the same
    // swept cells. Terminating and continuing arcs may enter the same cell.
    std::map<std::tuple<int, int, int>, int> prefixes;
    for (int i = 0; i < int(primitives.size()); ++i) {
        const auto &p = primitives[i];
        double displacement = std::hypot(p.delta.x, p.delta.y);
        if (displacement > 0)
            scale = std::min(scale, p.length.value() / displacement);
        if (!mesh) {
            add_arc(p.start, p.end, p.delta, i);
            continue;
        }
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
    scale *= 1.0 - 1e-12; // protect the lower bound from hypot rounding
}

Cost Graph::distance(Point a, Point b) const {
    return Cost::length(scale * std::hypot(a.x - b.x, a.y - b.y));
}
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
Cost Graph::reverse(Node start, Node u) const {
    return distance(point(start), point(u) - configs[type(u)].offset);
}
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
bool Graph::isolated_endpoint(Node start, Node goal) const {
    if (start == goal)
        return false;
    bool can_leave = false, can_arrive = false;
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
void Graph::neighbors(Node u, bool backwards, std::vector<Edge> &result) const {
    result.clear();
    Point p = point(u);
    if (grid.occupied(grid.cell(p)))
        return;
    const auto &cfg = configs[type(u)];
    const auto &arcs = backwards ? cfg.in : cfg.out;
    result.reserve(arcs.size());
    for (const Arc &a : arcs) {
        Point q = p + a.delta;
        Point source = backwards ? q : p;
        Point target = backwards ? p : q;
        if (!traversable(source, target, a.primitive))
            continue;
        result.push_back({at(q, a.type), a.cost, a.primitive, a.slot});
    }
}

Edge Graph::edge_from(Node source, int slot) const {
    const Arc &arc = configs[type(source)].out.at(slot);
    return {at(point(source) + arc.delta, arc.type), arc.cost, arc.primitive, slot};
}

bool Graph::traversable(Point source, Point target, int primitive) const {
    if (mesh)
        return grid.contains(source) && grid.contains(target) &&
               !grid.occupied(grid.cell(source)) && !grid.occupied(grid.cell(target));
    // The lattice trace already includes both endpoints.
    for (Point offset : primitives[primitive].trace) {
        Point cell = source + offset;
        if (!grid.contains(cell) || grid.occupied(grid.cell(cell)))
            return false;
    }
    return true;
}
