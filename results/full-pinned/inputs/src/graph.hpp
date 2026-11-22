#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

// Exact sums of the binary64 lengths in car16 (all lengths >= 1).
// Separate integer and fractional parts also allow long detours.
struct Cost {
    int64_t whole = 0;
    int64_t fraction = 0;
    int steps = 0;
    static constexpr int64_t unit = int64_t{1} << 52;
    static Cost inf() { return {int64_t{1} << 50, 0, 0}; }
    bool finite() const { return whole < inf().whole; }
    static Cost length(double value) {
        auto integral = static_cast<int64_t>(value);
        return {integral, static_cast<int64_t>((value - integral) * unit), 0};
    }
    double value() const { return whole + double(fraction) / unit; }
    auto parts() const { return std::tie(whole, fraction, steps); }
    bool operator<(const Cost &b) const { return parts() < b.parts(); }
    bool operator==(const Cost &b) const { return parts() == b.parts(); }
    bool operator!=(const Cost &b) const { return !(*this == b); }
    Cost operator+(const Cost &b) const {
        if (!finite() || !b.finite())
            return inf();
        auto sum = fraction + b.fraction;
        return {whole + b.whole + sum / unit, sum % unit, steps + b.steps};
    }
    Cost operator-(const Cost &b) const {
        auto diff = fraction - b.fraction;
        return {whole - b.whole - (diff < 0), diff < 0 ? diff + unit : diff, steps - b.steps};
    }
};

struct Point {
    int x = 0, y = 0;
    Point operator+(Point p) const { return {x + p.x, y + p.y}; }
    Point operator-(Point p) const { return {x - p.x, y - p.y}; }
    bool operator==(Point p) const { return x == p.x && y == p.y; }
};
struct Primitive {
    int id, start, end;
    Point delta;
    Cost length;
    std::vector<Point> trace;
};
using Node = int64_t;
struct Arc {
    int type;
    Point delta;
    int primitive; // index for a completed primitive, -1 for a mesh prefix
    Cost cost;
    int slot; // Position in the source's outgoing arcs, also on the reverse arc.
};
struct Config {
    Point offset; // displacement from the initial cell of this prefix
    std::vector<int> completions;
    std::vector<Arc> out, in;
};
struct Edge {
    Node node;
    Cost cost;
    int primitive;
    int slot;
};
struct CellCounts {
    int unique = 0;
    int64_t reads = 0;
};
struct CellChecks {
    CellCounts counts;
    std::vector<unsigned char> seen;
    void reset(size_t size) {
        counts = {};
        seen.assign(size, 0);
    }
    void visit(int cell) {
        ++counts.reads;
        if (!seen[cell]) {
            seen[cell] = 1;
            ++counts.unique;
        }
    }
};
struct Grid {
    int width, height;
    std::vector<unsigned char> blocked;
    std::vector<uint32_t> edit_version;
    uint32_t version = 0;
    explicit Grid(const std::string &filename);
    bool contains(Point p) const { return p.x >= 0 && p.y >= 0 && p.x < width && p.y < height; }
    int cell(Point p) const { return p.y * width + p.x; }
    CellChecks *checks = nullptr;
    bool occupied(int index) const {
        if (checks)
            checks->visit(index);
        return blocked[index] != 0;
    }
};
std::vector<Primitive> load_primitives(const std::string &filename);

class Graph {
    friend class ChangedEdgeIndex;
    bool mesh;
    std::vector<Config> configs;
    double scale = 1;
    void add_arc(int from, int to, Point delta, int primitive);
    Cost distance(Point a, Point b) const;
    void neighbors(Node node, bool backwards, std::vector<Edge> &out) const;

public:
    Grid grid;
    std::vector<Primitive> primitives;
    Graph(Grid map, std::vector<Primitive> controls, bool mesh);
    Node at(Point p, int type) const { return Node(grid.cell(p)) * configs.size() + type; }
    int cell(Node u) const { return int(u / configs.size()); }
    int type(Node u) const { return int(u % configs.size()); }
    Point point(Node u) const {
        int c = cell(u);
        return {c % grid.width, c / grid.width};
    }
    Cost forward(Node u, Node goal) const;
    Cost reverse(Node start, Node u) const;
    bool isolated_endpoint(Node start, Node goal) const;
    void successors(Node node, std::vector<Edge> &out) const { neighbors(node, false, out); }
    void predecessors(Node node, std::vector<Edge> &out) const { neighbors(node, true, out); }
    Edge edge_from(Node source, int slot) const;
    bool traversable(Point source, Point target, int primitive) const;
    std::vector<int> decode(Node start, Node goal, const std::vector<Edge> &path) const;
};
