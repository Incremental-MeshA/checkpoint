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
/// Stores a path length in fixed point and uses the step count to break ties between equal lengths.
struct Cost {
    int64_t whole = 0;
    int64_t fraction = 0;
    int steps = 0;
    static constexpr int64_t unit = int64_t{1} << 52;
    /// Return the sentinel used for unreachable states and unknown search distances.
    static Cost inf() { return {int64_t{1} << 50, 0, 0}; }
    /// Return whether this cost is below the infinity sentinel.
    bool finite() const { return whole < inf().whole; }
    /// Convert the nonnegative distance in value to fixed point with an initial step count of zero.
    static Cost length(double value) {
        auto integral = static_cast<int64_t>(value);
        return {integral, static_cast<int64_t>((value - integral) * unit), 0};
    }
    /// Return the length component as a double for reporting or geometric calculations.
    double value() const { return whole + double(fraction) / unit; }
    /// Expose the whole length, fractional length and step count in comparison order.
    auto parts() const { return std::tie(whole, fraction, steps); }
    /// Return whether this cost precedes b by length, then by step count.
    bool operator<(const Cost &b) const { return parts() < b.parts(); }
    /// Return whether this cost and b have identical lengths and step counts.
    bool operator==(const Cost &b) const { return parts() == b.parts(); }
    /// Return whether the length or step count differs from b.
    bool operator!=(const Cost &b) const { return !(*this == b); }
    /// Add the length and steps of b to this cost. An infinite operand gives an infinite sum.
    Cost operator+(const Cost &b) const {
        if (!finite() || !b.finite())
            return inf();
        auto sum = fraction + b.fraction;
        return {whole + b.whole + sum / unit, sum % unit, steps + b.steps};
    }
    /// Subtract the length and steps of b from this cost, borrowing a whole unit if needed.
    Cost operator-(const Cost &b) const {
        auto diff = fraction - b.fraction;
        return {whole - b.whole - (diff < 0), diff < 0 ? diff + unit : diff, steps - b.steps};
    }
};

/// Holds a grid position or displacement in columns (x) and rows (y).
struct Point {
    int x = 0, y = 0;
    /// Translate this point by the displacement p.
    Point operator+(Point p) const { return {x + p.x, y + p.y}; }
    /// Return the displacement from p to this point.
    Point operator-(Point p) const { return {x - p.x, y - p.y}; }
    /// Return whether p has the same column and row as this point.
    bool operator==(Point p) const { return x == p.x && y == p.y; }
};
/// Describes one motion for a given starting heading, including its ending pose, length and swept cells.
struct Primitive {
    int id, start, end;
    Point delta;
    Cost length;
    std::vector<Point> trace;
};
using Node = int64_t;
/// Describes a transition between configuration types relative to a cell. Incoming arcs store the reverse displacement.
struct Arc {
    int type;
    Point delta;
    int primitive; // index for a completed primitive, -1 for a mesh prefix
    Cost cost;
    int slot; // Position in the source's outgoing arcs, also on the reverse arc.
    int trace_id = -1; // >=0: chain footprint, -2: whole primitive, -1: native edge.
};
/// Groups transitions for a pose heading or mesh prefix and lists the primitives that can complete that prefix.
struct Config {
    int depth = 0; // Number of cell transitions from the initial pose.
    Point offset; // displacement from the initial cell of this prefix
    std::vector<int> completions;
    std::vector<Arc> out, in;
};
/// Identifies the neighbor reached during a graph scan, the transition cost and the collision trace to use.
struct Edge {
    Node node;
    Cost cost;
    int primitive;
    int slot;
    int trace_id = -1;
};
/// Counts all occupancy reads and the distinct cells read during a planning query.
struct CellCounts {
    int unique = 0;
    int64_t reads = 0;
};
/// Collects per-query occupancy counts and samples of collision checks for trace playback.
struct CellChecks {
    /// Records a tested transition, its collision result and the cumulative counts at the time of the check.
    struct Candidate {
        Point source, target;
        int primitive;
        bool clear;
        CellCounts counts;
        Point origin;
        std::vector<int> motion_ids;
    };
    /// Groups the candidate transitions recorded for one trace event.
    struct Sample {
        size_t event;
        std::vector<Candidate> edges;
    };
    std::vector<Sample> samples;
    size_t stride = 1;
    bool sample_next_scan = false;
    /// Use the one-based trace index event to decide whether to sample the next neighbor scan.
    void begin_event(size_t event) {
        // Bound trace memory even on long searches; keep an entire neighbor scan per sample.
        if ((event - 1) % stride == 0 && samples.size() >= 32) {
            stride *= 2;
            /// Drop an older sample s if its event falls between the newly spaced sample points.
            samples.erase(std::remove_if(samples.begin(), samples.end(), [&](const Sample &s) {
                return (s.event - 1) % stride != 0;
            }), samples.end());
        }
        sample_next_scan = (event - 1) % stride == 0;
        if (sample_next_scan)
            samples.push_back({event, {}});
    }
    CellCounts counts;
    std::vector<unsigned char> seen;
    /// Begin a new query on a grid with size cells, clearing all previous samples and visitation flags.
    void reset(size_t size) {
        counts = {};
        samples.clear();
        stride = 1;
        sample_next_scan = false;
        seen.assign(size, 0);
    }
    /// Count one read of the row-major cell index cell, counting it as distinct on its first visit.
    void visit(int cell) {
        ++counts.reads;
        if (!seen[cell]) {
            seen[cell] = 1;
            ++counts.unique;
        }
    }
};
/// Owns the occupancy map and per-cell edit stamps used to reconstruct the previous map state.
struct Grid {
    int width, height;
    std::vector<unsigned char> blocked;
    std::vector<uint32_t> edit_version;
    uint32_t version = 0;
    /// Read the octile map at filename and initialize the occupancy of every cell.
    explicit Grid(const std::string &filename);
    /// Return whether the grid coordinates p are inside the map.
    bool contains(Point p) const { return p.x >= 0 && p.y >= 0 && p.x < width && p.y < height; }
    /// Convert the in-bounds coordinates p to a zero-based row-major cell index.
    int cell(Point p) const { return p.y * width + p.x; }
    CellChecks *checks = nullptr;
    /// Read the occupancy of cell index and record the read when tracing is enabled.
    bool occupied(int index) const {
        if (checks)
            checks->visit(index);
        return blocked[index] != 0;
    }
};
/// Load the motion control set from filename and return the validated primitives in file order.
std::vector<Primitive> load_primitives(const std::string &filename);

/// Mesh configuration and transition tables supplied by the reference A* implementation.
struct MeshInfo;
/// Combines an occupancy grid and motion primitives into a state lattice or a mesh of shared motion prefixes.
class Graph {
    friend class ChangedEdgeIndex;
    bool mesh;
    std::vector<std::vector<Point>> chain_traces;
    std::vector<Config> configs;
    /// Import configuration IDs, transitions and possible primitive completions from the mesh table.
    void import_mesh(const MeshInfo &table);
    /// Link configuration types from and to with displacement delta. primitive identifies the completed motion or is -1 for a prefix.
    void add_arc(int from, int to, Point delta, int primitive);
    /// Return the straight-line distance between cell coordinates a and b as a cost with zero steps.
    Cost distance(Point a, Point b) const;
    /// Replace out with traversable neighbors of node. Set backwards to scan incoming arcs in the forward collision geometry.
    void neighbors(Node node, bool backwards, std::vector<Edge> &out) const;

public:
    /// Return the configuration table indexed by the type component of a node ID.
    const auto &configurations() const { return configs; }
    /// Contract single-entry, single-exit mesh prefixes and preserve the cost and swept cells of each resulting chain.
    void contract_chains();
    /// Return whether the graph uses the mesh representation.
    bool is_mesh() const { return mesh; }
    Grid grid;
    std::vector<Primitive> primitives;
    /// Take ownership of the occupancy map and motion controls. With mesh enabled, use table if supplied or build prefixes from the traces.
    Graph(Grid map, std::vector<Primitive> controls, bool mesh, const MeshInfo *table = nullptr);
    /// Encode coordinates p and configuration type into a node ID.
    Node at(Point p, int type) const { return Node(grid.cell(p)) * configs.size() + type; }
    /// Extract the row-major cell index from node u.
    int cell(Node u) const { return int(u / configs.size()); }
    /// Extract the heading or mesh-prefix configuration type from node u.
    int type(Node u) const { return int(u % configs.size()); }
    /// Recover the grid column and row of node u.
    Point point(Node u) const {
        int c = cell(u);
        return {c % grid.width, c / grid.width};
    }
    /// Estimate the remaining cost from u to goal, considering possible completions when u is a mesh prefix.
    Cost forward(Node u, Node goal) const;
    /// Estimate the cost from start to u using the origin of the current motion when u is a mesh prefix.
    Cost reverse(Node start, Node u) const;
    /// Return whether distinct poses start and goal lack a legal departure from start or a legal arrival at goal.
    bool isolated_endpoint(Node start, Node goal) const;
    /// Replace out with the neighbors that node can reach on the current map.
    void successors(Node node, std::vector<Edge> &out) const { neighbors(node, false, out); }
    /// Replace out with the neighbors that can reach node on the current map.
    void predecessors(Node node, std::vector<Edge> &out) const { neighbors(node, true, out); }
    /// Resolve outgoing arc slot at source into an edge without checking current occupancy.
    Edge edge_from(Node source, int slot) const;
    /// Test the swept cells from source to target. primitive indexes a whole motion. trace_id selects a chain (>= 0),
    /// a whole primitive (-2), or the native lattice or mesh collision check (-1).
    bool traversable(Point source, Point target, int primitive, int trace_id = -1) const;
    /// Validate path from pose start to pose goal and return the indices of its completed primitives in traversal order.
    std::vector<int> decode(Node start, Node goal, const std::vector<Edge> &path) const;
};
