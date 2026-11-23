#pragma once
#include "../../src/graph.hpp"
#include <array>

// Each opened cell q can enable any primitive covering q, including q in its
// interior. Compile a lower bound shared by all completions of a mesh prefix.
// All projections and rounded-down lengths are in units of 1/256 of a cell.
class MeshCaps {
    static constexpr int scale = 256;
    static constexpr std::array<Point, 8> directions{{
        {256,0}, {181,181}, {0,256}, {-181,181},
        {-256,0}, {-181,-181}, {0,-256}, {181,-181}
    }};
    static constexpr int64_t infinity = std::numeric_limits<int64_t>::max();
    using Row = std::array<int64_t, directions.size()>;
    using Table = std::array<Row, directions.size()>;
    Table correction;
    std::vector<Table> batches;
    Point goal;
    int width;

    static int64_t project(size_t dir, Point p) {
        return int64_t(directions[dir].x) * p.x + int64_t(directions[dir].y) * p.y;
    }
    static Table empty_table() {
        Table table;
        for (Row &row : table) row.fill(infinity);
        return table;
    }
    static int64_t length(const Primitive &p) {
        return p.length.whole * scale + p.length.fraction / (Cost::unit / scale);
    }
    // For witness offset d minimize c(p) - a.d - b.(delta(p)-d).
    // The completion minimum for direction b is shared by all incoming slopes a.
    void include(Point offset, const Row &remaining) {
        for (size_t b = 0; b < directions.size(); ++b)
            for (size_t a = 0; a < directions.size(); ++a)
                correction[b][a] = std::min(correction[b][a], remaining[b] - project(a, offset));
    }

public:
    MeshCaps(const Graph &graph, Node target)
        : correction(empty_table()), goal(graph.point(target)), width(graph.grid.width) {
        if (!graph.is_mesh()) throw std::invalid_argument("mpgaa_lazy requires mesh");
        for (const auto &cfg : graph.configurations()) {
            if (cfg.completions.empty()) continue;
            Row remaining;
            remaining.fill(infinity);
            for (int index : cfg.completions) {
                const auto &p = graph.primitives[index];
                for (size_t b = 0; b < directions.size(); ++b)
                    remaining[b] = std::min(remaining[b], length(p) - project(b, p.delta - cfg.offset));
            }
            include(cfg.offset, remaining);
        }
        // At either boundary the mesh vertex also belongs to other primitives.
        for (const auto &p : graph.primitives) {
            Row start, end;
            for (size_t b = 0; b < directions.size(); ++b)
                start[b] = length(p) - project(b, p.delta);
            end.fill(length(p));
            include({}, start);
            include(p.delta, end);
        }
    }

    size_t version() const { return batches.size(); }

    void add(const std::vector<int> &cleared) {
        Table table = empty_table();
        for (int cell : cleared) {
            Point q{cell % width, cell / width};
            Point tail = goal - q;
            size_t b = 0;
            for (size_t i = 1; i < directions.size(); ++i)
                if (project(i, tail) > project(b, tail)) b = i;
            int64_t suffix = project(b, tail);
            for (size_t a = 0; a < directions.size(); ++a)
                table[b][a] = std::min(table[b][a], project(a, q) + suffix + correction[b][a]);
        }
        batches.push_back(table);
    }

    // For each sector take max_a min_q of the affine lower bounds. This
    // underestimates min_q max_a but costs at most 64 projections per version.
    // Each slope has norm <= 1, so minima/maxima preserve consistency on poses.
    Cost limit(Point pose, Cost learned, Cost base, size_t since) const {
        Row at_pose;
        for (size_t a = 0; a < directions.size(); ++a) at_pose[a] = project(a, pose);
        for (size_t i = since; i < batches.size(); ++i) {
            for (const Row &sector : batches[i]) {
                if (sector[0] == infinity) continue;
                int64_t bound = 0;
                for (size_t a = 0; a < directions.size(); ++a)
                    bound = std::max(bound, sector[a] - at_pose[a]);
                Cost cap{bound / scale, (bound % scale) * (Cost::unit / scale), 0};
                if (!(base < cap)) return base;
                learned = std::min(learned, cap);
            }
        }
        return learned;
    }
};
