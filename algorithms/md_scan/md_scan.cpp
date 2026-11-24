#include "../../src/planner.hpp"

namespace {
int distance(Point a, Point b) {
    return std::max(std::abs(a.x-b.x), std::abs(a.y-b.y));
}

class MDScan final : public Planner {
    Graph working;
    std::unique_ptr<Planner> backend;
    std::set<int> deferred;
    int64_t correction = 0;
    Result cached;
    std::unordered_map<Node, size_t> positions;
    std::vector<Cost> remaining;

    // A path using a hidden opening must cover q inside some motion. The
    // correction accounts for the worst detour of its trace from its endpoints.
    Batch release(Node start, Cost upper) {
        Batch edits;
        Point s = graph.point(start), t = graph.point(goal);
        for (auto it = deferred.begin(); it != deferred.end();) {
            Point q{*it % graph.grid.width, *it / graph.grid.width};
            int64_t lower = int64_t(distance(s,q)) + distance(q,t) + correction;
            if (lower <= upper.whole) {
                edits.clear.push_back(*it);
                it = deferred.erase(it);
            } else ++it;
        }
        return edits;
    }
    void install(Node start, Result result) {
        cached = std::move(result);
        positions.clear();
        remaining.assign(cached.path.size()+1, {});
        if (!cached.found) return;
        Node pose = start;
        for (size_t i = 0; i < cached.path.size(); ++i) {
            positions[pose] = i;
            pose = cached.path[i].node;
        }
        positions[pose] = cached.path.size();
        for (size_t i = cached.path.size(); i > 0; --i)
            remaining[i-1] = remaining[i] + cached.path[i-1].cost;
    }
    bool feasible(Node start, size_t at) const {
        for (size_t i = at; i < cached.path.size(); ++i) {
            const Edge &edge = cached.path[i];
            if (!working.traversable(working.point(start), working.point(edge.node), edge.primitive, -2))
                return false;
            start = edge.node;
        }
        return true;
    }
    Result search(Node start, Batch &openings, Metrics &metrics) {
        openings.apply(working.grid);
        metrics.reused_edges = 0;
        return backend->plan(start, openings, metrics);
    }
public:
    MDScan(Graph &g, Node goal) : Planner(g, goal), working(g), backend(mpgaa(working, goal)) {
        if (!g.is_mesh()) throw std::invalid_argument("MD Scan requires mesh");
        // Integer Chebyshev distance is bounded by each motion's length.
        for (const auto &p : graph.primitives)
            for (Point q : p.trace)
                correction = std::min(correction, p.length.whole-distance({},q)-distance(q,p.delta));
    }
    Result plan(Node start, const Batch &batch, Metrics &metrics) override {
        working.grid.checks = graph.grid.checks;
        Batch blocks;
        for (int c : batch.block)
            if (!deferred.erase(c)) blocks.block.push_back(c);
        blocks.apply(working.grid);
        deferred.insert(batch.clear.begin(), batch.clear.end());

        auto pos = positions.find(start);
        if (pos != positions.end() && feasible(start, pos->second)) {
            Batch openings = release(start, remaining[pos->second]);
            if (openings.clear.empty()) {
                if (metrics.tracing) metrics.reused_edges = remaining[pos->second].steps;
                return {true, {cached.path.begin()+pos->second, cached.path.end()}};
            }
            install(start, search(start, openings, metrics));
        } else {
            // Expose the query endpoints before obtaining a working upper bound.
            Batch endpoints;
            for (int c : {graph.cell(start), graph.cell(goal)})
                if (deferred.erase(c)) endpoints.clear.push_back(c);
            Result result = search(start, endpoints, metrics);
            Cost upper = result.found ? Cost{} : Cost::inf();
            for (const Edge &e : result.path) upper = upper + e.cost;
            Batch openings = release(start, upper);
            if (!openings.clear.empty()) result = search(start, openings, metrics);
            install(start, std::move(result));
        }
        return cached;
    }
};
}

std::unique_ptr<Planner> md_scan(Graph &graph, Node goal) {
    return std::make_unique<MDScan>(graph, goal);
}
