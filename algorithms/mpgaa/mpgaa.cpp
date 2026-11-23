#include "../../src/changed_edges.hpp"
#include "a_star_search.hpp"
#include "mesh_caps.hpp"

namespace {
// Adaptive estimates and cached paths belong to complete poses. Mesh prefixes
// borrow the estimates of their possible endpoints (Finals). This also lets
// ReestablishConsistency work on whole primitives without expanding prefix trees.
struct RepairLabel { HeapSlot q_slot; };
struct LazyLabel { size_t version = 0; };
template<bool lazy>
class MPGAA final : public Planner {
    struct AdaptiveState : std::conditional_t<lazy, LazyLabel, RepairLabel> {
        Cost h;
    };
    std::vector<Config> pose_arcs;
    std::unique_ptr<ChangedEdgeIndex> increased_h_index;
    std::unique_ptr<MeshCaps> caps;
    std::unordered_map<Node, AdaptiveState> adaptive_states;
    std::unordered_map<Node, Edge> next;
    std::vector<double> initial_h_by_cell;

    Cost primitive_cost(int primitive_index) const {
        const Primitive &primitive = graph.primitives[primitive_index];
        Cost cost = primitive.length;
        cost.steps = graph.is_mesh() ? static_cast<int>(primitive.trace.size()) - 1 : 1;
        return cost;
    }
    std::vector<Config> build_pose_arcs() const {
        std::vector<Config> poses(16);
        for (int primitive_index = 0; primitive_index < static_cast<int>(graph.primitives.size()); ++primitive_index) {
            const Primitive &primitive = graph.primitives[primitive_index];
            int slot = static_cast<int>(poses[primitive.start].out.size());
            Cost cost = primitive_cost(primitive_index);
            poses[primitive.start].out.push_back({primitive.end, primitive.delta, primitive_index, cost, slot, -2});
            poses[primitive.end].in.push_back({primitive.start, Point{} - primitive.delta, primitive_index, cost, slot, -2});
        }
        return poses;
    }
    // H(s, s_goal) depends only on the cell for complete poses.
    Cost H(Node s) {
        double &value = initial_h_by_cell[graph.cell(s)];
        if (value < 0) value = graph.forward(s, goal).value();
        return Cost::length(value);
    }
    Cost h(Node s) {
        auto state = adaptive_states.find(s);
        if (state == adaptive_states.end()) return H(s);
        if constexpr (lazy) {
            auto &label = state->second;
            if (label.version < caps->version()) {
                label.h = caps->limit(graph.point(s), label.h, H(s), label.version);
                label.version = caps->version();
            }
        }
        return state->second.h;
    }

    // Algorithm 2, lines 14-22. Propagate decreases from successors to predecessors.
    void ReestablishConsistency(const Batch &batch, Metrics &metrics) {
        auto index_start = Metrics::Clock::now();
        std::unordered_map<Node, Cost> initial_h_decreases;
        increased_h_index->visit(batch, [&](Node s, Edge edge, EdgeChange) {
            Cost candidate_h = edge.cost + h(edge.node);
            if (candidate_h < adaptive_states.at(s).h) {
                auto it = initial_h_decreases.emplace(s, candidate_h).first;
                it->second = std::min(it->second, candidate_h);
            }
        });
        metrics.changed_edges_ms = Metrics::since(index_start);
        metrics.batch_seeds = initial_h_decreases.size();
        IndexedOpen Q;
        // Algorithm 2, lines 5-13. Callers check the decrease before insertion.
        // This variant updates h and Q here. Only BuildPath writes next.
        auto InsertState = [&](Node s, Cost candidate_h, AdaptiveState &state) {
            state.h = candidate_h;
            Q.set(s, {candidate_h, {}}, state.q_slot, metrics.repair);
        };
        for (auto [s, candidate_h] : initial_h_decreases) {
            metrics.record(graph, s, Metrics::Seed);
            InsertState(s, candidate_h, adaptive_states.at(s));
        }
        // Only predecessors with a raised h can violate consistency. Check the
        // candidate h before testing the primitive for collisions.
        while (!Q.empty()) {
            Node s_prime = Q.pop(metrics.repair).first;
            ++metrics.repair_expanded;
            metrics.repair_cells.insert(graph.cell(s_prime));
            metrics.record(graph, s_prime, Metrics::Repair);
            Cost h_prime = adaptive_states.at(s_prime).h;
            if (h_prime == graph.forward(s_prime, goal)) {
                increased_h_index->forget(s_prime);
                adaptive_states.erase(s_prime); // Q no longer holds this state.
            }
            Point target = graph.point(s_prime);
            for (const Arc &arc : pose_arcs[graph.type(s_prime)].in) {
                Point source = target + arc.delta;
                if (!graph.grid.contains(source)) continue;
                Node s = graph.at(source, arc.type);
                auto it = adaptive_states.find(s);
                if (it == adaptive_states.end()) continue;
                Cost candidate_h = arc.cost + h_prime;
                if (candidate_h < it->second.h && graph.traversable(source, target, arc.primitive, -2))
                    InsertState(s, candidate_h, it->second);
            }
        }
    }

    // Algorithm 2, lines 1-4. Return the verified next path on success.
    Result GoalCondition(Node s, std::unordered_set<Node> &invalid_next_path) {
        Result next_path{true, {}};
        Node initial_state = s;
        while (s != goal) {
            auto it = next.find(s);
            if (it == next.end() || invalid_next_path.count(s)) break;
            const Edge &edge = it->second;
            if (h(s) != edge.cost + h(edge.node) ||
                !graph.traversable(graph.point(s), graph.point(edge.node), edge.primitive, -2))
                break;
            next_path.path.push_back(edge);
            s = edge.node;
        }
        if (s == goal) return next_path;
        if (!next_path.path.empty()) {
            invalid_next_path.insert(initial_state);
            for (const Edge &edge : next_path.path) invalid_next_path.insert(edge.node);
        }
        return {};
    }

    void relax_primitive(AStarSearch &astar, Node s_prime, int primitive_index, Cost candidate_g) {
        if (!(candidate_g < astar.g(s_prime))) return;
        const Primitive &primitive = graph.primitives[primitive_index];
        Node origin = graph.at(graph.point(s_prime) - primitive.delta, primitive.start);
        astar.relax(origin, {s_prime, primitive_cost(primitive_index), primitive_index, -1, -2}, candidate_g, h(s_prime));
    }
    void relax_mesh_prefix(AStarSearch &astar, Node prefix, Cost prefix_g) {
        if (!(prefix_g < astar.g(prefix))) return;
        const Config &config = graph.configurations()[graph.type(prefix)];
        Point origin = graph.point(prefix) - config.offset;
        Cost prefix_h = Cost::inf();
        int completion_count = 0, only_primitive = -1;
        Node only_successor = -1;
        Cost only_g;
        for (int primitive_index : config.completions) {
            const Primitive &primitive = graph.primitives[primitive_index];
            Point target = origin + primitive.delta;
            if (!graph.grid.contains(target) || graph.grid.occupied(graph.grid.cell(target))) continue;
            Node s_prime = graph.at(target, primitive.end);
            Cost remaining = primitive_cost(primitive_index);
            remaining.steps -= config.depth;
            Cost candidate_g = prefix_g + remaining;
            // This query already has an equally good or cheaper route to s_prime.
            if (!(candidate_g < astar.g(s_prime))) continue;
            prefix_h = std::min(prefix_h, remaining + h(s_prime));
            ++completion_count;
            only_primitive = primitive_index;
            only_successor = s_prime;
            only_g = candidate_g;
        }
        if (completion_count == 0) return;
        if (completion_count == 1) {
            // The common prefix has already passed collision checks. When a
            // single final remains, validate its tail once and skip its OPEN work.
            const auto &trace = graph.primitives[only_primitive].trace;
            for (size_t k = config.depth + 1; k < trace.size(); ++k) {
                Point cell = origin + trace[k];
                if (!graph.grid.contains(cell) || graph.grid.occupied(graph.grid.cell(cell))) return;
            }
            relax_primitive(astar, only_successor, only_primitive, only_g);
        } else {
            astar.relax(-1, {prefix, {}, -1, -1}, prefix_g, prefix_h);
        }
    }

    // Algorithm 1, lines 29-32. Set next along the newly found path to s.
    std::vector<Edge> BuildPath(const AStarSearch &astar, Node s_start, Node s) {
        auto path = astar.path_from_start(s);
        Node parent = s_start;
        for (const Edge &edge : path) {
            next[parent] = edge;
            parent = edge.node;
        }
        return path;
    }

public:
    MPGAA(Graph &graph, Node s_goal)
        : Planner(graph, s_goal), initial_h_by_cell(graph.grid.blocked.size(), -1.0) {
        if constexpr (lazy) {
            caps = std::make_unique<MeshCaps>(graph, goal);
        } else {
            pose_arcs = build_pose_arcs();
            increased_h_index = std::make_unique<ChangedEdgeIndex>(graph, WatchedEnd::source, &pose_arcs);
        }
    }

    Result plan(Node s_start, const Batch &batch, Metrics &metrics) override {
        if (!batch.clear.empty()) {
            if constexpr (lazy) {
                if (!adaptive_states.empty()) {
                    auto started = Metrics::Clock::now();
                    caps->add(batch.clear);
                    metrics.changed_edges_ms = Metrics::since(started);
                }
            } else ReestablishConsistency(batch, metrics);
        }
        if (graph.isolated_endpoint(s_start, goal)) return {};
        AStarSearch astar(s_start, h(s_start), metrics.open);
        std::vector<Node> Closed; // Only full poses receive adaptive h values.
        std::unordered_set<Node> invalid_next_path;
        while (!astar.open_empty()) {
            Node s = astar.pop_min_f();
            Cost g = astar.g(s);
            if (graph.type(s) < 16) {
                Result next_path = GoalCondition(s, invalid_next_path);
                if (next_path.found) {
                    Cost f = g + h(s);
                    // Algorithm 1, lines 68-69: h(s') = f(s) - g(s').
                    for (Node s_prime : Closed) {
                        Cost updated_h = f - astar.g(s_prime);
                        if (h(s_prime) < updated_h) {
                            auto [it, inserted] = adaptive_states.try_emplace(s_prime);
                            if constexpr (lazy) it->second.version = caps->version();
                            else if (inserted) increased_h_index->watch(s_prime);
                            it->second.h = updated_h;
                        }
                    }
                    auto path_start = Metrics::Clock::now();
                    auto path = BuildPath(astar, s_start, s);
                    if (metrics.tracing) {
                        // The viewer addresses a mesh route by physical trace steps.
                        for (const Edge &edge : next_path.path) metrics.reused_edges += edge.cost.steps;
                    }
                    path.insert(path.end(), next_path.path.begin(), next_path.path.end());
                    metrics.path_ms += Metrics::since(path_start);
                    return {true, std::move(path)};
                }
                Closed.push_back(s);
            }
            metrics.expand(graph, s);
            graph.successors(s, successors);
            for (const Edge &edge : successors) {
                if (edge.primitive >= 0) relax_primitive(astar, edge.node, edge.primitive, g + edge.cost);
                else relax_mesh_prefix(astar, edge.node, g + edge.cost);
            }
        }
        return {};
    }
};
}

std::unique_ptr<Planner> mpgaa(Graph &graph, Node s_goal) {
    return std::make_unique<MPGAA<false>>(graph, s_goal);
}

std::unique_ptr<Planner> mpgaa_lazy(Graph &graph, Node s_goal) {
    return std::make_unique<MPGAA<true>>(graph, s_goal);
}
