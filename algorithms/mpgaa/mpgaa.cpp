#include "../../src/changed_edges.hpp"
#include "a_star_search.hpp"
#include "motion_masks.hpp"

namespace {
// Adaptive estimates and cached paths belong to complete poses. Mesh prefixes
// borrow the estimates of their possible endpoints (Finals). This also lets
// ReestablishConsistency work on whole primitives without expanding prefix trees.
class MPGAA final : public Planner {
    struct AdaptiveState {
        Cost h;
        HeapSlot q_slot;
    };
    std::vector<Config> pose_arcs;
    ChangedEdgeIndex increased_h_index;
    std::unordered_map<Node, AdaptiveState> adaptive_states;
    std::unordered_map<Node, Edge> next;
    std::vector<double> initial_h_by_cell;
    std::unique_ptr<MotionMasks> masks;
    size_t learned_arcs = 0;

    bool valid(Node source, Node target, int primitive) {
        if (masks) return masks->flags(source)[masks->slot_of(primitive)];
        return graph.traversable(graph.point(source), graph.point(target), primitive, -2);
    }

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
        return state == adaptive_states.end() ? H(s) : state->second.h;
    }

    // Algorithm 2, lines 14-22. Propagate decreases from successors to predecessors.
    void ReestablishConsistency(const Batch &batch, Metrics &metrics) {
        auto index_start = Metrics::Clock::now();
        std::unordered_map<Node, Cost> initial_h_decreases;
        auto seed = [&](Node s, Edge edge) {
            if (metrics.walkthrough)
                metrics.opened.observe(graph, s, graph.primitives[edge.primitive].id, {}, {});
            Cost candidate_h = edge.cost + h(edge.node);
            if (candidate_h < adaptive_states.at(s).h) {
                auto it = initial_h_decreases.emplace(s, candidate_h).first;
                it->second = std::min(it->second, candidate_h);
            }
        };
        if (masks && masks->scan_repair(learned_arcs, batch.clear.size())) {
            for (const auto &[s, state] : adaptive_states) {
                Point origin = graph.point(s);
                for (const auto &arc : pose_arcs[graph.type(s)].out) {
                    Point target = origin + arc.delta;
                    if (!graph.grid.contains(target)) continue;
                    Node v = graph.at(target, arc.type);
                    if (arc.cost + h(v) < state.h && valid(s, v, arc.primitive))
                        seed(s, {v, arc.cost, arc.primitive, arc.slot, -2});
                }
            }
        } else {
            increased_h_index.visit(batch, [&](Node s, Edge edge, EdgeChange) { seed(s, edge); });
        }
        metrics.changed_edges_ms += Metrics::since(index_start);
        metrics.batch_seeds += initial_h_decreases.size();
        IndexedOpen Q;
        // Algorithm 2, lines 5-13. Callers check the decrease before insertion.
        // This variant updates h and Q here. Only BuildPath writes next.
        auto InsertState = [&](Node s, Cost candidate_h, AdaptiveState &state) {
            if (metrics.walkthrough)
                metrics.lowered.observe(graph, s, -1, state.h, candidate_h);
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
                increased_h_index.forget(s_prime);
                if (masks) learned_arcs -= pose_arcs[graph.type(s_prime)].out.size();
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
                if (candidate_h < it->second.h && valid(s, s_prime, arc.primitive))
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
                !valid(s, edge.node, edge.primitive))
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
        : Planner(graph, s_goal), pose_arcs(build_pose_arcs()),
          increased_h_index(graph, WatchedEnd::source, &pose_arcs),
          initial_h_by_cell(graph.grid.blocked.size(), -1.0) {}
    MPGAA(Graph &graph, Node goal, MaskMode mode) : MPGAA(graph, goal) {
        masks = std::make_unique<MotionMasks>(graph, mode);
    }

    Result plan(Node s_start, const Batch &batch, Metrics &metrics) override {
        if (masks) masks->update(batch);
        if (!batch.clear.empty()) ReestablishConsistency(batch, metrics);
        if (graph.isolated_endpoint(s_start, goal)) return {};
        metrics.astar_started = true;
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
                            if (metrics.walkthrough) {
                                metrics.raised.observe(graph, s_prime, -1, h(s_prime), updated_h);
                                metrics.learned_poses.push_back(int64_t(graph.cell(s_prime)) * 16 + graph.type(s_prime));
                            }
                            auto [it, inserted] = adaptive_states.try_emplace(s_prime);
                            if (inserted) {
                                increased_h_index.watch(s_prime);
                                if (masks) learned_arcs += pose_arcs[graph.type(s_prime)].out.size();
                            }
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
            if (masks) {
                const auto &flags = masks->flags(s);
                for (const auto &arc : pose_arcs[graph.type(s)].out) {
                    if (!flags[masks->slot_of(arc.primitive)]) continue;
                    Node target = graph.at(graph.point(s) + arc.delta, arc.type);
                    relax_primitive(astar, target, arc.primitive, g + arc.cost);
                }
                continue;
            }
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
    return std::make_unique<MPGAA>(graph, s_goal);
}

std::unique_ptr<Planner> mpgaa_boundary(Graph &g, Node goal) {
    return std::make_unique<MPGAA>(g, goal, MaskMode::Boundary);
}
std::unique_ptr<Planner> mpgaa_stamp(Graph &g, Node goal) {
    return std::make_unique<MPGAA>(g, goal, MaskMode::Stamp);
}
std::unique_ptr<Planner> mpgaa_batch(Graph &g, Node goal) {
    return std::make_unique<MPGAA>(g, goal, MaskMode::Batch);
}
std::unique_ptr<Planner> mpgaa_witness(Graph &g, Node goal) {
    return std::make_unique<MPGAA>(g, goal, MaskMode::Witness);
}
