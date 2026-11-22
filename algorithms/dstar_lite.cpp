#include "../src/changed_edges.hpp"

class DStarLite final : public Planner {
public:
    using Planner::Planner;

    Result plan(Node new_start, const Batch &batch, Metrics &metrics) override {
        bool first_query = start == -1;
        if (!first_query && graph.cell(start) != graph.cell(new_start)) {
            // Lazy OPEN keys must stay lower bounds when the start moves. Rounding
            // three scaled hypot distances can violate the triangle inequality by
            // a few ulps. Bound that error even for the largest int coordinates.
            constexpr double roundoff = 16 * std::numeric_limits<double>::epsilon()
                                           * std::numeric_limits<int>::max();
            km = km + graph.reverse(start, new_start) + Cost::length(roundoff);
        }
        start = new_start;
        labels.try_emplace(start);

        if (first_query) {
            labels[goal].rhs = {};
            update_open(goal, metrics);
        } else if (!batch.block.empty() || !batch.clear.empty()) {
            update_changed_vertices(batch, metrics);
        }
        if (graph.isolated_endpoint(start, goal))
            return {};

        compute_shortest_path(metrics);
        if (!distance_to_goal(start).finite())
            return {};
        auto began = Metrics::Clock::now();
        Result result = extract_path();
        metrics.path_ms += Metrics::since(began);
        return result;
    }

private:
    struct Label {
        Cost g = Cost::inf();
        Cost rhs = Cost::inf();
        int best_slot = -1; // Ребро, на котором достигнут минимум rhs.
    };
    std::unordered_map<Node, Label> labels;
    ChangedEdgeIndex finite_targets{graph, WatchedEnd::target};
    Open open;
    Node start = -1;
    Cost km;

    Cost distance_to_goal(Node node) const {
        auto it = labels.find(node);
        return it == labels.end() ? Cost::inf() : it->second.g;
    }

    Key calculate_key(Node node) const {
        const Label &label = labels.at(node);
        Cost best = std::min(label.g, label.rhs);
        return {best + graph.reverse(start, node) + km, best};
    }

    void update_open(Node node, Metrics &metrics) {
        const Label &label = labels.at(node);
        if (label.g == label.rhs)
            open.erase(node, metrics.open);
        else
            open.put(node, calculate_key(node), metrics.open);
    }

    // При равенстве берём первое ребро в исходном порядке successors().
    static bool better(Cost cost, int slot, Cost best, int best_slot) {
        return cost < best || (cost == best && slot < best_slot);
    }

    void offer(Node node, int slot, Cost cost, Metrics &metrics) {
        Label &label = labels[node];
        if (better(cost, slot, label.rhs, label.best_slot)) {
            bool new_cost = cost != label.rhs;
            label.rhs = cost;
            label.best_slot = slot;
            if (new_cost)
                update_open(node, metrics);
        }
    }

    // Полный минимум нужен только при потере прежнего лучшего перехода.
    void recompute(Node node, Metrics &metrics) {
        Label &label = labels.at(node);
        label.rhs = Cost::inf();
        label.best_slot = -1;
        graph.successors(node, successors);
        for (const Edge &edge : successors) {
            Cost cost = edge.cost + distance_to_goal(edge.node);
            if (cost < label.rhs) {
                label.rhs = cost;
                label.best_slot = edge.slot;
            }
        }
        update_open(node, metrics);
    }

    void update_changed_vertices(const Batch &batch, Metrics &metrics) {
        struct Update {
            bool lost_best = false;
            Cost cost = Cost::inf();
            int slot = -1;
        };
        std::map<Node, Update> updates;
        auto began = Metrics::Clock::now();
        finite_targets.visit(batch, [&](Node source, Edge edge, EdgeChange change) {
            if (source == goal)
                return;
            auto it = labels.find(source);
            if (change == EdgeChange::closed) {
                if (it != labels.end() && it->second.best_slot == edge.slot)
                    updates[source].lost_best = true;
                return;
            }
            Cost cost = edge.cost + distance_to_goal(edge.node);
            if (it != labels.end() && !better(cost, edge.slot, it->second.rhs, it->second.best_slot))
                return;
            Update &best = updates[source];
            if (better(cost, edge.slot, best.cost, best.slot)) {
                best.cost = cost;
                best.slot = edge.slot;
            }
        });
        metrics.changed_edges_ms = Metrics::since(began);
        metrics.batch_seeds = updates.size();
        for (const auto &[node, update] : updates) {
            metrics.record(graph, node, Metrics::Seed);
            if (update.lost_best)
                recompute(node, metrics); // Уже видит все открытия в этом пакете.
            else
                offer(node, update.slot, update.cost, metrics);
        }
    }

    void compute_shortest_path(Metrics &metrics) {
        const Label &start_label = labels.at(start);
        while (open.top(metrics.open) < calculate_key(start) || start_label.rhs != start_label.g) {
            if (open.empty())
                throw std::logic_error("Inconsistent empty D* queue");
            auto [old_key, current] = open.pop(metrics.open);
            Key new_key = calculate_key(current);
            if (old_key < new_key) {
                open.put(current, new_key, metrics.open);
                continue;
            }

            metrics.expand(graph, current);
            Label &label = labels.at(current);
            graph.predecessors(current, predecessors);
            if (label.rhs < label.g) {
                if (!label.g.finite())
                    finite_targets.watch(current);
                label.g = label.rhs;
                for (const Edge &edge : predecessors)
                    offer(edge.node, edge.slot, edge.cost + label.g, metrics);
            } else {
                label.g = Cost::inf();
                finite_targets.forget(current);
                update_open(current, metrics);
                for (const Edge &edge : predecessors) {
                    auto pred = labels.find(edge.node);
                    if (pred != labels.end() && pred->second.best_slot == edge.slot)
                        recompute(edge.node, metrics);
                }
            }
        }
    }

    Result extract_path() const {
        Result result{true, {}};
        for (Node node = start; node != goal;) {
            const Label &label = labels.at(node);
            if (label.best_slot < 0)
                throw std::logic_error("Missing D* policy");
            Edge next = graph.edge_from(node, label.best_slot);
            Cost remaining = distance_to_goal(next.node);
            if (label.g != next.cost + remaining || !(remaining < label.g))
                throw std::logic_error("Invalid D* policy");
            result.path.push_back(next);
            node = next.node;
        }
        return result;
    }
};

std::unique_ptr<Planner> dstar_lite(Graph &graph, Node goal) {
    return std::make_unique<DStarLite>(graph, goal);
}
