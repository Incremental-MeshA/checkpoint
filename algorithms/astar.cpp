#include "../src/forward_search.hpp"

class AStar final : public Planner {
public:
    using Planner::Planner;

    Result plan(Node start, const Batch &, Metrics &metrics) override {
        if (graph.isolated_endpoint(start, goal))
            return {};

        // При каждом запросе начинаем с пустой очереди и расстояния g(start) = 0.
        ForwardSearch search(start, graph.forward(start, goal), metrics.open);
        while (!search.empty()) {
            Node current = search.pop_best();
            if (current == goal) {
                auto began = Metrics::Clock::now();
                Result result{true, search.path_to(goal)};
                metrics.path_ms += Metrics::since(began);
                return result;
            }

            metrics.expand(graph, current);
            Cost from_start = search.distance_to(current);
            graph.successors(current, successors);
            for (const Edge &edge : successors) {
                Cost candidate = from_start + edge.cost;
                if (candidate < search.distance_to(edge.node)) {
                    Cost estimate = graph.forward(edge.node, goal);
                    search.record_improvement(current, edge, candidate, estimate);
                }
            }
        }
        return {};
    }
};

std::unique_ptr<Planner> astar(Graph &graph, Node goal) {
    return std::make_unique<AStar>(graph, goal);
}
