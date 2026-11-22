#include "../src/forward_search.hpp"
#include "../src/changed_edges.hpp"

class MPGAA final : public Planner {
public:
    using Planner::Planner;

    Result plan(Node start, const Batch &batch, Metrics &metrics) override {
        // После открытия клеток прежняя оценка до цели может быть завышена.
        if (!batch.clear.empty())
            repair_heuristic(batch, metrics);
        if (graph.isolated_endpoint(start, goal))
            return {};

        ForwardSearch search(start, heuristic(start), metrics.open);
        std::vector<Node> expanded;
        std::unordered_set<Node> invalid_suffixes; // Только для текущего запроса.
        while (!search.empty()) {
            Node current = search.pop_best();

            // Поиск может остановиться раньше цели, если остаток пути проверен.
            Result suffix = cached_path_to_goal(current, invalid_suffixes);
            if (suffix.found) {
                if (metrics.tracing)
                    metrics.reused_edges = suffix.path.size();
                if (current == start)
                    return suffix;
                Cost path_cost = search.distance_to(current) + heuristic(current);
                learn_from_search(expanded, search, path_cost);
                auto began = Metrics::Clock::now();
                std::vector<Edge> path = search.path_to(current);
                remember_path(start, path);
                path.insert(path.end(), suffix.path.begin(), suffix.path.end());
                metrics.path_ms += Metrics::since(began);
                return {true, std::move(path)};
            }

            metrics.expand(graph, current);
            expanded.push_back(current);
            Cost from_start = search.distance_to(current);
            graph.successors(current, successors);
            for (const Edge &edge : successors) {
                Cost candidate = from_start + edge.cost;
                if (candidate < search.distance_to(edge.node))
                    search.record_improvement(current, edge, candidate, heuristic(edge.node));
            }
        }
        return {};
    }

private:
    // Эти две таблицы сохраняются между запросами к одной и той же цели.
    std::unordered_map<Node, Cost> learned_heuristic;
    ChangedEdgeIndex raised_sources{graph, WatchedEnd::source};
    std::unordered_map<Node, Edge> next_on_path;

    Cost heuristic(Node node) const {
        auto it = learned_heuristic.find(node);
        if (it == learned_heuristic.end())
            return graph.forward(node, goal);
        return it->second;
    }

    Result cached_path_to_goal(Node current, std::unordered_set<Node> &invalid_suffixes) const {
        Result suffix{true, {}};
        Node first = current;
        while (current != goal) {
            auto it = next_on_path.find(current);
            if (it == next_on_path.end() || invalid_suffixes.count(current))
                break;
            const Edge &cached = it->second;
            if (!graph.traversable(graph.point(current), graph.point(cached.node),
                                   cached.primitive) ||
                heuristic(current) != cached.cost + heuristic(cached.node)) {
                invalid_suffixes.insert(current);
                break;
            }

            suffix.path.push_back(cached);
            current = cached.node;
        }
        if (current == goal)
            return suffix;
        // Все пройденные вершины ведут в один и тот же непригодный хвост.
        if (!suffix.path.empty()) {
            invalid_suffixes.insert(first);
            for (const Edge &edge : suffix.path)
                invalid_suffixes.insert(edge.node);
        }
        return {};
    }

    void learn_from_search(const std::vector<Node> &expanded, const ForwardSearch &search,
                           Cost path_cost) {
        // Адаптивная оценка: h(u) = max(h(u), стоимость найденного пути - g(u)).
        for (Node node : expanded) {
            Cost remaining = path_cost - search.distance_to(node);
            if (heuristic(node) < remaining) {
                if (!learned_heuristic.count(node))
                    raised_sources.watch(node);
                learned_heuristic[node] = remaining;
            }
        }
    }

    void remember_path(Node current, const std::vector<Edge> &path) {
        for (const Edge &edge : path) {
            next_on_path[current] = edge;
            current = edge.node;
        }
    }

    void lower_heuristic(Node node, Cost bound, Open &queue, Metrics &metrics) {
        auto it = learned_heuristic.find(node);
        if (it != learned_heuristic.end() && bound < it->second) {
            if (bound == graph.forward(node, goal)) {
                learned_heuristic.erase(it);
                raised_sources.forget(node);
            } else {
                it->second = bound;
            }
            queue.put(node, {bound, Cost{}}, metrics.repair);
        }
    }

    void repair_heuristic(const Batch &batch, Metrics &metrics) {
        // Несколько открытых клеток и рёбер могут понизить h одной вершины.
        // Собираем для неё одну границу до распространения уменьшений.
        std::map<Node, Cost> bounds;
        auto began = Metrics::Clock::now();
        raised_sources.visit(batch, [&](Node source, Edge edge, EdgeChange) {
            Cost bound = edge.cost + heuristic(edge.node);
            if (bound < learned_heuristic.at(source)) {
                auto it = bounds.emplace(source, bound).first;
                it->second = std::min(it->second, bound);
            }
        });
        metrics.changed_edges_ms = Metrics::since(began);
        metrics.batch_seeds = bounds.size();
        Open queue;
        for (const auto &[node, bound] : bounds)
            lower_heuristic(node, bound, queue, metrics);

        // Понижение h(v) может нарушить согласованность у предшественника u.
        while (!queue.empty()) {
            Node current = queue.pop(metrics.repair).second;
            ++metrics.repair_expanded;
            metrics.record(graph, current, Metrics::Repair);
            metrics.repair_cells.insert(graph.cell(current));
            Cost remaining = heuristic(current);
            graph.predecessors(current, predecessors);
            for (const Edge &edge : predecessors)
                lower_heuristic(edge.node, edge.cost + remaining, queue, metrics);
        }
    }
};

std::unique_ptr<Planner> mpgaa(Graph &graph, Node goal) {
    return std::make_unique<MPGAA>(graph, goal);
}
