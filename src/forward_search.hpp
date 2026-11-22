#pragma once
#include "planner.hpp"

// Данные одного прямого поиска. A* и MPGAA* используют одинаковые
// расстояния от старта, родительские рёбра и порядок в очереди OPEN.
class ForwardSearch {
    struct Label {
        Cost distance;
        Node parent;
        Edge incoming;
    };

    Node start;
    std::unordered_map<Node, Label> labels;
    Open open;
    QueueOps &operations;

public:
    ForwardSearch(Node start_node, Cost estimate, QueueOps &queue_operations)
        : start(start_node), operations(queue_operations) {
        labels[start] = {Cost{}, -1, {}};
        open.put(start, {estimate, estimate}, operations);
    }

    bool empty() const { return open.empty(); }
    Node pop_best() { return open.pop(operations).second; }

    Cost distance_to(Node node) const {
        auto label = labels.find(node);
        if (label == labels.end())
            return Cost::inf();
        return label->second.distance;
    }

    void record_improvement(Node parent, const Edge &edge, Cost distance, Cost estimate) {
        labels[edge.node] = {distance, parent, edge};
        // При равном f = g + h первой раскрывается вершина с меньшим h.
        open.put(edge.node, {distance + estimate, estimate}, operations);
    }

    std::vector<Edge> path_to(Node node) const {
        std::vector<Edge> path;
        while (node != start) {
            const Label &label = labels.at(node);
            path.push_back(label.incoming);
            node = label.parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};
