#include "mpgaa.hpp"

class PrefixMPGAA final : public MPGAA {
public:
    using MPGAA::MPGAA;

private:
    void propagate(Node current, Open &queue, Metrics &metrics) override {
        // A non-root prefix has exactly one possible parent, including after
        // contraction. Walk straight up while that parent needs a lower h.
        while (graph.type(current) >= 16) {
            read_predecessors(current, metrics);
            if (predecessors.empty())
                return;
            if (predecessors.size() != 1)
                throw std::logic_error("A prefix must have a unique parent");
            const Edge &parent = predecessors.front();
            Cost bound = parent.cost + heuristic(current);
            if (!lower_heuristic(parent.node, bound))
                return;
            if (graph.type(parent.node) < 16) {
                queue.put(parent.node, {bound, Cost{}}, metrics.repair);
                return;
            }
            // This local visit supersedes any earlier queued decrease.
            queue.erase(parent.node, metrics.repair);
            ++metrics.local_repair_steps;
            current = parent.node;
        }
        MPGAA::propagate(current, queue, metrics);
    }
};

std::unique_ptr<Planner> mpgaa_prefix(Graph &graph, Node goal) {
    return std::make_unique<PrefixMPGAA>(graph, goal);
}
