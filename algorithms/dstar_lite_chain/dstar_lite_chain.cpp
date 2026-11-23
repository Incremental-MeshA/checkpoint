#include "../../src/planner.hpp"

// Contraction is performed once, before creating persistent labels and change indices.
// Node IDs for poses remain unchanged; only the mesh arcs are replaced.
std::unique_ptr<Planner> dstar_lite_chain(Graph &graph, Node goal) {
    graph.contract_chains();
    return dstar_lite(graph, goal);
}
