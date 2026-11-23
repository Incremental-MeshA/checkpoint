#include "../../src/planner.hpp"

// Contraction is performed once, before creating persistent labels and change indices.
// Node IDs for poses remain unchanged; only the mesh arcs are replaced.
std::unique_ptr<Planner> mpgaa_chain(Graph &graph, Node goal) {
    graph.contract_chains();
    return mpgaa(graph, goal);
}
