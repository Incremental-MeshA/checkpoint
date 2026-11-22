#include "mpgaa.hpp"

std::unique_ptr<Planner> mpgaa(Graph &graph, Node goal) {
    return std::make_unique<MPGAA>(graph, goal);
}
