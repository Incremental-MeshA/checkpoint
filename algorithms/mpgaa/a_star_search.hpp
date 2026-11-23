#pragma once
#include "../../src/indexed_open.hpp"

// Prefix labels only order the search. At a primitive endpoint the parent is
// always a pose, so the same reconstruction works for a normal edge or a jump.
class AStarSearch {
    struct State {
        Cost g = Cost::inf();
        Node parent = -1;
        Edge parent_edge{};
        HeapSlot open_slot;
    };
    Node s_init;
    std::unordered_map<Node, State> states;
    IndexedOpen Open;
    QueueOps &open_ops;

public:
    // Algorithm 1, lines 1-13. Each search has fresh states with g = infinity.
    AStarSearch(Node s_init, Cost h_init, QueueOps &open_ops) : s_init(s_init), open_ops(open_ops) {
        State &state = states[s_init];
        state.g = {};
        Open.set(s_init, {h_init, h_init}, state.open_slot, open_ops);
    }
    bool open_empty() const { return Open.empty(); }
    Node pop_min_f() { return Open.pop(open_ops).first; }
    Cost g(Node s) const {
        auto found = states.find(s);
        return found == states.end() ? Cost::inf() : found->second.g;
    }
    // Algorithm 1, lines 19-27. edge leads from parent to s'.
    void relax(Node parent, Edge edge, Cost candidate_g, Cost h_prime) {
        State &state = states[edge.node];
        if (!(candidate_g < state.g)) return;
        state.g = candidate_g;
        state.parent = parent;
        state.parent_edge = edge;
        Open.set(edge.node, {state.g + h_prime, h_prime}, state.open_slot, open_ops);
    }
    std::vector<Edge> path_from_start(Node s) const {
        std::vector<Edge> result;
        while (s != s_init) {
            const State &state = states.at(s);
            result.push_back(state.parent_edge);
            s = state.parent;
        }
        std::reverse(result.begin(), result.end());
        return result;
    }
};
