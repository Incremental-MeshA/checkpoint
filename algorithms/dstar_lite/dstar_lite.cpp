#include "../../src/changed_edges.hpp"
#include "../../src/indexed_open.hpp"

namespace {
class DStarLite final : public Planner {
    struct State {
        Cost g = Cost::inf(), rhs = Cost::inf();
        int via = -1;
        HeapSlot slot;
    };
    struct Candidate {
        Cost cost = Cost::inf();
        int via = -1;
        void consider(Cost value, int edge) {
            if (value < cost || (value.finite() && value == cost && edge < via)) {
                cost = value;
                via = edge;
            }
        }
    };
    std::unordered_map<Node, State> states;
    IndexedOpen open;
    ChangedEdgeIndex targets{graph, WatchedEnd::target};
    Node start = -1;
    Cost km;
    // Many prefix states project onto the same primitive origin. A lower bound
    // depends only on absolute displacement, even after the robot moves.
    std::vector<double> geometric = std::vector<double>(graph.grid.blocked.size(), -1.0);

    Cost estimate(Node u) {
        Point origin = graph.point(u) - graph.configurations()[graph.type(u)].offset;
        Point delta = origin - graph.point(start);
        Point d{std::abs(delta.x),std::abs(delta.y)};
        if (!graph.grid.contains(d)) return graph.reverse(start,u);
        double &cached = geometric[graph.grid.cell(d)];
        if (cached < 0) cached = graph.reverse(start,u).value();
        return Cost::length(cached);
    }

    Key key(Node u, const State &s) {
        Cost best = std::min(s.g,s.rhs);
        return {best + estimate(u) + km,best};
    }
    void schedule(Node u, State &s, Metrics &m) {
        if (s.g == s.rhs) open.erase(s.slot,m.open);
        else open.set(u,key(u,s),s.slot,m.open);
    }
    void improve(Node u, Cost candidate, int slot, Metrics &m) {
        if (u == goal || !candidate.finite()) return;
        State &s = states[u];
        if (candidate < s.rhs) {
            s.rhs = candidate;
            s.via = slot;
            schedule(u,s,m);
        } else if (candidate == s.rhs && slot < s.via) {
            s.via = slot;
        }
    }
    // Only losing the selected successor requires scanning the other choices.
    void replace_successor(Node u, State &s, Metrics &m) {
        Candidate best;
        graph.successors(u,successors);
        for (const Edge &edge : successors) {
            auto target = states.find(edge.node);
            if (target != states.end()) best.consider(edge.cost + target->second.g,edge.slot);
        }
        s.rhs = best.cost;
        s.via = best.via;
        schedule(u,s,m);
    }
    void apply_edits(const Batch &batch, Metrics &m) {
        struct Change { bool lost = false; Candidate opened; };
        std::unordered_map<Node, Change> changed;
        auto began = Metrics::Clock::now();
        targets.visit(batch,[&](Node u, Edge edge, EdgeChange action) {
            if (u == goal) return;
            auto s = states.find(u);
            if (action == EdgeChange::closed) {
                if (s != states.end() && edge.slot == s->second.via) changed[u].lost = true;
            } else {
                Cost value = edge.cost + states.at(edge.node).g;
                if (s == states.end() || value < s->second.rhs ||
                    (value == s->second.rhs && edge.slot < s->second.via))
                    changed[u].opened.consider(value,edge.slot);
            }
        });
        m.changed_edges_ms = Metrics::since(began);
        m.batch_seeds = changed.size();
        for (const auto &[u, change] : changed) {
            m.record(graph,u,Metrics::Seed);
            if (change.lost) replace_successor(u,states.at(u),m);
            else improve(u,change.opened.cost,change.opened.via,m);
        }
    }
    void propagate(Metrics &m) {
        State &robot = states[start];
        while (open.minimum(m.open) < key(start,robot) || robot.rhs != robot.g) {
            if (open.empty()) throw std::logic_error("Inconsistent start with empty D* OPEN");
            auto [u,old] = open.pop(m.open);
            State &s = states.at(u);
            Key now = key(u,s);
            // Moving the start invalidates keys lazily; inconsistency can reopen
            // any old state. A permanent CLOSED flag would invalidate this loop.
            if (old < now) {
                open.set(u,now,s.slot,m.open);
                continue;
            }
            m.expand(graph,u);
            graph.predecessors(u,predecessors);
            if (s.rhs < s.g) {
                if (!s.g.finite()) targets.watch(u);
                s.g = s.rhs;
                for (const Edge &pred : predecessors)
                    improve(pred.node,pred.cost+s.g,pred.slot,m);
            } else {
                s.g = Cost::inf();
                targets.forget(u);
                schedule(u,s,m);
                for (const Edge &pred : predecessors) {
                    auto it = states.find(pred.node);
                    if (it != states.end() && it->second.via == pred.slot)
                        replace_successor(pred.node,it->second,m);
                }
            }
        }
    }

public:
    using Planner::Planner;
    Result plan(Node robot, const Batch &batch, Metrics &m) override {
        if (start == -1) {
            start = robot;
            states[goal].rhs = {};
            schedule(goal,states.at(goal),m);
        } else {
            if (graph.cell(start) != graph.cell(robot)) {
                // Conservative slack for rounded hypot preserves the lazy-key bound.
                constexpr double slack = 16 * std::numeric_limits<double>::epsilon() *
                                         std::numeric_limits<int>::max();
                km = km + graph.reverse(start,robot) + Cost::length(slack);
            }
            start = robot;
            if (!batch.clear.empty() || !batch.block.empty()) apply_edits(batch,m);
        }
        if (graph.isolated_endpoint(start,goal)) return {};
        propagate(m);
        if (!states.at(start).g.finite()) return {};
        auto began = Metrics::Clock::now();
        Result route{true,{}};
        for (Node u = start; u != goal;) {
            const State &s = states.at(u);
            Edge edge = graph.edge_from(u,s.via);
            Cost next = states.at(edge.node).g;
            if (next + edge.cost != s.g || !(next < s.g))
                throw std::logic_error("Inconsistent D* policy");
            route.path.push_back(edge);
            u = edge.node;
        }
        m.path_ms += Metrics::since(began);
        return route;
    }
};
}

std::unique_ptr<Planner> dstar_lite(Graph &g, Node goal) {
    return std::make_unique<DStarLite>(g,goal);
}
