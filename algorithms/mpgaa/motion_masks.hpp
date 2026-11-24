#pragma once
#include "../../src/planner.hpp"

enum class MaskMode { Boundary, Stamp, Batch, Witness };

// The mesh serves as a collision index. OPEN contains full poses only.
class MotionMasks {
    struct Prefix {
        Point offset;
        std::vector<int> children, terminals;
        int begin = 0, end = 0; // DFS interval of all terminal slots below this prefix
    };
    struct Tree {
        std::vector<Prefix> nodes{{}};
        std::vector<int> order;
        Point low{}, high{};
    };
    struct Mask {
        uint64_t verified = 0;
        std::vector<unsigned char> flags;
        std::vector<int> blockers;
    };
    Graph &graph;
    MaskMode mode;
    std::array<Tree, 16> trees;
    std::vector<int> slot;
    std::unordered_map<Node, Mask> masks;
    static constexpr int tile = 8;
    int columns;
    uint64_t epoch = 1;
    std::vector<uint64_t> stamps;
    size_t offset_groups = 0;

    void number(Tree &tree, int i) {
        auto &n = tree.nodes[i];
        n.begin = int(tree.order.size());
        for (int pid : n.terminals) {
            slot[pid] = int(tree.order.size());
            tree.order.push_back(pid);
        }
        for (int child : n.children) number(tree, child);
        n.end = int(tree.order.size());
    }
    bool free(Point p) const {
        return graph.grid.contains(p) && !graph.grid.occupied(graph.grid.cell(p));
    }
    bool fresh(const Tree &tree, Point p, uint64_t verified) const {
        if (verified == epoch) return true;
        if (!verified) return false;
        Point low = p + tree.low, high = p + tree.high;
        for (int y = std::max(0, low.y) / tile; y <= std::min(graph.grid.height-1, high.y) / tile; ++y)
            for (int x = std::max(0, low.x) / tile; x <= std::min(graph.grid.width-1, high.x) / tile; ++x)
                if (stamps[y * columns + x] > verified) return false;
        return true;
    }
    void evaluate(const Tree &tree, int i, Point p, Point parent, Mask &mask,
                  const std::vector<unsigned char> &covered, CellChecks::Sample *sample) {
        const auto &n = tree.nodes[i];
        if (!covered.empty() && std::all_of(covered.begin()+n.begin, covered.begin()+n.end,
                                             [](unsigned char v) { return v != 0; })) return;
        bool clear = free(p + n.offset);
        if (sample) {
            std::vector<int> ids;
            for (int k = n.begin; k < n.end; ++k)
                ids.push_back(graph.primitives[tree.order[k]].id);
            int terminal = ids.size() == 1 && !n.terminals.empty() ? ids.front() : -1;
            sample->edges.push_back({parent, p+n.offset, terminal, clear, graph.grid.checks->counts,
                                     p, std::move(ids)});
        }
        if (!clear) {
            if (mode == MaskMode::Witness) mask.blockers.push_back(i);
            return;
        }
        for (int pid : n.terminals) mask.flags[slot[pid]] = 1;
        for (int child : n.children) evaluate(tree, child, p, p+n.offset, mask, covered, sample);
    }
public:
    MotionMasks(Graph &g, MaskMode variant) : graph(g), mode(variant), slot(g.primitives.size()),
        columns((g.grid.width + tile-1)/tile),
        stamps(columns * ((g.grid.height + tile-1)/tile)) {
        if (!g.is_mesh()) throw std::invalid_argument("Motion masks require mesh");
        for (int heading = 0; heading < 16; ++heading) {
            auto &tree = trees[heading];
            std::set<std::pair<int,int>> offsets;
            for (int pid = 0; pid < int(g.primitives.size()); ++pid) {
                const auto &motion = g.primitives[pid];
                if (motion.start != heading) continue;
                int at = 0;
                for (size_t j = 1; j < motion.trace.size(); ++j) {
                    Point d = motion.trace[j];
                    tree.low = {std::min(tree.low.x,d.x), std::min(tree.low.y,d.y)};
                    tree.high = {std::max(tree.high.x,d.x), std::max(tree.high.y,d.y)};
                    offsets.emplace(d.x,d.y);
                    auto &children = tree.nodes[at].children;
                    auto it = std::find_if(children.begin(), children.end(),
                                           [&](int child) { return tree.nodes[child].offset == d; });
                    if (it == children.end()) {
                        at = int(tree.nodes.size());
                        children.push_back(at);
                        tree.nodes.push_back({d});
                    } else at = *it;
                }
                tree.nodes[at].terminals.push_back(pid);
            }
            number(tree, 0);
            offset_groups += offsets.size() + 1;
        }
    }
    bool scan_repair(size_t learned_arcs, size_t openings) const {
        return (mode == MaskMode::Batch || mode == MaskMode::Witness) &&
               learned_arcs < double(openings) * offset_groups;
    }
    void update(const Batch &batch) {
        if (batch.block.empty() && batch.clear.empty()) return;
        ++epoch;
        for (const auto *edits : {&batch.block, &batch.clear})
            for (int c : *edits)
                stamps[(c / graph.grid.width / tile)*columns + c % graph.grid.width / tile] = epoch;
        if (mode == MaskMode::Boundary) {
            for (auto it = masks.begin(); it != masks.end();) {
                Node u = it->first;
                if (!fresh(trees[graph.type(u)], graph.point(u), it->second.verified))
                    it = masks.erase(it);
                else ++it;
            }
        }
    }
    const std::vector<unsigned char> &flags(Node u) {
        auto &mask = masks[u];
        const auto &tree = trees[graph.type(u)];
        Point p = graph.point(u);
        if (!fresh(tree, p, mask.verified)) {
            std::vector<unsigned char> covered;
            if (!mask.blockers.empty()) {
                covered.assign(tree.order.size(), 0);
                std::vector<int> surviving;
                for (int i : mask.blockers) {
                    const auto &n = tree.nodes[i];
                    if (!free(p + n.offset)) {
                        surviving.push_back(i);
                        std::fill(covered.begin()+n.begin, covered.begin()+n.end, 1);
                    }
                }
                mask.blockers = std::move(surviving);
            }
            mask.flags.assign(tree.order.size(), 0);
            CellChecks::Sample *sample = nullptr;
            auto *checks = graph.grid.checks;
            if (checks && checks->sample_next_scan) {
                checks->sample_next_scan = false;
                sample = &checks->samples.back();
            }
            if (free(p))
                for (int child : tree.nodes[0].children)
                    evaluate(tree, child, p, p, mask, covered, sample);
        }
        mask.verified = epoch;
        return mask.flags;
    }
    int slot_of(int pid) const { return slot[pid]; }
};
