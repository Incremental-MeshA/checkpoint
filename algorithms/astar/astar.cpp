#include "../../src/planner.hpp"
#include "paper/KC_astar.hpp"
#include "paper/KC_search_params.hpp"

MyHEAP *HEAP = nullptr; // One author search context per benchmark process.

void Map::expand(ptrVertex v) {
    int cell = v->i * width + v->j;
    ++metrics->expanded;
    metrics->cells.insert(cell);
    if (metrics->tracing) {
        bool prefix = mesh && mesh->theta_by_initial_config[v->config_id] < 0;
        metrics->trace.push_back({int64_t(cell) * 8 + (prefix ? 4 : 0), observed->checks->counts});
        observed->checks->begin_event(metrics->trace.size());
    }
}

void Map::sample(Point from, Point to, int primitive, bool clear, int config, Point origin) {
    auto &checks = *observed->checks;
    if (!checks.sample_next_scan) return;
    CellChecks::Candidate edge{from, to, primitive, clear, checks.counts, origin, {}};
    if (mesh) {
        if (primitive >= 0) edge.motion_ids.push_back(primitive);
        else for (auto final : mesh->primitives_in_config[config])
            edge.motion_ids.push_back(std::get<4>(final));
    }
    checks.samples.back().edges.push_back(std::move(edge));
}

namespace {
// For custom primitive sets with no published MeshInfo, express the common
// graph in the same table format consumed by the authors' successor generator.
void make_mesh_info(const Graph &graph, MeshInfo &info) {
    const auto &configs = graph.configurations();
    info.successors.resize(configs.size());
    info.theta_by_initial_config.assign(configs.size(), -1);
    info.primitives_in_config.resize(configs.size());
    std::vector<int> depth(configs.size());
    for (int heading = 0; heading < 16; ++heading) {
        info.initial_config_by_theta[heading] = heading;
        info.theta_by_initial_config[heading] = heading;
    }
    for (size_t type = 0; type < configs.size(); ++type) {
        for (const auto &arc : configs[type].out) {
            int id = arc.primitive < 0 ? -1 : graph.primitives[arc.primitive].id;
            info.successors[type].emplace_back(arc.delta.y, arc.delta.x, arc.type, id);
            if (id < 0) depth[arc.type] = depth[type] + 1;
        }
        for (int index : configs[type].completions) {
            const auto &p = graph.primitives[index];
            Point remaining = p.delta - configs[type].offset;
            info.primitives_in_config[type].emplace_back(p.end, remaining.y, remaining.x, depth[type], p.id);
        }
    }
}

class PaperAStar final : public Planner {
    MyHEAP pool;
    Map map;
    ControlSet controls;
    std::unique_ptr<MeshInfo> custom_mesh;
    std::unordered_map<int, int> by_id;

    template<class Params>
    Result search(Params &params, Metrics &metrics) {
        ResultSearch answer = AstarSearch(&params, false);
        if (!answer.path_found) return {};
        auto began = Metrics::Clock::now();
        Result result{true, {}};
        // The authors link each terminal node directly to the previous pose.
        // Retain the accepted primitive ID to support parallel custom motions.
        for (auto n = answer.final_node; !(n->parent == NULL_Node); n = n->parent) {
            int index = by_id.at(n->incoming_primitive);
            const auto &p = graph.primitives[index];
            Node end = graph.at({n->vertex->j,n->vertex->i},p.end);
            result.path.push_back({end,p.length,index,-1});
        }
        std::reverse(result.path.begin(),result.path.end());
        HEAP->delete_SearchNode(answer.final_node);
        metrics.path_ms += Metrics::since(began);
        return result;
    }

public:
    PaperAStar(Graph &g, Node goal, MeshInfo const *table) : Planner(g,goal) {
        HEAP = &pool;
        map.width = g.grid.width;
        map.height = g.grid.height;
        map.observed = &g.grid;
        for (int index = 0; index < int(g.primitives.size()); ++index) {
            const auto &p = g.primitives[index];
            if (p.id < 0 || !by_id.emplace(p.id,index).second)
                throw std::runtime_error("Invalid primitive ID");
            if (size_t(p.id) >= controls.list_all_prims.size())
                controls.list_all_prims.resize(size_t(p.id)+1);
            auto *motion = new PaperPrimitive;
            controls.list_all_prims[p.id] = motion;
            controls.prims_by_heading[p.start].push_back(p.id);
            motion->start_theta = p.start;
            motion->goal->i = p.delta.y;
            motion->goal->j = p.delta.x;
            motion->goal->theta = p.end;
            motion->length = p.length.value();
            motion->id = p.id;
            for (Point q : p.trace) motion->add_collision(q.y,q.x);
        }
        size_t layers = 16;
        if (g.is_mesh()) {
            if (!table) {
                custom_mesh = std::make_unique<MeshInfo>();
                make_mesh_info(g,*custom_mesh);
                table = custom_mesh.get();
            }
            map.mesh = table;
            const auto &mesh = *table;
            for (auto &arcs : mesh.successors)
                for (auto [di,dj,to,id] : arcs) {
                    layers = std::max(layers, size_t(to)+1);
                    if (id >= 0 && !by_id.count(id)) throw std::runtime_error("Mesh/control set mismatch");
                }
            for (int initial : mesh.initial_config_by_theta) {
                if (initial < 0) throw std::runtime_error("Missing initial mesh configuration");
                layers = std::max(layers, size_t(initial)+1);
            }
        }
        map.closed_bits.resize((layers*g.grid.blocked.size()+7)/8);
    }

    Result plan(Node start, const Batch &, Metrics &metrics) override {
        if (graph.isolated_endpoint(start,goal)) return {};
        Point s = graph.point(start), t = graph.point(goal);
        Vertex from(s.y,s.x,graph.type(start)), to(t.y,t.x,graph.type(goal));
        map.metrics = &metrics;
        if (graph.is_mesh()) {
            MeshGraphParams params(&from,&to,&map,&controls,map.mesh,1,0,0);
            return search(params,metrics);
        }
        StateLatticeParams params(&from,&to,&map,&controls,1,0,0);
        return search(params,metrics);
    }
};
}

std::unique_ptr<Planner> paper_astar(Graph &g, Node goal, MeshInfo const *table) {
    return std::make_unique<PaperAStar>(g,goal,table);
}
