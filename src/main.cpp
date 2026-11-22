#include "planner.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>

using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point t) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}
void print_ops(const QueueOps &q) {
    std::cout << "{\"insert\":" << q.insert << ",\"pop\":" << q.pop << ",\"change\":" << q.change
              << ",\"erase\":" << q.erase << ",\"peek\":" << q.peek << "}";
}
void print_cell_frames(const std::vector<Metrics::TraceEvent> &trace, CellCounts final) {
    // Preserve phase boundaries and send exact counts for each displayed frame.
    const size_t batch = (trace.size() + 34) / 35;
    std::cout << ",\"checked_cells\":" << final.unique << ",\"cell_checks\":" << final.reads
              << ",\"cell_frames\":[";
    for (size_t begin = 0; begin < trace.size();) {
        size_t end = begin + 1;
        while (end < trace.size() && end - begin < batch && trace[end].code % 4 == trace[begin].code % 4)
            ++end;
        CellCounts counts = end < trace.size() ? trace[end].before : final;
        if (begin) std::cout << ',';
        std::cout << '[' << end << ',' << counts.unique << ',' << counts.reads << ']';
        begin = end;
    }
    std::cout << ']';
}
void print_edge_samples(const CellChecks &checks) {
    std::cout << ",\"edge_samples\":[";
    bool first = true;
    for (const auto &sample : checks.samples) {
        if (sample.edges.empty()) continue;
        if (!first) std::cout << ',';
        first = false;
        std::cout << '[' << sample.event << ",[";
        for (size_t i = 0; i < sample.edges.size(); ++i) {
            const auto &e = sample.edges[i];
            if (i) std::cout << ',';
            std::cout << '[' << e.source.x << ',' << e.source.y << ',' << e.target.x << ','
                      << e.target.y << ',' << e.primitive << ',' << int(e.clear) << ','
                      << e.counts.unique << ',' << e.counts.reads;
            if (!e.motion_ids.empty()) {
                std::cout << ",[" << e.origin.x << ',' << e.origin.y << "],[";
                for (size_t j = 0; j < e.motion_ids.size(); ++j) {
                    if (j) std::cout << ',';
                    std::cout << e.motion_ids[j];
                }
                std::cout << ']';
            }
            std::cout << ']';
        }
        std::cout << "]]";
    }
    std::cout << ']';
}
int main(int argc, char **argv) {
    try {
        bool tracing = argc == 9 && std::string(argv[8]) == "--trace";
        if (argc != 8 && !tracing)
            throw std::runtime_error(
                "Usage: planner MAP PRIMITIVES GRAPH ALGORITHM GOAL_X GOAL_Y GOAL_H [--trace]");
        Grid grid(argv[1]);
        auto controls = load_primitives(argv[2]);
        std::string kind = argv[3], algorithm = argv[4];
        if (kind != "mesh" && kind != "lattice")
            throw std::runtime_error("Unknown graph");
        std::map<std::string, Factory *> factories = {
            {"astar", astar}, {"dstar_lite", dstar_lite}, {"mpgaa", mpgaa}};
        auto initialized = Clock::now();
        Graph graph(std::move(grid), std::move(controls), kind == "mesh");
        auto pose = [&](int x, int y, int heading) {
            if (!graph.grid.contains({x, y}) || heading < 0 || heading >= 16)
                throw std::runtime_error("Invalid pose");
            return graph.at({x, y}, heading);
        };
        Node goal = pose(std::stoi(argv[5]), std::stoi(argv[6]), std::stoi(argv[7]));
        auto planner = factories.at(algorithm)(graph, goal);
        CellChecks checks;
        if (tracing)
            graph.grid.checks = &checks;
        std::cout << std::setprecision(17) << "{\"init_ms\":" << elapsed(initialized) << "}"
                  << std::endl;
        int x, y, heading, nblock, nclear;
        while (std::cin >> x >> y >> heading >> nblock >> nclear) {
            Batch batch;
            auto read_cells = [&](int count, std::vector<int> &out) {
                if (count < 0 || size_t(count) > graph.grid.blocked.size())
                    throw std::runtime_error("Bad edit count");
                out.resize(count);
                for (int &c : out)
                    if (!(std::cin >> c))
                        throw std::runtime_error("Truncated batch");
            };
            read_cells(nblock, batch.block);
            read_cells(nclear, batch.clear);
            Node start = pose(x, y, heading);
            auto began = Clock::now();
            batch.apply(graph.grid);
            double apply_ms = elapsed(began);
            if (graph.grid.blocked[graph.cell(start)] || graph.grid.blocked[graph.cell(goal)])
                throw std::runtime_error("Occupied query endpoint");
            Metrics m;
            m.tracing = tracing;
            if (tracing)
                checks.reset(graph.grid.blocked.size());
            Result result = planner->plan(start, batch, m);
            auto decode_began = Clock::now();
            std::vector<int> route;
            Cost cost;
            if (result.found) {
                route = graph.decode(start, goal, result.path);
                for (int i : route)
                    cost = cost + graph.primitives[i].length;
            }
            m.path_ms += elapsed(decode_began);
            auto search_cells = m.cells.size();
            m.cells.insert(m.repair_cells.begin(), m.repair_cells.end());
            double ms = elapsed(began);
            // Search includes endpoint checks, suffix validation and metric accounting.
            double search_ms = ms - apply_ms - m.changed_edges_ms - m.path_ms;
            std::cout << "{\"plan_ms\":" << ms << ",\"apply_ms\":" << apply_ms
                      << ",\"changed_edges_ms\":" << m.changed_edges_ms << ",\"search_ms\":" << search_ms
                      << ",\"path_ms\":" << m.path_ms << ",\"outcome\":\""
                      << (result.found ? "found" : "no_path") << "\",\"cost\":";
            if (result.found)
                std::cout << cost.value();
            else
                std::cout << "null";
            std::cout << ",\"expanded\":" << m.expanded + m.repair_expanded
                      << ",\"expanded_cells\":" << m.cells.size()
                      << ",\"search_expanded\":" << m.expanded << ",\"search_cells\":" << search_cells
                      << ",\"repair_expanded\":" << m.repair_expanded
                      << ",\"repair_cells\":" << m.repair_cells.size()
                      << ",\"batch_seeds\":" << m.batch_seeds << ",\"open\":";
            print_ops(m.open);
            std::cout << ",\"repair_queue\":";
            print_ops(m.repair);
            if (tracing) {
                print_cell_frames(m.trace, checks.counts);
                print_edge_samples(checks);
                std::cout << ",\"trace\":[";
                for (size_t i = 0; i < m.trace.size(); ++i) {
                    if (i) std::cout << ',';
                    std::cout << m.trace[i].code;
                }
                std::cout << "],\"reuse_from_edge\":";
                if (m.reused_edges)
                    std::cout << result.path.size() - m.reused_edges;
                else
                    std::cout << "null";
            }
            std::cout << ",\"route\":[";
            for (size_t i = 0; i < route.size(); ++i) {
                if (i)
                    std::cout << ',';
                std::cout << graph.primitives[route[i]].id;
            }
            std::cout << "]}" << std::endl;
        }
        if (!std::cin.eof())
            throw std::runtime_error("Invalid query");
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
