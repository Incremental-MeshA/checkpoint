#include "planner.hpp"

void Batch::apply(Grid &grid) {
    if (block.empty() && clear.empty())
        return;
    // The version both detects duplicates and identifies cells that changed.
    if (++grid.version == 0) {
        std::fill(grid.edit_version.begin(), grid.edit_version.end(), 0);
        ++grid.version;
    }
    auto validate = [&](int cell, bool occupied) {
        if (cell < 0 || size_t(cell) >= grid.blocked.size() ||
            grid.edit_version[cell] == grid.version || bool(grid.blocked[cell]) == occupied)
            throw std::runtime_error("Invalid occupancy change");
        grid.edit_version[cell] = grid.version;
    };
    for (int cell : block)
        validate(cell, true);
    for (int cell : clear)
        validate(cell, false);
    for (int cell : block)
        grid.blocked[cell] = 1;
    for (int cell : clear)
        grid.blocked[cell] = 0;
}
