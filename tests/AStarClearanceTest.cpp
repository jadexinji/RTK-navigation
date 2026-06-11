#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "map/GridMap.h"
#include "planner/AStar.h"

namespace {

using namespace rtk_nav;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

}  // namespace

int main() {
    try {
        GridMap map(21, 11, 1.0, 0.0, 0.0, CellState::Occupied);
        for (int row = 1; row <= 9; ++row) {
            for (int col = 1; col <= 19; ++col) {
                map.setCell(row, col, CellState::Free);
            }
        }

        const GridCell start{1, 1};
        const GridCell goal{1, 19};
        const std::vector<GridCell> shortest =
            AStar{}.plan(map, start, goal);
        require(!shortest.empty(), "Baseline A* path was not found");

        AStarConfig config;
        config.clearance_cost_radius = 4.0;
        config.clearance_cost_weight = 8.0;
        const std::vector<GridCell> centered =
            AStar{config}.plan(map, start, goal);
        require(!centered.empty(), "Clearance-aware A* path was not found");

        const int baseline_max_row =
            std::max_element(
                shortest.begin(),
                shortest.end(),
                [](const GridCell& lhs, const GridCell& rhs) {
                    return lhs.row < rhs.row;
                })->row;
        const int centered_max_row =
            std::max_element(
                centered.begin(),
                centered.end(),
                [](const GridCell& lhs, const GridCell& rhs) {
                    return lhs.row < rhs.row;
                })->row;

        require(baseline_max_row == 1,
                "Baseline A* unexpectedly left the shortest edge route");
        require(centered_max_row >= 4,
                "Clearance-aware A* did not move toward the road interior");

        std::cout << "A* clearance test passed: baseline max row="
                  << baseline_max_row
                  << ", centered max row=" << centered_max_row << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "A* clearance test failed: "
                  << error.what() << "\n";
        return 1;
    }
}
