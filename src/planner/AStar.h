#pragma once

#include <vector>

#include "common/Types.h"
#include "map/GridMap.h"

namespace rtk_nav {

struct AStarConfig {
    double clearance_cost_radius = 0.0;
    double clearance_cost_weight = 0.0;
};

class AStar {
public:
    explicit AStar(AStarConfig config = {});

    std::vector<GridCell> plan(const GridMap& map, const GridCell& start, const GridCell& goal) const;

private:
    double heuristic(const GridCell& a, const GridCell& b) const;
    std::vector<double> buildClearanceMap(const GridMap& map) const;
    double clearancePenalty(double clearance) const;

    AStarConfig config_;
};

}  // namespace rtk_nav
