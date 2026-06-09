#pragma once

#include <vector>

#include "common/Types.h"
#include "map/GridMap.h"

namespace rtk_nav {

class AStar {
public:
    std::vector<GridCell> plan(const GridMap& map, const GridCell& start, const GridCell& goal) const;

private:
    double heuristic(const GridCell& a, const GridCell& b) const;
};

}  // namespace rtk_nav
