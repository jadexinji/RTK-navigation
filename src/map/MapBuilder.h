#pragma once

#include <vector>

#include "common/Types.h"
#include "map/GridMap.h"

namespace rtk_nav {

struct MapBuilderConfig {
    double resolution = 0.5;
    double padding = 5.0;
    double road_radius = 2.5;
    double obstacle_radius = 1.1;
    double boundary_radius = 0.5;
};

class MapBuilder {
public:
    explicit MapBuilder(MapBuilderConfig config = {});

    GridMap build(const std::vector<LocalPoint>& points) const;

private:
    void markDisk(GridMap& map, double x, double y, double radius, CellState state) const;
    void markLine(GridMap& map,
                  double start_x,
                  double start_y,
                  double end_x,
                  double end_y,
                  double radius,
                  CellState state) const;
    void markOuterBorder(GridMap& map) const;

    MapBuilderConfig config_;
};

}  // namespace rtk_nav
