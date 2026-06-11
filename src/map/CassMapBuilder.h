#pragma once

#include <vector>

#include "map/GridMap.h"
#include "sensor/CassDatReader.h"

namespace rtk_nav {

struct CassMapConfig {
    double resolution = 0.5;
    double padding = 3.0;
    double building_inflation = 1.8;
    double water_inflation = 1.0;
    double vegetation_radius = 0.75;
    double endpoint_clearance = 1.0;
};

class CassMapBuilder {
public:
    explicit CassMapBuilder(CassMapConfig config = {});

    GridMap build(const std::vector<CassLocalPoint>& points,
                  const LocalPoint& start,
                  const LocalPoint& goal) const;

private:
    std::vector<LocalPoint> groupPoints(const std::vector<CassLocalPoint>& points,
                                        const std::string& group) const;
    void fillPolygon(GridMap& map,
                     const std::vector<LocalPoint>& polygon,
                     double inflation) const;
    void markDisk(GridMap& map,
                  double x,
                  double y,
                  double radius,
                  CellState state) const;
    void markLine(GridMap& map,
                  const LocalPoint& start,
                  const LocalPoint& end,
                  double radius,
                  CellState state) const;
    void markOuterBorder(GridMap& map) const;

    CassMapConfig config_;
};

}  // namespace rtk_nav
