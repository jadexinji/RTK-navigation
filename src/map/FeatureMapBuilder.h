#pragma once

#include <vector>

#include "common/Types.h"
#include "config/LayerConfig.h"
#include "map/GridMap.h"
#include "map/MapFeature.h"

namespace rtk_nav {

class FeatureMapBuilder {
public:
    explicit FeatureMapBuilder(LayerConfig config);

    GridMap build(const std::vector<MapFeature>& features,
                  const LocalPoint& start,
                  const LocalPoint& goal) const;

    std::vector<ClassifiedFeature> classify(
        const std::vector<MapFeature>& features) const;

private:
    void rasterize(GridMap& map, const ClassifiedFeature& feature) const;
    void fillPolygon(GridMap& map,
                     const std::vector<FeatureVertex>& polygon,
                     CellState state) const;
    void markDisk(GridMap& map,
                  double x,
                  double y,
                  double radius,
                  CellState state) const;
    void markLine(GridMap& map,
                  const FeatureVertex& start,
                  const FeatureVertex& end,
                  double radius,
                  CellState state) const;
    void markOuterBorder(GridMap& map) const;

    LayerConfig config_;
};

}  // namespace rtk_nav
