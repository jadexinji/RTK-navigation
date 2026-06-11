#include <iostream>
#include <stdexcept>
#include <vector>

#include "common/Types.h"
#include "config/LayerConfig.h"
#include "map/FeatureMapBuilder.h"
#include "planner/AStar.h"

namespace {

using namespace rtk_nav;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

MapFeature makeBuilding() {
    MapFeature feature;
    feature.id = "building-1";
    feature.layer = "BUILDING";
    feature.geometry = FeatureGeometry::Polygon;
    feature.closed = true;
    feature.vertices = {
        FeatureVertex{8.0, 4.0, 0.0},
        FeatureVertex{12.0, 4.0, 0.0},
        FeatureVertex{12.0, 16.0, 0.0},
        FeatureVertex{8.0, 16.0, 0.0}
    };
    return feature;
}

MapFeature makeUnknownFeature() {
    MapFeature feature;
    feature.id = "annotation-1";
    feature.layer = "TEXT_NOTE";
    feature.geometry = FeatureGeometry::Polyline;
    feature.vertices = {
        FeatureVertex{2.0, 2.0, 0.0},
        FeatureVertex{18.0, 18.0, 0.0}
    };
    return feature;
}

}  // namespace

int main() {
    try {
        const LayerConfig config = LayerConfig::load(RTK_NAV_TEST_LAYER_CONFIG);
        FeatureMapBuilder builder(config);

        const std::vector<MapFeature> features = {
            makeBuilding(),
            makeUnknownFeature()
        };
        const LocalPoint start{"start", 2.0, 10.0, 0.0, PointType::Start};
        const LocalPoint goal{"goal", 18.0, 10.0, 0.0, PointType::Goal};
        const GridMap map = builder.build(features, start, goal);

        require(map.cell(map.worldToGrid(10.0, 10.0)) == CellState::Occupied,
                "Building polygon center was not rasterized as occupied");
        require(map.isFree(map.worldToGrid(start.x, start.y)),
                "Start cell is not free");
        require(map.isFree(map.worldToGrid(goal.x, goal.y)),
                "Goal cell is not free");

        const std::vector<ClassifiedFeature> classified =
            builder.classify(features);
        require(classified.size() == 2, "Unexpected classification result count");
        require(classified[0].style.semantic == FeatureSemantic::Building,
                "BUILDING layer did not map to building semantic");
        require(classified[0].style.occupancy == OccupancyEffect::Occupied,
                "BUILDING layer did not map to occupied cells");
        require(classified[1].style.occupancy == OccupancyEffect::Ignore,
                "Unknown layer should be ignored");

        const GridCell start_cell = map.worldToGrid(start.x, start.y);
        const GridCell goal_cell = map.worldToGrid(goal.x, goal.y);
        const std::vector<GridCell> path =
            AStar{}.plan(map, start_cell, goal_cell);
        require(!path.empty(), "A* failed to route around the building");
        for (const GridCell& cell : path) {
            require(map.isFree(cell), "A* path entered an occupied cell");
        }

        std::cout << "Feature map pipeline test passed: "
                  << path.size() << " path cells\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Feature map pipeline test failed: "
                  << error.what() << "\n";
        return 1;
    }
}
