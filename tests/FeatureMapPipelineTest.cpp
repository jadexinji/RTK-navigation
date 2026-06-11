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

MapFeature makeRoad() {
    MapFeature feature;
    feature.id = "road-1";
    feature.layer = "ROAD";
    feature.geometry = FeatureGeometry::Polygon;
    feature.closed = true;
    feature.vertices = {
        FeatureVertex{0.0, 0.0, 0.0},
        FeatureVertex{20.0, 0.0, 0.0},
        FeatureVertex{20.0, 20.0, 0.0},
        FeatureVertex{0.0, 20.0, 0.0}
    };
    return feature;
}

}  // namespace

int main() {
    try {
        const LayerConfig config = LayerConfig::load(RTK_NAV_TEST_LAYER_CONFIG);
        FeatureMapBuilder builder(config);

        const std::vector<MapFeature> features = {
            makeRoad(),
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
        require(map.cell(map.worldToGrid(-1.0, 10.0)) == CellState::Occupied,
                "Closed road polygon was incorrectly expanded by line_width");

        const std::vector<ClassifiedFeature> classified =
            builder.classify(features);
        require(classified.size() == 3, "Unexpected classification result count");
        require(classified[0].style.semantic == FeatureSemantic::Road,
                "ROAD layer did not map to road semantic");
        require(classified[0].style.occupancy == OccupancyEffect::Free,
                "ROAD layer did not map to free cells");
        require(classified[1].style.semantic == FeatureSemantic::Building,
                "BUILDING layer did not map to building semantic");
        require(classified[1].style.occupancy == OccupancyEffect::Occupied,
                "BUILDING layer did not map to occupied cells");
        require(classified[2].style.occupancy == OccupancyEffect::Ignore,
                "Unknown layer should be ignored");

        const GridCell start_cell = map.worldToGrid(start.x, start.y);
        const GridCell goal_cell = map.worldToGrid(goal.x, goal.y);
        AStarConfig planner_config;
        planner_config.clearance_cost_radius =
            config.mapSettings().clearance_cost_radius;
        planner_config.clearance_cost_weight =
            config.mapSettings().clearance_cost_weight;
        const std::vector<GridCell> path =
            AStar{planner_config}.plan(map, start_cell, goal_cell);
        require(!path.empty(), "A* failed to route around the building");
        for (const GridCell& cell : path) {
            require(map.isFree(cell), "A* path entered an occupied cell");
        }

        bool rejected_off_road_start = false;
        try {
            const LocalPoint off_road{
                "off-road", -1.0, 10.0, 0.0, PointType::Start};
            builder.build(features, off_road, goal);
        } catch (const std::runtime_error&) {
            rejected_off_road_start = true;
        }
        require(rejected_off_road_start,
                "Road-constrained map accepted an off-road start point");

        std::cout << "Feature map pipeline test passed: "
                  << path.size() << " path cells\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Feature map pipeline test failed: "
                  << error.what() << "\n";
        return 1;
    }
}
