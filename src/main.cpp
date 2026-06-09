#include <filesystem>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "common/Types.h"
#include "coordinate/CoordinateTransformer.h"
#include "map/GridMap.h"
#include "map/MapBuilder.h"
#include "navigation/Navigator.h"
#include "planner/AStar.h"
#include "sensor/RTKReader.h"
#include "visualization/Viewer.h"

namespace {

using namespace rtk_nav;

struct ProgramOptions {
    std::string csv_path = "data/rtk_points.csv";
    std::string output_path = "output/navigation_result.png";
    bool no_gui = false;
};

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [--csv data/rtk_points.csv] [--output output.png] [--no-gui]\n";
}

ProgramOptions parseArgs(int argc, char** argv) {
    ProgramOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--no-gui") {
            options.no_gui = true;
            continue;
        }
        if (arg == "--csv") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--csv requires a file path");
            }
            options.csv_path = argv[++i];
            continue;
        }
        if (arg == "--output") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--output requires a file path");
            }
            options.output_path = argv[++i];
            continue;
        }
        throw std::runtime_error("Unknown argument: " + arg);
    }
    return options;
}

const RTKPoint& findRequiredPoint(const std::vector<RTKPoint>& points, PointType type) {
    for (const RTKPoint& point : points) {
        if (point.type == type) {
            return point;
        }
    }
    throw std::runtime_error("CSV data is missing required point type: " + pointTypeToString(type));
}

const LocalPoint& findRequiredPoint(const std::vector<LocalPoint>& points, PointType type) {
    for (const LocalPoint& point : points) {
        if (point.type == type) {
            return point;
        }
    }
    throw std::runtime_error("Local data is missing required point type: " + pointTypeToString(type));
}

std::vector<std::pair<double, double>> pathCellsToWorld(const GridMap& map,
                                                        const std::vector<GridCell>& path) {
    std::vector<std::pair<double, double>> world_path;
    world_path.reserve(path.size());
    for (const GridCell& cell : path) {
        world_path.push_back(map.gridToWorld(cell));
    }
    return world_path;
}

void createOutputDirectory(const std::string& output_path) {
    const std::filesystem::path path(output_path);
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const ProgramOptions options = parseArgs(argc, argv);

        RTKReader reader;
        const std::vector<RTKPoint> rtk_points = reader.load(options.csv_path);
        const RTKPoint& start_rtk = findRequiredPoint(rtk_points, PointType::Start);
        const RTKPoint& goal_rtk = findRequiredPoint(rtk_points, PointType::Goal);

        CoordinateTransformer transformer(start_rtk.latitude, start_rtk.longitude, start_rtk.height);
        const std::vector<LocalPoint> local_points = transformer.toLocal(rtk_points);
        const LocalPoint& start_local = findRequiredPoint(local_points, PointType::Start);
        const LocalPoint& goal_local = findRequiredPoint(local_points, PointType::Goal);

        MapBuilder map_builder;
        GridMap grid_map = map_builder.build(local_points);
        const GridCell start_cell = grid_map.worldToGrid(start_local.x, start_local.y);
        const GridCell goal_cell = grid_map.worldToGrid(goal_local.x, goal_local.y);

        AStar planner;
        const std::vector<GridCell> path_cells = planner.plan(grid_map, start_cell, goal_cell);

        std::vector<std::pair<double, double>> world_path;
        std::vector<VehicleState> trajectory;
        if (!path_cells.empty()) {
            world_path = pathCellsToWorld(grid_map, path_cells);
            Navigator navigator;
            trajectory = navigator.simulate(world_path);
        }

        std::cout << "Loaded RTK points: " << rtk_points.size() << "\n";
        std::cout << "ENU origin: " << start_rtk.id
                  << " lat=" << transformer.originLatitude()
                  << " lon=" << transformer.originLongitude()
                  << " h=" << transformer.originHeight() << "\n";
        std::cout << "Navigation goal: " << goal_rtk.id
                  << " lat=" << goal_rtk.latitude
                  << " lon=" << goal_rtk.longitude
                  << " h=" << goal_rtk.height << "\n";
        std::cout << "Grid map: " << grid_map.width() << " x " << grid_map.height()
                  << " cells, resolution=" << grid_map.resolution() << " m/cell\n";
        std::cout << "Start cell: row=" << start_cell.row << " col=" << start_cell.col << "\n";
        std::cout << "Goal cell: row=" << goal_cell.row << " col=" << goal_cell.col << "\n";
        std::cout << "A* path cells: " << path_cells.size() << "\n";
        std::cout << "Vehicle trajectory states: " << trajectory.size() << "\n";

        if (path_cells.empty()) {
            std::cout << "Warning: no path found. The map and RTK points will still be visualized.\n";
        }

        Viewer viewer;
        createOutputDirectory(options.output_path);
        viewer.saveSnapshot(options.output_path, grid_map, local_points, path_cells, trajectory, start_cell, goal_cell);
        std::cout << "Saved visualization snapshot: " << options.output_path << "\n";

        if (!options.no_gui) {
            viewer.show(grid_map, local_points, path_cells, trajectory, start_cell, goal_cell);
        }

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\n";
        return 1;
    }
}
