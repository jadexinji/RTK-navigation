#include <filesystem>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "common/Types.h"
#include "config/LayerConfig.h"
#include "coordinate/CoordinateTransformer.h"
#include "dxf/DxfReader.h"
#include "map/CassMapBuilder.h"
#include "map/FeatureMapBuilder.h"
#include "map/GridMap.h"
#include "map/MapBuilder.h"
#include "navigation/Navigator.h"
#include "planner/AStar.h"
#include "sensor/CassDatReader.h"
#include "sensor/RTKReader.h"
#include "visualization/Viewer.h"

namespace {

using namespace rtk_nav;

struct ProgramOptions {
    std::string csv_path = "data/rtk_points.csv";
    std::string output_path = "output/navigation_result.png";
    std::optional<std::string> cass_path;
    std::optional<std::string> dxf_path;
    std::string layer_config_path = "config/dxf_layers.yaml";
    std::optional<double> start_x;
    std::optional<double> start_y;
    std::optional<double> goal_x;
    std::optional<double> goal_y;
    std::string start_id = "I51";
    std::string goal_id = "I73";
    bool output_was_set = false;
    bool no_gui = false;
};

void printUsage(const char* program_name) {
    std::cout << "Usage:\n"
              << "  " << program_name
              << " [--csv data/rtk_points.csv] [--output output.png] [--no-gui]\n"
              << "  " << program_name
              << " --cass survey.dat [--start-id I51] [--goal-id I73]"
                 " [--output output.png] [--no-gui]\n"
              << "  " << program_name
              << " --dxf site.dxf --layer-config config/dxf_layers.yaml"
                 " --start-x X --start-y Y --goal-x X --goal-y Y"
                 " [--output output.png] [--no-gui]\n";
}

double parseArgumentDouble(const std::string& option, const std::string& value) {
    try {
        std::size_t parsed = 0;
        const double result = std::stod(value, &parsed);
        if (parsed != value.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return result;
    } catch (const std::exception&) {
        throw std::runtime_error(option + " requires a numeric value, got: " + value);
    }
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
        if (arg == "--cass") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--cass requires a DAT file path");
            }
            options.cass_path = argv[++i];
            continue;
        }
        if (arg == "--dxf") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--dxf requires a DXF file path");
            }
            options.dxf_path = argv[++i];
            continue;
        }
        if (arg == "--layer-config") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--layer-config requires a YAML file path");
            }
            options.layer_config_path = argv[++i];
            continue;
        }
        if (arg == "--start-x" || arg == "--start-y" ||
            arg == "--goal-x" || arg == "--goal-y") {
            if (i + 1 >= argc) {
                throw std::runtime_error(arg + " requires a numeric value");
            }
            const double value = parseArgumentDouble(arg, argv[++i]);
            if (arg == "--start-x") {
                options.start_x = value;
            } else if (arg == "--start-y") {
                options.start_y = value;
            } else if (arg == "--goal-x") {
                options.goal_x = value;
            } else {
                options.goal_y = value;
            }
            continue;
        }
        if (arg == "--start-id") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--start-id requires a CASS point id");
            }
            options.start_id = argv[++i];
            continue;
        }
        if (arg == "--goal-id") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--goal-id requires a CASS point id");
            }
            options.goal_id = argv[++i];
            continue;
        }
        if (arg == "--output") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--output requires a file path");
            }
            options.output_path = argv[++i];
            options.output_was_set = true;
            continue;
        }
        throw std::runtime_error("Unknown argument: " + arg);
    }
    return options;
}

PointType displayPointType(const FeatureStyle& style) {
    if (style.semantic == FeatureSemantic::Road) {
        return PointType::Road;
    }
    if (style.semantic == FeatureSemantic::Boundary) {
        return PointType::Boundary;
    }
    if (style.occupancy == OccupancyEffect::Occupied) {
        return PointType::Obstacle;
    }
    return PointType::Survey;
}

const CassPoint& findCassPoint(const std::vector<CassPoint>& points, const std::string& id) {
    const auto result = std::find_if(points.begin(), points.end(), [&](const CassPoint& point) {
        return point.id == id;
    });
    if (result == points.end()) {
        throw std::runtime_error("CASS data is missing point id: " + id);
    }
    return *result;
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

void visualizeResult(const ProgramOptions& options,
                     const std::string& output_path,
                     const GridMap& grid_map,
                     const std::vector<LocalPoint>& local_points,
                     const GridCell& start_cell,
                     const GridCell& goal_cell,
                     const std::vector<GridCell>& path_cells,
                     double navigation_lookahead = 1.0) {
    std::vector<VehicleState> trajectory;
    if (!path_cells.empty()) {
        NavigatorConfig navigator_config;
        navigator_config.lookahead_distance = navigation_lookahead;
        Navigator navigator(navigator_config);
        trajectory = navigator.simulate(pathCellsToWorld(grid_map, path_cells));
    }
    const std::size_t collision_states = static_cast<std::size_t>(std::count_if(
        trajectory.begin(), trajectory.end(), [&](const VehicleState& state) {
            return !grid_map.isFree(grid_map.worldToGrid(state.x, state.y));
        }));

    std::cout << "Grid map: " << grid_map.width() << " x " << grid_map.height()
              << " cells, resolution=" << grid_map.resolution() << " m/cell\n";
    std::cout << "Start cell: row=" << start_cell.row << " col=" << start_cell.col << "\n";
    std::cout << "Goal cell: row=" << goal_cell.row << " col=" << goal_cell.col << "\n";
    std::cout << "A* path cells: " << path_cells.size() << "\n";
    std::cout << "Vehicle trajectory states: " << trajectory.size() << "\n";
    std::cout << "Vehicle trajectory collision states: " << collision_states << "\n";
    if (path_cells.empty()) {
        std::cout << "Warning: no path found. The map will still be visualized.\n";
    }

    Viewer viewer;
    createOutputDirectory(output_path);
    viewer.saveSnapshot(output_path,
                        grid_map,
                        local_points,
                        path_cells,
                        trajectory,
                        start_cell,
                        goal_cell);
    std::cout << "Saved visualization snapshot: " << output_path << "\n";
    if (!options.no_gui) {
        viewer.show(grid_map,
                    local_points,
                    path_cells,
                    trajectory,
                    start_cell,
                    goal_cell);
    }
}

void runGeodeticCsv(const ProgramOptions& options) {
    RTKReader reader;
    const std::vector<RTKPoint> rtk_points = reader.load(options.csv_path);
    const RTKPoint& start_rtk = findRequiredPoint(rtk_points, PointType::Start);
    const RTKPoint& goal_rtk = findRequiredPoint(rtk_points, PointType::Goal);

    CoordinateTransformer transformer(start_rtk.latitude, start_rtk.longitude, start_rtk.height);
    const std::vector<LocalPoint> local_points = transformer.toLocal(rtk_points);
    const LocalPoint& start_local = findRequiredPoint(local_points, PointType::Start);
    const LocalPoint& goal_local = findRequiredPoint(local_points, PointType::Goal);

    MapBuilder map_builder;
    const GridMap grid_map = map_builder.build(local_points);
    const GridCell start_cell = grid_map.worldToGrid(start_local.x, start_local.y);
    const GridCell goal_cell = grid_map.worldToGrid(goal_local.x, goal_local.y);
    const std::vector<GridCell> path_cells = AStar{}.plan(grid_map, start_cell, goal_cell);

    std::cout << "Input mode: WGS84 CSV\n";
    std::cout << "Loaded RTK points: " << rtk_points.size() << "\n";
    std::cout << "ENU origin: " << start_rtk.id
              << " lat=" << transformer.originLatitude()
              << " lon=" << transformer.originLongitude()
              << " h=" << transformer.originHeight() << "\n";
    std::cout << "Navigation goal: " << goal_rtk.id
              << " lat=" << goal_rtk.latitude
              << " lon=" << goal_rtk.longitude
              << " h=" << goal_rtk.height << "\n";
    visualizeResult(options,
                    options.output_path,
                    grid_map,
                    local_points,
                    start_cell,
                    goal_cell,
                    path_cells);
}

void runCassDat(const ProgramOptions& options) {
    CassDatReader reader;
    const std::vector<CassPoint> all_points = reader.load(*options.cass_path);
    const std::vector<CassPoint> survey_points = reader.filterSurveyArea(all_points);
    if (survey_points.empty()) {
        throw std::runtime_error("No survey-area points remain after filtering the base station");
    }

    const CassPoint& start_raw = findCassPoint(survey_points, options.start_id);
    const CassPoint& goal_raw = findCassPoint(survey_points, options.goal_id);
    const auto min_easting = std::min_element(
        survey_points.begin(), survey_points.end(), [](const CassPoint& lhs, const CassPoint& rhs) {
            return lhs.easting < rhs.easting;
        })->easting;
    const auto min_northing = std::min_element(
        survey_points.begin(), survey_points.end(), [](const CassPoint& lhs, const CassPoint& rhs) {
            return lhs.northing < rhs.northing;
        })->northing;

    std::vector<CassLocalPoint> cass_local =
        reader.toLocal(survey_points, min_easting, min_northing);
    std::vector<LocalPoint> display_points;
    display_points.reserve(cass_local.size() + 2);
    for (const CassLocalPoint& point : cass_local) {
        display_points.push_back(point.point);
    }

    LocalPoint start_local{start_raw.id,
                           start_raw.easting - min_easting,
                           start_raw.northing - min_northing,
                           start_raw.height,
                           PointType::Start};
    LocalPoint goal_local{goal_raw.id,
                          goal_raw.easting - min_easting,
                          goal_raw.northing - min_northing,
                          goal_raw.height,
                          PointType::Goal};
    display_points.push_back(start_local);
    display_points.push_back(goal_local);

    CassMapBuilder map_builder;
    const GridMap grid_map = map_builder.build(cass_local, start_local, goal_local);
    const GridCell start_cell = grid_map.worldToGrid(start_local.x, start_local.y);
    const GridCell goal_cell = grid_map.worldToGrid(goal_local.x, goal_local.y);
    const std::vector<GridCell> path_cells = AStar{}.plan(grid_map, start_cell, goal_cell);

    const std::string output_path =
        options.output_was_set ? options.output_path : "output/cass_navigation_result.png";
    std::cout << "Input mode: CHCNAV/CASS planar DAT\n";
    std::cout << "Loaded DAT records: " << all_points.size() << "\n";
    std::cout << "Survey-area points: " << survey_points.size()
              << " (excluded " << all_points.size() - survey_points.size()
              << " distant base-station record(s))\n";
    std::cout << "Local origin: E=" << min_easting << " N=" << min_northing << "\n";
    std::cout << "Start: " << start_raw.id << " E=" << start_raw.easting
              << " N=" << start_raw.northing << "\n";
    std::cout << "Goal: " << goal_raw.id << " E=" << goal_raw.easting
              << " N=" << goal_raw.northing << "\n";
    std::cout << "Approximate classification: B=building, R=water, F=vegetation/structures\n";
    visualizeResult(options,
                    output_path,
                    grid_map,
                    display_points,
                    start_cell,
                    goal_cell,
                    path_cells);
}

void runDxf(const ProgramOptions& options) {
    if (!options.start_x.has_value() || !options.start_y.has_value() ||
        !options.goal_x.has_value() || !options.goal_y.has_value()) {
        throw std::runtime_error(
            "DXF mode requires --start-x, --start-y, --goal-x, and --goal-y");
    }

    const LayerConfig layer_config =
        LayerConfig::load(options.layer_config_path);
    DxfReader reader;
    const std::vector<MapFeature> features = reader.load(*options.dxf_path);
    if (features.empty()) {
        throw std::runtime_error("DXF file contains no supported map features");
    }

    const LocalPoint start{
        "start", *options.start_x, *options.start_y, 0.0, PointType::Start};
    const LocalPoint goal{
        "goal", *options.goal_x, *options.goal_y, 0.0, PointType::Goal};
    FeatureMapBuilder map_builder(layer_config);
    const std::vector<ClassifiedFeature> classified =
        map_builder.classify(features);
    const GridMap grid_map = map_builder.build(features, start, goal);
    const GridCell start_cell = grid_map.worldToGrid(start.x, start.y);
    const GridCell goal_cell = grid_map.worldToGrid(goal.x, goal.y);
    const FeatureMapSettings& map_settings =
        layer_config.mapSettings();
    AStarConfig planner_config;
    planner_config.clearance_cost_radius =
        map_settings.clearance_cost_radius;
    planner_config.clearance_cost_weight =
        map_settings.clearance_cost_weight;
    const std::vector<GridCell> path_cells =
        AStar{planner_config}.plan(grid_map, start_cell, goal_cell);

    std::vector<LocalPoint> display_points;
    for (const ClassifiedFeature& feature : classified) {
        if (feature.style.occupancy == OccupancyEffect::Ignore) {
            continue;
        }
        const PointType point_type = displayPointType(feature.style);
        for (std::size_t i = 0; i < feature.feature.vertices.size(); ++i) {
            const FeatureVertex& vertex = feature.feature.vertices[i];
            display_points.push_back(LocalPoint{
                feature.feature.id + "-" + std::to_string(i),
                vertex.x,
                vertex.y,
                vertex.z,
                point_type
            });
        }
    }
    display_points.push_back(start);
    display_points.push_back(goal);

    const std::string output_path =
        options.output_was_set ? options.output_path
                               : "output/dxf_navigation_result.png";
    std::cout << "Input mode: generic DXF\n";
    std::cout << "DXF features: " << features.size() << "\n";
    std::cout << "Layer config: " << options.layer_config_path << "\n";
    std::cout << "Road-constrained mode: "
              << (map_settings.require_endpoints_on_free ? "enabled" : "disabled")
              << "\n";
    std::cout << "Road-edge cost: radius="
              << map_settings.clearance_cost_radius
              << " m weight=" << map_settings.clearance_cost_weight << "\n";
    visualizeResult(options,
                    output_path,
                    grid_map,
                    display_points,
                    start_cell,
                    goal_cell,
                    path_cells);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const ProgramOptions options = parseArgs(argc, argv);
        if (options.dxf_path.has_value()) {
            runDxf(options);
        } else if (options.cass_path.has_value()) {
            runCassDat(options);
        } else {
            runGeodeticCsv(options);
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\n";
        return 1;
    }
}
