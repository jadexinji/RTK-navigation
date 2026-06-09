#include "map/MapBuilder.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rtk_nav {
namespace {

std::vector<LocalPoint> pointsOfType(const std::vector<LocalPoint>& points, PointType type) {
    std::vector<LocalPoint> selected;
    for (const LocalPoint& point : points) {
        if (point.type == type) {
            selected.push_back(point);
        }
    }
    return selected;
}

}  // namespace

MapBuilder::MapBuilder(MapBuilderConfig config) : config_(config) {
    if (config_.resolution <= 0.0) {
        throw std::runtime_error("Map resolution must be positive");
    }
}

GridMap MapBuilder::build(const std::vector<LocalPoint>& points) const {
    if (points.empty()) {
        throw std::runtime_error("Cannot build map from empty point set");
    }

    double min_x = points.front().x;
    double max_x = points.front().x;
    double min_y = points.front().y;
    double max_y = points.front().y;

    for (const LocalPoint& point : points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }

    min_x -= config_.padding;
    max_x += config_.padding;
    min_y -= config_.padding;
    max_y += config_.padding;

    const int width = static_cast<int>(std::ceil((max_x - min_x) / config_.resolution)) + 1;
    const int height = static_cast<int>(std::ceil((max_y - min_y) / config_.resolution)) + 1;

    GridMap map(width, height, config_.resolution, min_x, min_y);

    const std::vector<LocalPoint> road_points = pointsOfType(points, PointType::Road);
    for (const LocalPoint& point : road_points) {
        markDisk(map, point.x, point.y, config_.road_radius, CellState::Free);
    }
    for (std::size_t i = 1; i < road_points.size(); ++i) {
        markLine(map,
                 road_points[i - 1].x,
                 road_points[i - 1].y,
                 road_points[i].x,
                 road_points[i].y,
                 config_.road_radius,
                 CellState::Free);
    }

    for (const LocalPoint& point : points) {
        if (point.type == PointType::Start || point.type == PointType::Goal) {
            markDisk(map, point.x, point.y, config_.road_radius, CellState::Free);
        }
    }

    const std::vector<LocalPoint> boundary_points = pointsOfType(points, PointType::Boundary);
    for (const LocalPoint& point : boundary_points) {
        markDisk(map, point.x, point.y, config_.boundary_radius, CellState::Occupied);
    }
    for (std::size_t i = 1; i < boundary_points.size(); ++i) {
        markLine(map,
                 boundary_points[i - 1].x,
                 boundary_points[i - 1].y,
                 boundary_points[i].x,
                 boundary_points[i].y,
                 config_.boundary_radius,
                 CellState::Occupied);
    }

    for (const LocalPoint& point : points) {
        if (point.type == PointType::Obstacle) {
            markDisk(map, point.x, point.y, config_.obstacle_radius, CellState::Occupied);
        }
    }

    markOuterBorder(map);

    for (const LocalPoint& point : points) {
        if (point.type == PointType::Start || point.type == PointType::Goal) {
            map.setCell(map.worldToGrid(point.x, point.y), CellState::Free);
        }
    }

    return map;
}

void MapBuilder::markDisk(GridMap& map, double x, double y, double radius, CellState state) const {
    const GridCell center = map.worldToGrid(x, y);
    const int radius_cells = static_cast<int>(std::ceil(radius / map.resolution()));

    for (int row = center.row - radius_cells; row <= center.row + radius_cells; ++row) {
        for (int col = center.col - radius_cells; col <= center.col + radius_cells; ++col) {
            if (!map.inBounds(row, col)) {
                continue;
            }

            const auto [cell_x, cell_y] = map.gridToWorld(GridCell{row, col});
            const double dx = cell_x - x;
            const double dy = cell_y - y;
            if (std::sqrt(dx * dx + dy * dy) <= radius) {
                map.setCell(row, col, state);
            }
        }
    }
}

void MapBuilder::markLine(GridMap& map,
                          double start_x,
                          double start_y,
                          double end_x,
                          double end_y,
                          double radius,
                          CellState state) const {
    const double dx = end_x - start_x;
    const double dy = end_y - start_y;
    const double length = std::sqrt(dx * dx + dy * dy);
    if (length < 1e-6) {
        markDisk(map, start_x, start_y, radius, state);
        return;
    }

    const int samples = std::max(1, static_cast<int>(std::ceil(length / (map.resolution() * 0.5))));
    for (int i = 0; i <= samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples);
        markDisk(map, start_x + t * dx, start_y + t * dy, radius, state);
    }
}

void MapBuilder::markOuterBorder(GridMap& map) const {
    for (int col = 0; col < map.width(); ++col) {
        map.setCell(0, col, CellState::Occupied);
        map.setCell(map.height() - 1, col, CellState::Occupied);
    }
    for (int row = 0; row < map.height(); ++row) {
        map.setCell(row, 0, CellState::Occupied);
        map.setCell(row, map.width() - 1, CellState::Occupied);
    }
}

}  // namespace rtk_nav
