#include "map/CassMapBuilder.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rtk_nav {
namespace {

bool pointInPolygon(double x, double y, const std::vector<LocalPoint>& polygon) {
    bool inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const double xi = polygon[i].x;
        const double yi = polygon[i].y;
        const double xj = polygon[j].x;
        const double yj = polygon[j].y;
        const bool crosses = ((yi > y) != (yj > y)) &&
                             (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if (crosses) {
            inside = !inside;
        }
    }
    return inside;
}

}  // namespace

CassMapBuilder::CassMapBuilder(CassMapConfig config) : config_(config) {
    if (config_.resolution <= 0.0) {
        throw std::runtime_error("CASS map resolution must be positive");
    }
}

GridMap CassMapBuilder::build(const std::vector<CassLocalPoint>& points,
                              const LocalPoint& start,
                              const LocalPoint& goal) const {
    if (points.empty()) {
        throw std::runtime_error("Cannot build CASS map from empty points");
    }

    double min_x = points.front().point.x;
    double max_x = points.front().point.x;
    double min_y = points.front().point.y;
    double max_y = points.front().point.y;
    for (const CassLocalPoint& point : points) {
        min_x = std::min(min_x, point.point.x);
        max_x = std::max(max_x, point.point.x);
        min_y = std::min(min_y, point.point.y);
        max_y = std::max(max_y, point.point.y);
    }

    min_x -= config_.padding;
    max_x += config_.padding;
    min_y -= config_.padding;
    max_y += config_.padding;

    const int width = static_cast<int>(std::ceil((max_x - min_x) / config_.resolution)) + 1;
    const int height = static_cast<int>(std::ceil((max_y - min_y) / config_.resolution)) + 1;
    GridMap map(width,
                height,
                config_.resolution,
                min_x,
                min_y,
                CellState::Free);

    const std::vector<LocalPoint> building = groupPoints(points, "B");
    const std::vector<LocalPoint> water = groupPoints(points, "R");
    if (building.size() >= 3) {
        fillPolygon(map, building, config_.building_inflation);
    }
    if (water.size() >= 3) {
        fillPolygon(map, water, config_.water_inflation);
    }

    for (const CassLocalPoint& point : points) {
        if (point.group == "F") {
            markDisk(map,
                     point.point.x,
                     point.point.y,
                     config_.vegetation_radius,
                     CellState::Occupied);
        }
    }

    markOuterBorder(map);
    markDisk(map, start.x, start.y, config_.endpoint_clearance, CellState::Free);
    markDisk(map, goal.x, goal.y, config_.endpoint_clearance, CellState::Free);
    return map;
}

std::vector<LocalPoint> CassMapBuilder::groupPoints(const std::vector<CassLocalPoint>& points,
                                                    const std::string& group) const {
    std::vector<LocalPoint> selected;
    for (const CassLocalPoint& point : points) {
        if (point.group == group) {
            selected.push_back(point.point);
        }
    }
    return selected;
}

void CassMapBuilder::fillPolygon(GridMap& map,
                                 const std::vector<LocalPoint>& polygon,
                                 double inflation) const {
    double min_x = polygon.front().x;
    double max_x = polygon.front().x;
    double min_y = polygon.front().y;
    double max_y = polygon.front().y;
    for (const LocalPoint& point : polygon) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }

    const GridCell min_cell = map.worldToGrid(min_x, min_y);
    const GridCell max_cell = map.worldToGrid(max_x, max_y);
    for (int row = min_cell.row; row <= max_cell.row; ++row) {
        for (int col = min_cell.col; col <= max_cell.col; ++col) {
            if (!map.inBounds(row, col)) {
                continue;
            }
            const auto [x, y] = map.gridToWorld(GridCell{row, col});
            if (pointInPolygon(x, y, polygon)) {
                map.setCell(row, col, CellState::Occupied);
            }
        }
    }

    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const LocalPoint& current = polygon[i];
        const LocalPoint& next = polygon[(i + 1) % polygon.size()];
        markLine(map, current, next, inflation, CellState::Occupied);
    }
}

void CassMapBuilder::markDisk(GridMap& map,
                              double x,
                              double y,
                              double radius,
                              CellState state) const {
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

void CassMapBuilder::markLine(GridMap& map,
                              const LocalPoint& start,
                              const LocalPoint& end,
                              double radius,
                              CellState state) const {
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length = std::sqrt(dx * dx + dy * dy);
    const int samples = std::max(1, static_cast<int>(std::ceil(length / (map.resolution() * 0.5))));
    for (int i = 0; i <= samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples);
        markDisk(map, start.x + t * dx, start.y + t * dy, radius, state);
    }
}

void CassMapBuilder::markOuterBorder(GridMap& map) const {
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
