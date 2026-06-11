#include "map/FeatureMapBuilder.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace rtk_nav {
namespace {

bool pointInPolygon(double x,
                    double y,
                    const std::vector<FeatureVertex>& polygon) {
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

CellState effectToCellState(OccupancyEffect effect) {
    switch (effect) {
        case OccupancyEffect::Free:
            return CellState::Free;
        case OccupancyEffect::Occupied:
            return CellState::Occupied;
        case OccupancyEffect::Ignore:
            break;
    }
    return CellState::Unknown;
}

int effectOrder(OccupancyEffect effect) {
    switch (effect) {
        case OccupancyEffect::Free:
            return 0;
        case OccupancyEffect::Occupied:
            return 1;
        case OccupancyEffect::Ignore:
            return 2;
    }
    return 2;
}

}  // namespace

FeatureMapBuilder::FeatureMapBuilder(LayerConfig config)
    : config_(std::move(config)) {}

GridMap FeatureMapBuilder::build(const std::vector<MapFeature>& features,
                                 const LocalPoint& start,
                                 const LocalPoint& goal) const {
    bool has_vertex = false;
    double min_x = std::min(start.x, goal.x);
    double max_x = std::max(start.x, goal.x);
    double min_y = std::min(start.y, goal.y);
    double max_y = std::max(start.y, goal.y);

    for (const MapFeature& feature : features) {
        for (const FeatureVertex& vertex : feature.vertices) {
            has_vertex = true;
            min_x = std::min(min_x, vertex.x);
            max_x = std::max(max_x, vertex.x);
            min_y = std::min(min_y, vertex.y);
            max_y = std::max(max_y, vertex.y);
        }
    }
    if (!has_vertex) {
        throw std::runtime_error("Cannot build a feature map without geometry vertices");
    }

    const FeatureMapSettings& settings = config_.mapSettings();
    min_x -= settings.padding;
    max_x += settings.padding;
    min_y -= settings.padding;
    max_y += settings.padding;

    const int width =
        static_cast<int>(std::ceil((max_x - min_x) / settings.resolution)) + 1;
    const int height =
        static_cast<int>(std::ceil((max_y - min_y) / settings.resolution)) + 1;
    GridMap map(width,
                height,
                settings.resolution,
                min_x,
                min_y,
                settings.default_state);

    std::vector<ClassifiedFeature> classified = classify(features);
    std::stable_sort(classified.begin(),
                     classified.end(),
                     [](const ClassifiedFeature& lhs, const ClassifiedFeature& rhs) {
                         return effectOrder(lhs.style.occupancy) <
                                effectOrder(rhs.style.occupancy);
                     });
    for (const ClassifiedFeature& feature : classified) {
        rasterize(map, feature);
    }

    markOuterBorder(map);
    markDisk(map,
             start.x,
             start.y,
             settings.endpoint_clearance,
             CellState::Free);
    markDisk(map,
             goal.x,
             goal.y,
             settings.endpoint_clearance,
             CellState::Free);
    return map;
}

std::vector<ClassifiedFeature> FeatureMapBuilder::classify(
    const std::vector<MapFeature>& features) const {
    std::vector<ClassifiedFeature> classified;
    classified.reserve(features.size());
    for (const MapFeature& feature : features) {
        classified.push_back(ClassifiedFeature{feature, config_.classify(feature)});
    }
    return classified;
}

void FeatureMapBuilder::rasterize(GridMap& map,
                                  const ClassifiedFeature& feature) const {
    if (feature.style.occupancy == OccupancyEffect::Ignore ||
        feature.feature.vertices.empty()) {
        return;
    }

    const CellState state = effectToCellState(feature.style.occupancy);
    const double radius =
        feature.style.inflation + feature.style.line_width * 0.5;
    const bool closed = feature.feature.closed ||
                        feature.style.force_closed ||
                        feature.feature.geometry == FeatureGeometry::Polygon;

    if (feature.feature.geometry == FeatureGeometry::Point) {
        const FeatureVertex& point = feature.feature.vertices.front();
        markDisk(map, point.x, point.y, radius, state);
        return;
    }

    if (closed && feature.feature.vertices.size() >= 3) {
        fillPolygon(map, feature.feature.vertices, state);
    }

    for (std::size_t i = 1; i < feature.feature.vertices.size(); ++i) {
        markLine(map,
                 feature.feature.vertices[i - 1],
                 feature.feature.vertices[i],
                 radius,
                 state);
    }
    if (closed && feature.feature.vertices.size() >= 2) {
        markLine(map,
                 feature.feature.vertices.back(),
                 feature.feature.vertices.front(),
                 radius,
                 state);
    }
}

void FeatureMapBuilder::fillPolygon(
    GridMap& map,
    const std::vector<FeatureVertex>& polygon,
    CellState state) const {
    double min_x = polygon.front().x;
    double max_x = polygon.front().x;
    double min_y = polygon.front().y;
    double max_y = polygon.front().y;
    for (const FeatureVertex& vertex : polygon) {
        min_x = std::min(min_x, vertex.x);
        max_x = std::max(max_x, vertex.x);
        min_y = std::min(min_y, vertex.y);
        max_y = std::max(max_y, vertex.y);
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
                map.setCell(row, col, state);
            }
        }
    }
}

void FeatureMapBuilder::markDisk(GridMap& map,
                                 double x,
                                 double y,
                                 double radius,
                                 CellState state) const {
    const GridCell center = map.worldToGrid(x, y);
    const int radius_cells =
        static_cast<int>(std::ceil(radius / map.resolution()));
    for (int row = center.row - radius_cells;
         row <= center.row + radius_cells;
         ++row) {
        for (int col = center.col - radius_cells;
             col <= center.col + radius_cells;
             ++col) {
            if (!map.inBounds(row, col)) {
                continue;
            }
            const auto [cell_x, cell_y] =
                map.gridToWorld(GridCell{row, col});
            const double dx = cell_x - x;
            const double dy = cell_y - y;
            if (std::sqrt(dx * dx + dy * dy) <= radius) {
                map.setCell(row, col, state);
            }
        }
    }
}

void FeatureMapBuilder::markLine(GridMap& map,
                                 const FeatureVertex& start,
                                 const FeatureVertex& end,
                                 double radius,
                                 CellState state) const {
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length = std::sqrt(dx * dx + dy * dy);
    const int samples = std::max(
        1,
        static_cast<int>(
            std::ceil(length / (map.resolution() * 0.5))));
    for (int i = 0; i <= samples; ++i) {
        const double t = static_cast<double>(i) /
                         static_cast<double>(samples);
        markDisk(map,
                 start.x + t * dx,
                 start.y + t * dy,
                 radius,
                 state);
    }
}

void FeatureMapBuilder::markOuterBorder(GridMap& map) const {
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
