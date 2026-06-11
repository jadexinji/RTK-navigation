#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dxf/DxfReader.h"

namespace {

using namespace rtk_nav;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

const MapFeature& findByLayerAndGeometry(
    const std::vector<MapFeature>& features,
    const std::string& layer,
    FeatureGeometry geometry,
    std::size_t occurrence = 0) {
    std::size_t found = 0;
    for (const MapFeature& feature : features) {
        if (feature.layer == layer && feature.geometry == geometry) {
            if (found == occurrence) {
                return feature;
            }
            ++found;
        }
    }
    throw std::runtime_error("Expected DXF feature was not found: " + layer);
}

}  // namespace

int main() {
    try {
        DxfReader reader;
        require(DxfReader::backendAvailable(),
                "Built-in ASCII DXF backend should be available");
        const std::vector<MapFeature> features =
            reader.load(RTK_NAV_TEST_DXF);
        require(features.size() == 7,
                "Expected seven supported DXF features");

        const MapFeature& line =
            findByLayerAndGeometry(
                features, "CENTERLINE_TEST", FeatureGeometry::Polyline);
        require(line.vertices.size() == 2,
                "LINE should contain two vertices");
        require(std::abs(line.vertices[1].x - 20.0) < 1e-9,
                "LINE endpoint X was parsed incorrectly");

        const MapFeature& point =
            findByLayerAndGeometry(features, "POINT_TEST", FeatureGeometry::Point);
        require(point.vertices.size() == 1,
                "POINT should contain one vertex");
        require(std::abs(point.vertices[0].y - 4.0) < 1e-9,
                "POINT Y coordinate was parsed incorrectly");

        const MapFeature& building =
            findByLayerAndGeometry(
                features, "BUILDING_TEST", FeatureGeometry::Polygon);
        require(building.closed,
                "Closed LWPOLYLINE was not marked as closed");
        require(building.vertices.size() == 4,
                "Building LWPOLYLINE should contain four vertices");

        const MapFeature& road_surface =
            findByLayerAndGeometry(features, "ROAD", FeatureGeometry::Polygon);
        require(road_surface.closed,
                "Road surface LWPOLYLINE was not marked as closed");
        require(road_surface.vertices.size() == 4,
                "Road surface polygon should contain four vertices");

        const MapFeature& curved_road =
            findByLayerAndGeometry(
                features, "CURVE_TEST", FeatureGeometry::Polyline);
        require(curved_road.vertices.size() > 2,
                "Bulge arc was not expanded into intermediate vertices");

        const MapFeature& wall =
            findByLayerAndGeometry(
                features, "WALL_TEST", FeatureGeometry::Polygon);
        require(wall.closed,
                "Traditional POLYLINE closed flag was not preserved");
        require(wall.vertices.size() == 3,
                "Traditional POLYLINE vertices were parsed incorrectly");

        std::cout << "DXF reader test passed: "
                  << features.size() << " features\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "DXF reader test failed: "
                  << error.what() << "\n";
        return 1;
    }
}
