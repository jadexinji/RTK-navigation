#pragma once

#include <string>
#include <vector>

namespace rtk_nav {

enum class FeatureGeometry {
    Point,
    Polyline,
    Polygon
};

enum class FeatureSemantic {
    Unknown,
    Road,
    Building,
    Water,
    Vegetation,
    Wall,
    Boundary
};

enum class OccupancyEffect {
    Ignore,
    Free,
    Occupied
};

struct FeatureVertex {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct MapFeature {
    std::string id;
    std::string layer;
    FeatureGeometry geometry = FeatureGeometry::Polyline;
    std::vector<FeatureVertex> vertices;
    bool closed = false;
};

struct FeatureStyle {
    FeatureSemantic semantic = FeatureSemantic::Unknown;
    OccupancyEffect occupancy = OccupancyEffect::Ignore;
    bool force_closed = false;
    double inflation = 0.0;
    double line_width = 0.3;
};

struct ClassifiedFeature {
    MapFeature feature;
    FeatureStyle style;
};

}  // namespace rtk_nav
