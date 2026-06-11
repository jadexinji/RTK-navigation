#pragma once

#include <optional>
#include <string>

namespace rtk_nav {

enum class PointType {
    Road,
    Obstacle,
    Boundary,
    Survey,
    Start,
    Goal
};

struct RTKPoint {
    std::string id;
    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    PointType type = PointType::Road;
};

struct LocalPoint {
    std::string id;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    PointType type = PointType::Road;
};

struct GridCell {
    int row = 0;
    int col = 0;

    bool operator==(const GridCell& other) const {
        return row == other.row && col == other.col;
    }

    bool operator!=(const GridCell& other) const {
        return !(*this == other);
    }
};

struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double velocity = 0.0;
};

inline std::string pointTypeToString(PointType type) {
    switch (type) {
        case PointType::Road:
            return "road";
        case PointType::Obstacle:
            return "obstacle";
        case PointType::Boundary:
            return "boundary";
        case PointType::Survey:
            return "survey";
        case PointType::Start:
            return "start";
        case PointType::Goal:
            return "goal";
    }
    return "unknown";
}

inline std::optional<PointType> pointTypeFromString(const std::string& text) {
    if (text == "road") {
        return PointType::Road;
    }
    if (text == "obstacle") {
        return PointType::Obstacle;
    }
    if (text == "boundary") {
        return PointType::Boundary;
    }
    if (text == "survey") {
        return PointType::Survey;
    }
    if (text == "start") {
        return PointType::Start;
    }
    if (text == "goal") {
        return PointType::Goal;
    }
    return std::nullopt;
}

}  // namespace rtk_nav
