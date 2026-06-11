#include "dxf/DxfReader.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace rtk_nav {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kArcStepRadians = 10.0 * kPi / 180.0;

struct DxfPair {
    int code = 0;
    std::string value;
    int code_line = 0;
};

struct PolylineVertex {
    FeatureVertex point;
    double bulge = 0.0;
};

std::string trim(const std::string& value) {
    auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
        return std::isspace(ch);
    });
    auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
        return std::isspace(ch);
    }).base();
    return begin < end ? std::string(begin, end) : std::string{};
}

std::string toUpper(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return value;
}

int parseInt(const DxfPair& pair) {
    try {
        std::size_t parsed = 0;
        const int result = std::stoi(pair.value, &parsed);
        if (parsed != pair.value.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return result;
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid DXF integer at line " +
                                 std::to_string(pair.code_line + 1) +
                                 ": '" + pair.value + "'");
    }
}

double parseDouble(const DxfPair& pair) {
    try {
        std::size_t parsed = 0;
        const double result = std::stod(pair.value, &parsed);
        if (parsed != pair.value.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return result;
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid DXF number at line " +
                                 std::to_string(pair.code_line + 1) +
                                 ": '" + pair.value + "'");
    }
}

std::vector<DxfPair> readPairs(const std::string& dxf_path) {
    std::ifstream file(dxf_path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open DXF file: " + dxf_path);
    }

    char signature[22] = {};
    file.read(signature, sizeof(signature));
    const std::string header(signature, static_cast<std::size_t>(file.gcount()));
    if (header.rfind("AutoCAD Binary DXF", 0) == 0) {
        throw std::runtime_error(
            "Binary DXF is not supported by the built-in parser. "
            "Export the drawing as ASCII DXF (AutoCAD R12/2007 recommended).");
    }
    file.clear();
    file.seekg(0);

    std::vector<DxfPair> pairs;
    std::string code_line;
    std::string value_line;
    int line_number = 0;
    while (std::getline(file, code_line)) {
        ++line_number;
        if (!std::getline(file, value_line)) {
            throw std::runtime_error(
                "DXF file ends after a group-code line at line " +
                std::to_string(line_number));
        }

        const int group_code_line = line_number;
        ++line_number;
        DxfPair pair;
        pair.code_line = group_code_line;
        pair.value = trim(value_line);
        try {
            std::size_t parsed = 0;
            const std::string normalized_code = trim(code_line);
            pair.code = std::stoi(normalized_code, &parsed);
            if (parsed != normalized_code.size()) {
                throw std::invalid_argument("trailing characters");
            }
        } catch (const std::exception&) {
            throw std::runtime_error("Invalid DXF group code at line " +
                                     std::to_string(group_code_line) +
                                     ": '" + trim(code_line) + "'");
        }
        pairs.push_back(std::move(pair));
    }
    return pairs;
}

std::string valueForCode(const std::vector<DxfPair>& pairs,
                         std::size_t begin,
                         std::size_t end,
                         int code,
                         const std::string& fallback = "") {
    for (std::size_t i = begin; i < end; ++i) {
        if (pairs[i].code == code) {
            return pairs[i].value;
        }
    }
    return fallback;
}

std::optional<double> doubleForCode(const std::vector<DxfPair>& pairs,
                                    std::size_t begin,
                                    std::size_t end,
                                    int code) {
    for (std::size_t i = begin; i < end; ++i) {
        if (pairs[i].code == code) {
            return parseDouble(pairs[i]);
        }
    }
    return std::nullopt;
}

int intForCode(const std::vector<DxfPair>& pairs,
               std::size_t begin,
               std::size_t end,
               int code,
               int fallback = 0) {
    for (std::size_t i = begin; i < end; ++i) {
        if (pairs[i].code == code) {
            return parseInt(pairs[i]);
        }
    }
    return fallback;
}

std::string entityId(const std::vector<DxfPair>& pairs,
                     std::size_t begin,
                     std::size_t end,
                     const std::string& entity_type,
                     std::size_t fallback_index) {
    const std::string handle = valueForCode(pairs, begin, end, 5);
    if (!handle.empty()) {
        return entity_type + "-" + handle;
    }
    return entity_type + "-" + std::to_string(fallback_index);
}

std::vector<FeatureVertex> expandBulgePolyline(
    const std::vector<PolylineVertex>& source,
    bool closed) {
    std::vector<FeatureVertex> expanded;
    if (source.empty()) {
        return expanded;
    }

    expanded.push_back(source.front().point);
    const std::size_t segment_count =
        closed ? source.size() : source.size() - 1;
    for (std::size_t i = 0; i < segment_count; ++i) {
        const PolylineVertex& current = source[i];
        const PolylineVertex& next = source[(i + 1) % source.size()];
        const double bulge = current.bulge;

        const double dx = next.point.x - current.point.x;
        const double dy = next.point.y - current.point.y;
        const double chord = std::sqrt(dx * dx + dy * dy);
        if (std::abs(bulge) > 1e-9 && chord > 1e-9) {
            const double sweep = 4.0 * std::atan(bulge);
            const double midpoint_x = (current.point.x + next.point.x) * 0.5;
            const double midpoint_y = (current.point.y + next.point.y) * 0.5;
            const double left_x = -dy / chord;
            const double left_y = dx / chord;
            const double center_offset =
                chord * (1.0 - bulge * bulge) / (4.0 * bulge);
            const double center_x = midpoint_x + left_x * center_offset;
            const double center_y = midpoint_y + left_y * center_offset;
            const double radius =
                std::sqrt((current.point.x - center_x) *
                              (current.point.x - center_x) +
                          (current.point.y - center_y) *
                              (current.point.y - center_y));
            const double start_angle =
                std::atan2(current.point.y - center_y,
                           current.point.x - center_x);
            const int steps = std::max(
                2,
                static_cast<int>(
                    std::ceil(std::abs(sweep) / kArcStepRadians)));
            for (int step = 1; step < steps; ++step) {
                const double t =
                    static_cast<double>(step) / static_cast<double>(steps);
                const double angle = start_angle + sweep * t;
                expanded.push_back(FeatureVertex{
                    center_x + radius * std::cos(angle),
                    center_y + radius * std::sin(angle),
                    current.point.z +
                        (next.point.z - current.point.z) * t
                });
            }
        }

        if (!closed || i + 1 < segment_count) {
            expanded.push_back(next.point);
        }
    }
    return expanded;
}

MapFeature parseLine(const std::vector<DxfPair>& pairs,
                     std::size_t begin,
                     std::size_t end,
                     std::size_t entity_index) {
    const std::optional<double> x1 = doubleForCode(pairs, begin, end, 10);
    const std::optional<double> y1 = doubleForCode(pairs, begin, end, 20);
    const std::optional<double> x2 = doubleForCode(pairs, begin, end, 11);
    const std::optional<double> y2 = doubleForCode(pairs, begin, end, 21);
    if (!x1 || !y1 || !x2 || !y2) {
        throw std::runtime_error("DXF LINE entity is missing endpoint coordinates");
    }

    MapFeature feature;
    feature.id = entityId(pairs, begin, end, "LINE", entity_index);
    feature.layer = valueForCode(pairs, begin, end, 8, "0");
    feature.geometry = FeatureGeometry::Polyline;
    feature.vertices = {
        FeatureVertex{*x1, *y1, doubleForCode(pairs, begin, end, 30).value_or(0.0)},
        FeatureVertex{*x2, *y2, doubleForCode(pairs, begin, end, 31).value_or(0.0)}
    };
    return feature;
}

MapFeature parsePoint(const std::vector<DxfPair>& pairs,
                      std::size_t begin,
                      std::size_t end,
                      std::size_t entity_index) {
    const std::optional<double> x = doubleForCode(pairs, begin, end, 10);
    const std::optional<double> y = doubleForCode(pairs, begin, end, 20);
    if (!x || !y) {
        throw std::runtime_error("DXF POINT entity is missing coordinates");
    }

    MapFeature feature;
    feature.id = entityId(pairs, begin, end, "POINT", entity_index);
    feature.layer = valueForCode(pairs, begin, end, 8, "0");
    feature.geometry = FeatureGeometry::Point;
    feature.vertices.push_back(
        FeatureVertex{*x, *y, doubleForCode(pairs, begin, end, 30).value_or(0.0)});
    return feature;
}

MapFeature parseLwPolyline(const std::vector<DxfPair>& pairs,
                           std::size_t begin,
                           std::size_t end,
                           std::size_t entity_index) {
    const bool closed = (intForCode(pairs, begin, end, 70) & 1) != 0;
    const double elevation =
        doubleForCode(pairs, begin, end, 38).value_or(0.0);
    std::vector<PolylineVertex> vertices;

    for (std::size_t i = begin; i < end; ++i) {
        if (pairs[i].code != 10) {
            continue;
        }
        PolylineVertex vertex;
        vertex.point.x = parseDouble(pairs[i]);
        vertex.point.z = elevation;
        for (std::size_t j = i + 1; j < end && pairs[j].code != 10; ++j) {
            if (pairs[j].code == 20) {
                vertex.point.y = parseDouble(pairs[j]);
            } else if (pairs[j].code == 30) {
                vertex.point.z = parseDouble(pairs[j]);
            } else if (pairs[j].code == 42) {
                vertex.bulge = parseDouble(pairs[j]);
            }
        }
        vertices.push_back(vertex);
    }
    if (vertices.size() < 2) {
        throw std::runtime_error(
            "DXF LWPOLYLINE entity contains fewer than two vertices");
    }

    MapFeature feature;
    feature.id = entityId(pairs, begin, end, "LWPOLYLINE", entity_index);
    feature.layer = valueForCode(pairs, begin, end, 8, "0");
    feature.geometry =
        closed ? FeatureGeometry::Polygon : FeatureGeometry::Polyline;
    feature.closed = closed;
    feature.vertices = expandBulgePolyline(vertices, closed);
    return feature;
}

PolylineVertex parseVertex(const std::vector<DxfPair>& pairs,
                           std::size_t begin,
                           std::size_t end,
                           double default_z) {
    const std::optional<double> x = doubleForCode(pairs, begin, end, 10);
    const std::optional<double> y = doubleForCode(pairs, begin, end, 20);
    if (!x || !y) {
        throw std::runtime_error("DXF VERTEX entity is missing coordinates");
    }
    PolylineVertex vertex;
    vertex.point.x = *x;
    vertex.point.y = *y;
    vertex.point.z = doubleForCode(pairs, begin, end, 30).value_or(default_z);
    vertex.bulge = doubleForCode(pairs, begin, end, 42).value_or(0.0);
    return vertex;
}

}  // namespace

bool DxfReader::backendAvailable() {
    return true;
}

std::string DxfReader::backendStatus() {
    return "Built-in ASCII DXF parser: LINE, POINT, LWPOLYLINE, and "
           "2D/3D POLYLINE entities are supported. Binary DXF, INSERT blocks, "
           "SPLINE, ELLIPSE, and HATCH are not yet expanded.";
}

std::vector<MapFeature> DxfReader::load(const std::string& dxf_path) const {
    const std::vector<DxfPair> pairs = readPairs(dxf_path);
    std::vector<MapFeature> features;
    bool in_entities = false;
    std::size_t entity_index = 0;

    for (std::size_t i = 0; i < pairs.size();) {
        if (pairs[i].code != 0) {
            ++i;
            continue;
        }

        const std::string marker = toUpper(pairs[i].value);
        if (marker == "SECTION") {
            if (i + 1 < pairs.size() && pairs[i + 1].code == 2) {
                in_entities = toUpper(pairs[i + 1].value) == "ENTITIES";
            }
            ++i;
            continue;
        }
        if (marker == "ENDSEC") {
            in_entities = false;
            ++i;
            continue;
        }
        if (!in_entities) {
            ++i;
            continue;
        }

        const std::size_t begin = i + 1;
        std::size_t end = begin;
        while (end < pairs.size() && pairs[end].code != 0) {
            ++end;
        }

        if (marker == "LINE") {
            features.push_back(parseLine(pairs, begin, end, entity_index++));
            i = end;
            continue;
        }
        if (marker == "POINT") {
            features.push_back(parsePoint(pairs, begin, end, entity_index++));
            i = end;
            continue;
        }
        if (marker == "LWPOLYLINE") {
            features.push_back(
                parseLwPolyline(pairs, begin, end, entity_index++));
            i = end;
            continue;
        }
        if (marker == "POLYLINE") {
            const int flags = intForCode(pairs, begin, end, 70);
            const bool closed = (flags & 1) != 0;
            const bool polygon_mesh = (flags & 16) != 0 || (flags & 64) != 0;
            const double elevation =
                doubleForCode(pairs, begin, end, 30).value_or(0.0);
            const std::string layer =
                valueForCode(pairs, begin, end, 8, "0");
            const std::string id =
                entityId(pairs, begin, end, "POLYLINE", entity_index++);

            std::vector<PolylineVertex> vertices;
            std::size_t cursor = end;
            while (cursor < pairs.size() && pairs[cursor].code == 0) {
                const std::string child_type = toUpper(pairs[cursor].value);
                const std::size_t child_begin = cursor + 1;
                std::size_t child_end = child_begin;
                while (child_end < pairs.size() && pairs[child_end].code != 0) {
                    ++child_end;
                }
                if (child_type == "VERTEX" && !polygon_mesh) {
                    vertices.push_back(
                        parseVertex(pairs, child_begin, child_end, elevation));
                }
                cursor = child_end;
                if (child_type == "SEQEND") {
                    break;
                }
            }

            if (!polygon_mesh && vertices.size() >= 2) {
                MapFeature feature;
                feature.id = id;
                feature.layer = layer;
                feature.geometry =
                    closed ? FeatureGeometry::Polygon
                           : FeatureGeometry::Polyline;
                feature.closed = closed;
                feature.vertices = expandBulgePolyline(vertices, closed);
                features.push_back(std::move(feature));
            }
            i = cursor;
            continue;
        }

        i = end;
    }

    if (features.empty()) {
        throw std::runtime_error(
            "DXF file contains no supported entities in the ENTITIES section. " +
            backendStatus());
    }
    return features;
}

}  // namespace rtk_nav
