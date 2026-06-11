#include "sensor/CassDatReader.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace rtk_nav {
namespace {

std::string trim(const std::string& value) {
    auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
        return std::isspace(ch);
    });
    auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
        return std::isspace(ch);
    }).base();
    return begin < end ? std::string(begin, end) : std::string{};
}

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(trim(field));
    }
    return fields;
}

double parseDouble(const std::string& value, int line_number, const std::string& field_name) {
    try {
        std::size_t parsed = 0;
        const double result = std::stod(value, &parsed);
        if (parsed != value.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return result;
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid CASS " + field_name + " at line " +
                                 std::to_string(line_number) + ": '" + value + "'");
    }
}

double median(std::vector<double> values) {
    if (values.empty()) {
        throw std::runtime_error("Cannot calculate median of empty values");
    }
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<long>(middle), values.end());
    const double upper = values[middle];
    if (values.size() % 2 != 0) {
        return upper;
    }
    std::nth_element(values.begin(), values.begin() + static_cast<long>(middle - 1), values.end());
    return (values[middle - 1] + upper) * 0.5;
}

PointType displayTypeForGroup(const std::string& group) {
    if (group == "B" || group == "R" || group == "F") {
        return PointType::Obstacle;
    }
    return PointType::Survey;
}

}  // namespace

std::vector<CassPoint> CassDatReader::load(const std::string& dat_path) const {
    std::ifstream file(dat_path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open CASS DAT file: " + dat_path);
    }

    std::vector<CassPoint> points;
    std::string line;
    int line_number = 0;
    while (std::getline(file, line)) {
        ++line_number;
        line = trim(line);
        if (line.empty()) {
            continue;
        }

        const std::vector<std::string> fields = splitCsvLine(line);
        if (fields.size() != 5) {
            throw std::runtime_error("Expected 5 CASS DAT columns at line " +
                                     std::to_string(line_number) + ", got " +
                                     std::to_string(fields.size()));
        }

        CassPoint point;
        point.id = fields[0];
        point.code = fields[1];
        point.easting = parseDouble(fields[2], line_number, "easting");
        point.northing = parseDouble(fields[3], line_number, "northing");
        point.height = parseDouble(fields[4], line_number, "height");
        points.push_back(point);
    }

    if (points.empty()) {
        throw std::runtime_error("CASS DAT file contains no points: " + dat_path);
    }
    return points;
}

std::vector<CassPoint> CassDatReader::filterSurveyArea(const std::vector<CassPoint>& points,
                                                       double max_distance_from_median) const {
    std::vector<double> eastings;
    std::vector<double> northings;
    eastings.reserve(points.size());
    northings.reserve(points.size());
    for (const CassPoint& point : points) {
        eastings.push_back(point.easting);
        northings.push_back(point.northing);
    }

    const double median_easting = median(eastings);
    const double median_northing = median(northings);

    std::vector<CassPoint> filtered;
    for (const CassPoint& point : points) {
        const double dx = point.easting - median_easting;
        const double dy = point.northing - median_northing;
        if (std::sqrt(dx * dx + dy * dy) <= max_distance_from_median) {
            filtered.push_back(point);
        }
    }
    return filtered;
}

std::vector<CassLocalPoint> CassDatReader::toLocal(const std::vector<CassPoint>& points,
                                                   double origin_easting,
                                                   double origin_northing) const {
    std::vector<CassLocalPoint> local_points;
    local_points.reserve(points.size());
    for (const CassPoint& point : points) {
        const std::string group = pointGroup(point.id);
        LocalPoint local;
        local.id = point.id;
        local.x = point.easting - origin_easting;
        local.y = point.northing - origin_northing;
        local.z = point.height;
        local.type = displayTypeForGroup(group);
        local_points.push_back(CassLocalPoint{local, group});
    }
    return local_points;
}

std::string CassDatReader::pointGroup(const std::string& point_id) {
    std::string group;
    for (unsigned char ch : point_id) {
        if (std::isalpha(ch)) {
            group.push_back(static_cast<char>(std::toupper(ch)));
        } else if (!group.empty()) {
            break;
        }
    }
    return group;
}

}  // namespace rtk_nav
