#include "sensor/RTKReader.h"

#include <algorithm>
#include <cctype>
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

    if (begin >= end) {
        return "";
    }
    return std::string(begin, end);
}

std::string toLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
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

double parseDouble(const std::string& value, const std::string& field_name, int line_number) {
    try {
        size_t parsed = 0;
        const double result = std::stod(value, &parsed);
        if (parsed != value.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return result;
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid " + field_name + " at CSV line " + std::to_string(line_number) +
                                 ": '" + value + "'");
    }
}

}  // namespace

std::vector<RTKPoint> RTKReader::load(const std::string& csv_path) const {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open RTK CSV file: " + csv_path);
    }

    std::vector<RTKPoint> points;
    std::string line;
    int line_number = 0;

    while (std::getline(file, line)) {
        ++line_number;
        if (trim(line).empty()) {
            continue;
        }

        std::vector<std::string> fields = splitCsvLine(line);
        if (!fields.empty() && toLower(fields[0]) == "id") {
            continue;
        }

        if (fields.size() != 5) {
            throw std::runtime_error("Expected 5 CSV columns at line " + std::to_string(line_number) +
                                     ", got " + std::to_string(fields.size()));
        }

        RTKPoint point;
        point.id = fields[0];
        point.latitude = parseDouble(fields[1], "latitude", line_number);
        point.longitude = parseDouble(fields[2], "longitude", line_number);
        point.height = parseDouble(fields[3], "height", line_number);

        const std::string type_text = toLower(fields[4]);
        const std::optional<PointType> type = pointTypeFromString(type_text);
        if (!type.has_value()) {
            throw std::runtime_error("Unknown RTK point type at CSV line " + std::to_string(line_number) +
                                     ": '" + fields[4] + "'");
        }
        point.type = *type;

        points.push_back(point);
    }

    if (points.empty()) {
        throw std::runtime_error("RTK CSV file contains no points: " + csv_path);
    }

    return points;
}

}  // namespace rtk_nav
