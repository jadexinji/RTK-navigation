#include "config/LayerConfig.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

#include <opencv2/core.hpp>

namespace rtk_nav {
namespace {

std::string toUpper(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return value;
}

std::string readRequiredString(const cv::FileNode& node,
                               const std::string& key,
                               const std::string& context) {
    const cv::FileNode value = node[key];
    if (value.empty() || !value.isString()) {
        throw std::runtime_error("Missing string '" + key + "' in " + context);
    }
    return static_cast<std::string>(value);
}

FeatureSemantic parseSemantic(const std::string& text) {
    const std::string value = toUpper(text);
    if (value == "ROAD") {
        return FeatureSemantic::Road;
    }
    if (value == "BUILDING") {
        return FeatureSemantic::Building;
    }
    if (value == "WATER") {
        return FeatureSemantic::Water;
    }
    if (value == "VEGETATION") {
        return FeatureSemantic::Vegetation;
    }
    if (value == "WALL") {
        return FeatureSemantic::Wall;
    }
    if (value == "BOUNDARY") {
        return FeatureSemantic::Boundary;
    }
    if (value == "UNKNOWN") {
        return FeatureSemantic::Unknown;
    }
    throw std::runtime_error("Unknown feature semantic: " + text);
}

OccupancyEffect parseOccupancy(const std::string& text) {
    const std::string value = toUpper(text);
    if (value == "FREE") {
        return OccupancyEffect::Free;
    }
    if (value == "OCCUPIED") {
        return OccupancyEffect::Occupied;
    }
    if (value == "IGNORE") {
        return OccupancyEffect::Ignore;
    }
    throw std::runtime_error("Unknown occupancy effect: " + text);
}

CellState parseDefaultState(const std::string& text) {
    const std::string value = toUpper(text);
    if (value == "FREE") {
        return CellState::Free;
    }
    if (value == "OCCUPIED") {
        return CellState::Occupied;
    }
    if (value == "UNKNOWN") {
        return CellState::Unknown;
    }
    throw std::runtime_error("Unknown map default_state: " + text);
}

LayerMatchMode parseMatchMode(const std::string& text) {
    const std::string value = toUpper(text);
    if (value == "EXACT") {
        return LayerMatchMode::Exact;
    }
    if (value == "CONTAINS") {
        return LayerMatchMode::Contains;
    }
    throw std::runtime_error("Unknown layer match mode: " + text);
}

bool matches(const std::string& layer, const LayerRule& rule) {
    const std::string normalized_layer = toUpper(layer);
    for (const std::string& alias : rule.layers) {
        const std::string normalized_alias = toUpper(alias);
        if (rule.match_mode == LayerMatchMode::Exact &&
            normalized_layer == normalized_alias) {
            return true;
        }
        if (rule.match_mode == LayerMatchMode::Contains &&
            normalized_layer.find(normalized_alias) != std::string::npos) {
            return true;
        }
    }
    return false;
}

}  // namespace

LayerConfig LayerConfig::load(const std::string& yaml_path) {
    cv::FileStorage storage(yaml_path, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        throw std::runtime_error("Cannot open DXF layer config: " + yaml_path);
    }

    LayerConfig config;
    const cv::FileNode map = storage["map"];
    if (map.empty() || !map.isMap()) {
        throw std::runtime_error("DXF layer config is missing the 'map' section");
    }

    if (!map["resolution"].empty()) {
        map["resolution"] >> config.map_settings_.resolution;
    }
    if (!map["padding"].empty()) {
        map["padding"] >> config.map_settings_.padding;
    }
    if (!map["endpoint_clearance"].empty()) {
        map["endpoint_clearance"] >> config.map_settings_.endpoint_clearance;
    }
    if (!map["require_endpoints_on_free"].empty()) {
        int require_endpoints = 0;
        map["require_endpoints_on_free"] >> require_endpoints;
        config.map_settings_.require_endpoints_on_free =
            require_endpoints != 0;
    }
    if (!map["clearance_cost_radius"].empty()) {
        map["clearance_cost_radius"] >>
            config.map_settings_.clearance_cost_radius;
    }
    if (!map["clearance_cost_weight"].empty()) {
        map["clearance_cost_weight"] >>
            config.map_settings_.clearance_cost_weight;
    }
    config.map_settings_.default_state =
        parseDefaultState(readRequiredString(map, "default_state", "map section"));

    if (config.map_settings_.resolution <= 0.0 ||
        config.map_settings_.padding < 0.0 ||
        config.map_settings_.endpoint_clearance < 0.0 ||
        config.map_settings_.clearance_cost_radius < 0.0 ||
        config.map_settings_.clearance_cost_weight < 0.0) {
        throw std::runtime_error("DXF map settings contain invalid negative values");
    }

    const cv::FileNode rules = storage["rules"];
    if (rules.empty() || !rules.isSeq()) {
        throw std::runtime_error("DXF layer config is missing the 'rules' sequence");
    }

    int rule_index = 0;
    for (const cv::FileNode& node : rules) {
        const std::string context = "layer rule " + std::to_string(rule_index++);
        LayerRule rule;

        const cv::FileNode layers = node["layers"];
        if (layers.empty() || !layers.isSeq()) {
            throw std::runtime_error(context + " must contain a 'layers' sequence");
        }
        for (const cv::FileNode& layer : layers) {
            rule.layers.push_back(static_cast<std::string>(layer));
        }
        if (rule.layers.empty()) {
            throw std::runtime_error(context + " contains no layer aliases");
        }

        rule.match_mode = parseMatchMode(readRequiredString(node, "match", context));
        rule.style.semantic = parseSemantic(readRequiredString(node, "semantic", context));
        rule.style.occupancy = parseOccupancy(readRequiredString(node, "occupancy", context));

        if (!node["force_closed"].empty()) {
            int force_closed = 0;
            node["force_closed"] >> force_closed;
            rule.style.force_closed = force_closed != 0;
        }
        if (!node["inflation"].empty()) {
            node["inflation"] >> rule.style.inflation;
        }
        if (!node["line_width"].empty()) {
            node["line_width"] >> rule.style.line_width;
        }
        if (rule.style.inflation < 0.0 || rule.style.line_width < 0.0) {
            throw std::runtime_error(context + " contains a negative width or inflation");
        }

        config.rules_.push_back(std::move(rule));
    }
    return config;
}

FeatureStyle LayerConfig::classify(const MapFeature& feature) const {
    for (const LayerRule& rule : rules_) {
        if (matches(feature.layer, rule)) {
            return rule.style;
        }
    }
    return FeatureStyle{};
}

}  // namespace rtk_nav
