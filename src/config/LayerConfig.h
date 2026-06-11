#pragma once

#include <string>
#include <vector>

#include "map/GridMap.h"
#include "map/MapFeature.h"

namespace rtk_nav {

enum class LayerMatchMode {
    Exact,
    Contains
};

struct LayerRule {
    std::vector<std::string> layers;
    LayerMatchMode match_mode = LayerMatchMode::Exact;
    FeatureStyle style;
};

struct FeatureMapSettings {
    double resolution = 0.5;
    double padding = 5.0;
    CellState default_state = CellState::Free;
    double endpoint_clearance = 1.0;
    bool require_endpoints_on_free = false;
    double clearance_cost_radius = 0.0;
    double clearance_cost_weight = 0.0;
};

class LayerConfig {
public:
    static LayerConfig load(const std::string& yaml_path);

    const FeatureMapSettings& mapSettings() const { return map_settings_; }
    const std::vector<LayerRule>& rules() const { return rules_; }

    FeatureStyle classify(const MapFeature& feature) const;

private:
    FeatureMapSettings map_settings_;
    std::vector<LayerRule> rules_;
};

}  // namespace rtk_nav
