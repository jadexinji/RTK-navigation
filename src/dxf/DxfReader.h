#pragma once

#include <string>
#include <vector>

#include "map/MapFeature.h"

namespace rtk_nav {

class DxfReader {
public:
    static bool backendAvailable();
    static std::string backendStatus();

    std::vector<MapFeature> load(const std::string& dxf_path) const;
};

}  // namespace rtk_nav
