#pragma once

#include <string>
#include <vector>

#include "common/Types.h"

namespace rtk_nav {

class RTKReader {
public:
    std::vector<RTKPoint> load(const std::string& csv_path) const;
};

}  // namespace rtk_nav
