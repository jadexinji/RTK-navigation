#pragma once

#include <utility>
#include <vector>

#include "common/Types.h"

namespace rtk_nav {

struct NavigatorConfig {
    double lookahead_distance = 2.0;
    double speed = 1.2;
    double time_step = 0.1;
    double goal_tolerance = 0.35;
    int max_steps = 3000;
};

class Navigator {
public:
    explicit Navigator(NavigatorConfig config = {});

    std::vector<VehicleState> simulate(const std::vector<std::pair<double, double>>& path) const;

private:
    std::size_t closestPathIndex(const VehicleState& state,
                                 const std::vector<std::pair<double, double>>& path,
                                 std::size_t start_index) const;
    std::size_t lookaheadPathIndex(const VehicleState& state,
                                   const std::vector<std::pair<double, double>>& path,
                                   std::size_t closest_index) const;

    NavigatorConfig config_;
};

}  // namespace rtk_nav
