#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "map/GridMap.h"
#include "planner/AStar.h"
#include "planner/DWAPlanner.h"

namespace {

using namespace rtk_nav;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::vector<std::pair<double, double>> pathToWorld(
    const GridMap& map,
    const std::vector<GridCell>& path) {
    std::vector<std::pair<double, double>> world_path;
    world_path.reserve(path.size());
    for (const GridCell& cell : path) {
        world_path.push_back(map.gridToWorld(cell));
    }
    return world_path;
}

bool footprintIsCollisionFree(const GridMap& map,
                              const VehicleState& state,
                              double robot_radius) {
    const double half_cell = map.resolution() * 0.5;
    for (int row = 0; row < map.height(); ++row) {
        for (int col = 0; col < map.width(); ++col) {
            if (map.isFree(row, col)) {
                continue;
            }
            const auto [obstacle_x, obstacle_y] =
                map.gridToWorld(GridCell{row, col});
            const double dx = std::max(
                std::abs(state.x - obstacle_x) - half_cell,
                0.0);
            const double dy = std::max(
                std::abs(state.y - obstacle_y) - half_cell,
                0.0);
            if (std::sqrt(dx * dx + dy * dy) <= robot_radius) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace

int main() {
    try {
        GridMap map(45, 25, 0.5, 0.0, 0.0, CellState::Free);
        for (int row = 0; row < map.height(); ++row) {
            map.setCell(row, 0, CellState::Occupied);
            map.setCell(row,
                        map.width() - 1,
                        CellState::Occupied);
        }
        for (int col = 0; col < map.width(); ++col) {
            map.setCell(0, col, CellState::Occupied);
            map.setCell(map.height() - 1,
                        col,
                        CellState::Occupied);
        }
        for (int row = 8; row <= 16; ++row) {
            for (int col = 20; col <= 23; ++col) {
                map.setCell(row, col, CellState::Occupied);
            }
        }

        const GridCell start{12, 3};
        const GridCell goal{12, 40};
        AStarConfig astar_config;
        astar_config.clearance_cost_radius = 1.5;
        astar_config.clearance_cost_weight = 4.0;
        const std::vector<GridCell> global_path =
            AStar{astar_config}.plan(map, start, goal);
        require(!global_path.empty(),
                "A* did not produce a global path");

        DWAConfig dwa_config;
        dwa_config.max_steps = 2500;
        DWAPlanner planner(dwa_config);
        const std::vector<std::pair<double, double>> world_path =
            pathToWorld(map, global_path);

        VehicleState initial_state;
        initial_state.x = world_path.front().first;
        initial_state.y = world_path.front().second;
        initial_state.yaw = std::atan2(
            world_path[1].second - initial_state.y,
            world_path[1].first - initial_state.x);
        const std::pair<double, double> goal_point =
            map.gridToWorld(goal);
        const DWAPlanResult local_plan =
            planner.plan(map,
                         initial_state,
                         goal_point,
                         world_path);
        require(local_plan.valid() &&
                    local_plan.trajectory.size() > 1,
                "DWA single-cycle plan did not output a local trajectory");
        require(local_plan.candidates.size() > 1,
                "DWA did not retain sampled candidate trajectories");

        bool found_best_candidate = false;
        for (const DWACandidateTrajectory& candidate :
             local_plan.candidates) {
            require(!candidate.trajectory.empty(),
                    "DWA candidate trajectory is empty");
            if (!candidate.collision_free) {
                continue;
            }
            require(local_plan.score >= candidate.score - 1e-9,
                    "DWA did not select the highest-scoring trajectory");
            if (std::abs(candidate.velocity -
                         local_plan.velocity) <= 1e-9 &&
                std::abs(candidate.angular_velocity -
                         local_plan.angular_velocity) <= 1e-9 &&
                std::abs(candidate.score -
                         local_plan.score) <= 1e-9) {
                found_best_candidate = true;
            }
        }
        require(found_best_candidate,
                "DWA best trajectory is missing from candidates");

        const DWANavigationResult navigation =
            planner.navigate(map, goal_point, world_path);

        require(navigation.reached_goal,
                "DWA did not reach the A* goal");
        require(navigation.trajectory.size() > 1,
                "DWA did not output a robot trajectory");
        require(!navigation.local_trajectories.empty(),
                "DWA did not output local trajectories");
        require(navigation.local_trajectories.front().size() > 1,
                "DWA local prediction is empty");
        require(navigation.candidate_trajectories.size() ==
                    navigation.local_trajectories.size(),
                "DWA candidate sets do not match planning cycles");
        require(!navigation.candidate_trajectories.front().empty(),
                "DWA navigation did not retain sampled candidates");

        bool used_angular_velocity = false;
        for (const VehicleState& state : navigation.trajectory) {
            require(footprintIsCollisionFree(
                        map, state, dwa_config.robot_radius),
                    "DWA robot footprint touched an occupied cell");
            require(state.velocity >=
                        dwa_config.min_speed - 1e-9 &&
                        state.velocity <=
                        dwa_config.max_speed + 1e-9,
                    "DWA velocity sample exceeded limits");
            require(std::abs(state.angular_velocity) <=
                        dwa_config.max_yaw_rate + 1e-9,
                    "DWA yaw-rate sample exceeded limits");
            used_angular_velocity =
                used_angular_velocity ||
                std::abs(state.angular_velocity) > 1e-3;
        }
        require(used_angular_velocity,
                "DWA never sampled a turning command");

        std::cout << "DWA planner test passed: "
                  << global_path.size() << " global cells, "
                  << navigation.local_trajectories.size()
                  << " local plans, "
                  << navigation.trajectory.size()
                  << " robot states\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "DWA planner test failed: "
                  << error.what() << "\n";
        return 1;
    }
}
