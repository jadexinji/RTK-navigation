#pragma once

#include <utility>
#include <vector>

#include "common/Types.h"
#include "map/GridMap.h"

namespace rtk_nav {

struct DWAConfig {
    double min_speed = 0.0;
    double max_speed = 1.2;
    double max_yaw_rate = 1.2;
    double max_acceleration = 1.0;
    double max_yaw_acceleration = 2.0;
    double velocity_resolution = 0.1;
    double yaw_rate_resolution = 0.05;
    double time_step = 0.1;
    double prediction_time = 2.0;
    double robot_radius = 0.2;
    double safety_margin = 0.02;
    double path_lookahead = 2.0;
    double obstacle_distance_cap = 3.0;
    double goal_tolerance = 0.4;
    int max_steps = 4000;
    double goal_direction_weight = 1.0;
    double path_following_weight = 4.0;
    double obstacle_weight = 1.5;
    double speed_weight = 0.8;
};

struct DWACandidateTrajectory {
    double velocity = 0.0;
    double angular_velocity = 0.0;
    double score = 0.0;
    double clearance = 0.0;
    bool collision_free = false;
    std::vector<VehicleState> trajectory;
};

struct DWAPlanResult {
    double velocity = 0.0;
    double angular_velocity = 0.0;
    double score = 0.0;
    std::vector<VehicleState> trajectory;
    std::vector<DWACandidateTrajectory> candidates;

    bool valid() const { return !trajectory.empty(); }
};

struct DWANavigationResult {
    std::vector<VehicleState> trajectory;
    std::vector<std::vector<VehicleState>> local_trajectories;
    std::vector<std::vector<std::vector<VehicleState>>>
        candidate_trajectories;
    bool reached_goal = false;
};

class DWAPlanner {
public:
    explicit DWAPlanner(DWAConfig config = {});

    DWAPlanResult plan(
        const GridMap& map,
        const VehicleState& current_state,
        const std::pair<double, double>& goal,
        const std::vector<std::pair<double, double>>& global_path) const;

    DWAPlanResult plan(
        const GridMap& map,
        const std::vector<std::pair<double, double>>& global_path,
        const VehicleState& current_state) const;

    DWANavigationResult navigate(
        const GridMap& map,
        const std::pair<double, double>& goal,
        const std::vector<std::pair<double, double>>& global_path) const;

    DWANavigationResult navigate(
        const GridMap& map,
        const std::vector<std::pair<double, double>>& global_path) const;

private:
    DWAPlanResult planWithDistanceMap(
        const GridMap& map,
        const std::vector<std::pair<double, double>>& global_path,
        const std::vector<double>& obstacle_distance,
        const VehicleState& current_state,
        const std::pair<double, double>& goal) const;
    std::vector<VehicleState> predictTrajectory(
        const VehicleState& current_state,
        double velocity,
        double angular_velocity) const;
    std::vector<double> buildObstacleDistanceMap(const GridMap& map) const;
    double trajectoryClearance(
        const GridMap& map,
        const std::vector<double>& obstacle_distance,
        const std::vector<VehicleState>& trajectory) const;
    double goalDirectionScore(
        const std::vector<std::pair<double, double>>& global_path,
        const VehicleState& current_state,
        const VehicleState& predicted_state,
        const std::pair<double, double>& goal) const;
    double pathFollowingScore(
        const std::vector<std::pair<double, double>>& global_path,
        const std::vector<VehicleState>& trajectory) const;
    double obstacleScore(double clearance) const;
    double speedScore(double velocity,
                      double minimum_velocity,
                      double maximum_velocity) const;

    DWAConfig config_;
};

}  // namespace rtk_nav
