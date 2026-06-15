#include "planner/DWAPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace rtk_nav {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

struct DistanceNode {
    GridCell cell;
    double distance = 0.0;
};

struct DistanceNodeCompare {
    bool operator()(const DistanceNode& lhs,
                    const DistanceNode& rhs) const {
        return lhs.distance > rhs.distance;
    }
};

double distance2D(double ax, double ay, double bx, double by) {
    const double dx = ax - bx;
    const double dy = ay - by;
    return std::sqrt(dx * dx + dy * dy);
}

double distanceToSegment(double x,
                         double y,
                         double start_x,
                         double start_y,
                         double end_x,
                         double end_y) {
    const double segment_x = end_x - start_x;
    const double segment_y = end_y - start_y;
    const double length_squared =
        segment_x * segment_x + segment_y * segment_y;
    if (length_squared <= kEpsilon) {
        return distance2D(x, y, start_x, start_y);
    }

    const double projection = std::clamp(
        ((x - start_x) * segment_x +
         (y - start_y) * segment_y) /
            length_squared,
        0.0,
        1.0);
    return distance2D(x,
                      y,
                      start_x + projection * segment_x,
                      start_y + projection * segment_y);
}

double normalizeAngle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

std::vector<double> sampleRange(double minimum,
                                double maximum,
                                double resolution) {
    std::vector<double> samples;
    if (minimum > maximum + kEpsilon) {
        return samples;
    }

    for (double value = minimum;
         value <= maximum + kEpsilon;
         value += resolution) {
        samples.push_back(std::min(value, maximum));
    }
    if (samples.empty() ||
        std::abs(samples.back() - maximum) > kEpsilon) {
        samples.push_back(maximum);
    }
    if (minimum < 0.0 && maximum > 0.0 &&
        std::none_of(samples.begin(), samples.end(), [](double value) {
            return std::abs(value) <= kEpsilon;
        })) {
        samples.push_back(0.0);
        std::sort(samples.begin(), samples.end());
    }
    return samples;
}

std::size_t closestPathIndex(
    const std::vector<std::pair<double, double>>& path,
    double x,
    double y) {
    std::size_t best_index = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.size(); ++i) {
        const double candidate =
            distance2D(x, y, path[i].first, path[i].second);
        if (candidate < best_distance) {
            best_distance = candidate;
            best_index = i;
        }
    }
    return best_index;
}

std::size_t lookaheadPathIndex(
    const std::vector<std::pair<double, double>>& path,
    std::size_t start_index,
    double lookahead_distance) {
    double accumulated = 0.0;
    for (std::size_t i = start_index + 1; i < path.size(); ++i) {
        accumulated += distance2D(path[i - 1].first,
                                  path[i - 1].second,
                                  path[i].first,
                                  path[i].second);
        if (accumulated >= lookahead_distance) {
            return i;
        }
    }
    return path.size() - 1;
}

}  // namespace

DWAPlanner::DWAPlanner(DWAConfig config) : config_(config) {
    if (config_.min_speed < 0.0 ||
        config_.max_speed <= config_.min_speed ||
        config_.max_yaw_rate <= 0.0 ||
        config_.max_acceleration <= 0.0 ||
        config_.max_yaw_acceleration <= 0.0 ||
        config_.velocity_resolution <= 0.0 ||
        config_.yaw_rate_resolution <= 0.0 ||
        config_.time_step <= 0.0 ||
        config_.prediction_time < config_.time_step ||
        config_.robot_radius < 0.0 ||
        config_.safety_margin < 0.0 ||
        config_.path_lookahead <= 0.0 ||
        config_.obstacle_distance_cap <= 0.0 ||
        config_.goal_tolerance <= 0.0 ||
        config_.max_steps <= 0 ||
        config_.goal_direction_weight < 0.0 ||
        config_.path_following_weight < 0.0 ||
        config_.obstacle_weight < 0.0 ||
        config_.speed_weight < 0.0) {
        throw std::runtime_error("Invalid DWA planner configuration");
    }
}

DWAPlanResult DWAPlanner::plan(
    const GridMap& map,
    const VehicleState& current_state,
    const std::pair<double, double>& goal,
    const std::vector<std::pair<double, double>>& global_path) const {
    return planWithDistanceMap(map,
                               global_path,
                               buildObstacleDistanceMap(map),
                               current_state,
                               goal);
}

DWAPlanResult DWAPlanner::plan(
    const GridMap& map,
    const std::vector<std::pair<double, double>>& global_path,
    const VehicleState& current_state) const {
    if (global_path.empty()) {
        return {};
    }
    return plan(map,
                current_state,
                global_path.back(),
                global_path);
}

DWANavigationResult DWAPlanner::navigate(
    const GridMap& map,
    const std::pair<double, double>& goal,
    const std::vector<std::pair<double, double>>& global_path) const {
    DWANavigationResult result;
    if (global_path.empty()) {
        return result;
    }

    VehicleState state;
    state.x = global_path.front().first;
    state.y = global_path.front().second;
    if (global_path.size() > 1) {
        state.yaw = std::atan2(global_path[1].second - state.y,
                               global_path[1].first - state.x);
    }
    result.trajectory.push_back(state);

    const std::vector<double> obstacle_distance =
        buildObstacleDistanceMap(map);

    for (int step = 0; step < config_.max_steps; ++step) {
        if (distance2D(state.x,
                       state.y,
                       goal.first,
                       goal.second) <= config_.goal_tolerance) {
            result.reached_goal = true;
            break;
        }

        const DWAPlanResult local_plan =
            planWithDistanceMap(map,
                                global_path,
                                obstacle_distance,
                                state,
                                goal);
        if (!local_plan.valid() || local_plan.trajectory.size() < 2) {
            break;
        }

        std::vector<std::vector<VehicleState>> candidates;
        candidates.reserve(local_plan.candidates.size());
        for (const DWACandidateTrajectory& candidate :
             local_plan.candidates) {
            candidates.push_back(candidate.trajectory);
        }
        result.candidate_trajectories.push_back(
            std::move(candidates));
        result.local_trajectories.push_back(local_plan.trajectory);
        state = local_plan.trajectory[1];
        result.trajectory.push_back(state);
    }

    return result;
}

DWANavigationResult DWAPlanner::navigate(
    const GridMap& map,
    const std::vector<std::pair<double, double>>& global_path) const {
    if (global_path.empty()) {
        return {};
    }
    return navigate(map, global_path.back(), global_path);
}

DWAPlanResult DWAPlanner::planWithDistanceMap(
    const GridMap& map,
    const std::vector<std::pair<double, double>>& global_path,
    const std::vector<double>& obstacle_distance,
    const VehicleState& current_state,
    const std::pair<double, double>& goal) const {
    DWAPlanResult best;
    if (global_path.empty()) {
        return best;
    }

    const double velocity_delta =
        config_.max_acceleration * config_.time_step;
    const double yaw_rate_delta =
        config_.max_yaw_acceleration * config_.time_step;
    const double minimum_velocity =
        std::max(config_.min_speed,
                 current_state.velocity - velocity_delta);
    const double maximum_velocity =
        std::min(config_.max_speed,
                 current_state.velocity + velocity_delta);
    const double minimum_yaw_rate =
        std::max(-config_.max_yaw_rate,
                 current_state.angular_velocity - yaw_rate_delta);
    const double maximum_yaw_rate =
        std::min(config_.max_yaw_rate,
                 current_state.angular_velocity + yaw_rate_delta);

    const std::vector<double> velocity_samples =
        sampleRange(minimum_velocity,
                    maximum_velocity,
                    config_.velocity_resolution);
    const std::vector<double> yaw_rate_samples =
        sampleRange(minimum_yaw_rate,
                    maximum_yaw_rate,
                    config_.yaw_rate_resolution);

    double best_score = -std::numeric_limits<double>::infinity();
    for (double velocity : velocity_samples) {
        for (double angular_velocity : yaw_rate_samples) {
            std::vector<VehicleState> trajectory =
                predictTrajectory(current_state,
                                  velocity,
                                  angular_velocity);
            const double clearance =
                trajectoryClearance(map,
                                    obstacle_distance,
                                    trajectory);
            const double stopping_distance =
                velocity * velocity /
                (2.0 * config_.max_acceleration);
            const bool collision_free =
                clearance > stopping_distance +
                                config_.safety_margin;

            DWACandidateTrajectory candidate;
            candidate.velocity = velocity;
            candidate.angular_velocity = angular_velocity;
            candidate.clearance = clearance;
            candidate.collision_free = collision_free;
            candidate.trajectory = trajectory;
            if (!collision_free) {
                candidate.score =
                    -std::numeric_limits<double>::infinity();
                best.candidates.push_back(std::move(candidate));
                continue;
            }

            const double score =
                config_.goal_direction_weight *
                    goalDirectionScore(global_path,
                                       current_state,
                                       trajectory.back(),
                                       goal) +
                config_.path_following_weight *
                    pathFollowingScore(global_path, trajectory) +
                config_.obstacle_weight * obstacleScore(clearance) +
                config_.speed_weight *
                    speedScore(velocity,
                               minimum_velocity,
                               maximum_velocity);

            candidate.score = score;
            best.candidates.push_back(std::move(candidate));
            if (score > best_score + kEpsilon ||
                (std::abs(score - best_score) <= kEpsilon &&
                 velocity > best.velocity)) {
                best_score = score;
                best.velocity = velocity;
                best.angular_velocity = angular_velocity;
                best.score = score;
                best.trajectory = std::move(trajectory);
            }
        }
    }
    return best;
}

std::vector<VehicleState> DWAPlanner::predictTrajectory(
    const VehicleState& current_state,
    double velocity,
    double angular_velocity) const {
    std::vector<VehicleState> trajectory;
    trajectory.push_back(current_state);

    VehicleState state = current_state;
    const int prediction_steps = static_cast<int>(
        std::ceil(config_.prediction_time / config_.time_step));
    for (int step = 0; step < prediction_steps; ++step) {
        state.x += velocity * std::cos(state.yaw) *
                   config_.time_step;
        state.y += velocity * std::sin(state.yaw) *
                   config_.time_step;
        state.yaw = normalizeAngle(
            state.yaw + angular_velocity * config_.time_step);
        state.velocity = velocity;
        state.angular_velocity = angular_velocity;
        trajectory.push_back(state);
    }
    return trajectory;
}

std::vector<double> DWAPlanner::buildObstacleDistanceMap(
    const GridMap& map) const {
    const int total_cells = map.width() * map.height();
    std::vector<double> distance(
        static_cast<std::size_t>(total_cells),
        std::numeric_limits<double>::infinity());
    std::priority_queue<DistanceNode,
                        std::vector<DistanceNode>,
                        DistanceNodeCompare>
        open;

    for (int row = 0; row < map.height(); ++row) {
        for (int col = 0; col < map.width(); ++col) {
            const GridCell cell{row, col};
            if (!map.isFree(cell)) {
                const int index = map.toIndex(cell);
                distance[static_cast<std::size_t>(index)] = 0.0;
                open.push(DistanceNode{cell, 0.0});
            }
        }
    }

    const std::vector<GridCell> neighbors = {
        GridCell{-1, 0}, GridCell{1, 0},  GridCell{0, -1},
        GridCell{0, 1},  GridCell{-1, -1}, GridCell{-1, 1},
        GridCell{1, -1}, GridCell{1, 1}
    };
    while (!open.empty()) {
        const DistanceNode current = open.top();
        open.pop();
        const int current_index = map.toIndex(current.cell);
        if (current.distance >
            distance[static_cast<std::size_t>(current_index)]) {
            continue;
        }

        for (const GridCell& offset : neighbors) {
            const GridCell next{
                current.cell.row + offset.row,
                current.cell.col + offset.col
            };
            if (!map.inBounds(next)) {
                continue;
            }
            const double step =
                (offset.row != 0 && offset.col != 0)
                    ? std::sqrt(2.0) * map.resolution()
                    : map.resolution();
            const double candidate = current.distance + step;
            const int next_index = map.toIndex(next);
            if (candidate <
                distance[static_cast<std::size_t>(next_index)]) {
                distance[static_cast<std::size_t>(next_index)] =
                    candidate;
                open.push(DistanceNode{next, candidate});
            }
        }
    }
    return distance;
}

double DWAPlanner::trajectoryClearance(
    const GridMap& map,
    const std::vector<double>& obstacle_distance,
    const std::vector<VehicleState>& trajectory) const {
    double minimum_clearance =
        std::numeric_limits<double>::infinity();
    const double half_cell = map.resolution() * 0.5;
    const double cell_diagonal =
        std::sqrt(2.0) * map.resolution();
    const double search_distance =
        config_.obstacle_distance_cap +
        config_.robot_radius + cell_diagonal;
    const int search_cells = static_cast<int>(
        std::ceil(search_distance / map.resolution()));
    const double map_max_x =
        map.originX() +
        static_cast<double>(map.width()) * map.resolution();
    const double map_max_y =
        map.originY() +
        static_cast<double>(map.height()) * map.resolution();

    for (const VehicleState& state : trajectory) {
        const GridCell cell = map.worldToGrid(state.x, state.y);
        if (!map.inBounds(cell)) {
            return -std::numeric_limits<double>::infinity();
        }
        const double boundary_clearance =
            std::min({state.x - map.originX(),
                      map_max_x - state.x,
                      state.y - map.originY(),
                      map_max_y - state.y}) -
            config_.robot_radius;
        minimum_clearance =
            std::min(minimum_clearance, boundary_clearance);
        if (minimum_clearance <= 0.0) {
            return minimum_clearance;
        }
        const double center_distance =
            obstacle_distance[static_cast<std::size_t>(
                map.toIndex(cell))];
        const double lower_bound =
            center_distance - cell_diagonal -
            config_.robot_radius;
        if (lower_bound >= config_.obstacle_distance_cap) {
            minimum_clearance = std::min(
                minimum_clearance,
                config_.obstacle_distance_cap);
            continue;
        }

        double state_clearance =
            config_.obstacle_distance_cap;
        for (int row = cell.row - search_cells;
             row <= cell.row + search_cells;
             ++row) {
            for (int col = cell.col - search_cells;
                 col <= cell.col + search_cells;
                 ++col) {
                if (!map.inBounds(row, col) ||
                    map.isFree(row, col)) {
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
                const double clearance =
                    std::sqrt(dx * dx + dy * dy) -
                    config_.robot_radius;
                state_clearance =
                    std::min(state_clearance, clearance);
            }
        }
        minimum_clearance =
            std::min(minimum_clearance, state_clearance);
        if (minimum_clearance <= 0.0) {
            return minimum_clearance;
        }
    }
    return minimum_clearance;
}

double DWAPlanner::goalDirectionScore(
    const std::vector<std::pair<double, double>>& global_path,
    const VehicleState& current_state,
    const VehicleState& predicted_state,
    const std::pair<double, double>& goal) const {
    const std::size_t closest_index =
        closestPathIndex(global_path,
                         current_state.x,
                         current_state.y);
    const std::size_t target_index =
        lookaheadPathIndex(global_path,
                           closest_index,
                           config_.path_lookahead);
    double target_heading = predicted_state.yaw;
    const double goal_distance = distance2D(predicted_state.x,
                                            predicted_state.y,
                                            goal.first,
                                            goal.second);
    if (target_index == global_path.size() - 1 ||
        goal_distance <= config_.path_lookahead) {
        target_heading = std::atan2(goal.second - predicted_state.y,
                                    goal.first - predicted_state.x);
    } else if (target_index > 0) {
        target_heading = std::atan2(
            global_path[target_index].second -
                global_path[target_index - 1].second,
            global_path[target_index].first -
                global_path[target_index - 1].first);
    } else {
        target_heading = std::atan2(
            global_path[target_index].second -
                predicted_state.y,
            global_path[target_index].first -
                predicted_state.x);
    }
    const double normalized_error =
        std::abs(normalizeAngle(
            target_heading - predicted_state.yaw)) /
        kPi;
    return 1.0 - normalized_error;
}

double DWAPlanner::pathFollowingScore(
    const std::vector<std::pair<double, double>>& global_path,
    const std::vector<VehicleState>& trajectory) const {
    double total_distance = 0.0;
    for (const VehicleState& state : trajectory) {
        double minimum_distance =
            std::numeric_limits<double>::infinity();
        if (global_path.size() == 1) {
            minimum_distance =
                distance2D(state.x,
                           state.y,
                           global_path.front().first,
                           global_path.front().second);
        }
        for (std::size_t i = 1; i < global_path.size(); ++i) {
            minimum_distance = std::min(
                minimum_distance,
                distanceToSegment(state.x,
                                  state.y,
                                  global_path[i - 1].first,
                                  global_path[i - 1].second,
                                  global_path[i].first,
                                  global_path[i].second));
        }
        total_distance += minimum_distance;
    }
    const double average_distance =
        total_distance / static_cast<double>(trajectory.size());
    const double normalized_distance =
        std::min(1.0,
                 average_distance / config_.path_lookahead);
    return 1.0 - normalized_distance;
}

double DWAPlanner::obstacleScore(double clearance) const {
    if (!std::isfinite(clearance) ||
        clearance >= config_.obstacle_distance_cap) {
        return 1.0;
    }
    return std::clamp(
        clearance /
            config_.obstacle_distance_cap,
        0.0,
        1.0);
}

double DWAPlanner::speedScore(double velocity,
                              double minimum_velocity,
                              double maximum_velocity) const {
    const double window_size =
        maximum_velocity - minimum_velocity;
    if (window_size <= kEpsilon) {
        return 1.0;
    }
    return (velocity - minimum_velocity) / window_size;
}

}  // namespace rtk_nav
