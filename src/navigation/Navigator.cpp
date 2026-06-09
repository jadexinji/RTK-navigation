#include "navigation/Navigator.h"

#include <cmath>
#include <stdexcept>

namespace rtk_nav {
namespace {

constexpr double kPi = 3.14159265358979323846;

double distance2D(double ax, double ay, double bx, double by) {
    const double dx = ax - bx;
    const double dy = ay - by;
    return std::sqrt(dx * dx + dy * dy);
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

}  // namespace

Navigator::Navigator(NavigatorConfig config) : config_(config) {
    if (config_.lookahead_distance <= 0.0 || config_.speed <= 0.0 || config_.time_step <= 0.0) {
        throw std::runtime_error("Navigator configuration values must be positive");
    }
}

std::vector<VehicleState> Navigator::simulate(const std::vector<std::pair<double, double>>& path) const {
    std::vector<VehicleState> trajectory;
    if (path.empty()) {
        return trajectory;
    }

    VehicleState state;
    state.x = path.front().first;
    state.y = path.front().second;
    state.velocity = config_.speed;
    if (path.size() > 1) {
        state.yaw = std::atan2(path[1].second - path[0].second, path[1].first - path[0].first);
    }
    trajectory.push_back(state);

    if (path.size() == 1) {
        return trajectory;
    }

    std::size_t search_start = 0;
    const auto [goal_x, goal_y] = path.back();

    for (int step = 0; step < config_.max_steps; ++step) {
        if (distance2D(state.x, state.y, goal_x, goal_y) <= config_.goal_tolerance) {
            break;
        }

        const std::size_t closest_index = closestPathIndex(state, path, search_start);
        const std::size_t target_index = lookaheadPathIndex(state, path, closest_index);
        search_start = closest_index;

        const auto [target_x, target_y] = path[target_index];
        const double target_heading = std::atan2(target_y - state.y, target_x - state.x);
        const double alpha = normalizeAngle(target_heading - state.yaw);
        const double distance_to_target = std::max(config_.lookahead_distance,
                                                   distance2D(state.x, state.y, target_x, target_y));
        const double curvature = 2.0 * std::sin(alpha) / distance_to_target;

        state.yaw = normalizeAngle(state.yaw + config_.speed * curvature * config_.time_step);
        state.x += config_.speed * std::cos(state.yaw) * config_.time_step;
        state.y += config_.speed * std::sin(state.yaw) * config_.time_step;
        state.velocity = config_.speed;

        trajectory.push_back(state);
    }

    return trajectory;
}

std::size_t Navigator::closestPathIndex(const VehicleState& state,
                                        const std::vector<std::pair<double, double>>& path,
                                        std::size_t start_index) const {
    std::size_t best_index = std::min(start_index, path.size() - 1);
    double best_distance = distance2D(state.x, state.y, path[best_index].first, path[best_index].second);

    for (std::size_t i = best_index + 1; i < path.size(); ++i) {
        const double candidate_distance = distance2D(state.x, state.y, path[i].first, path[i].second);
        if (candidate_distance < best_distance) {
            best_distance = candidate_distance;
            best_index = i;
        }
    }

    return best_index;
}

std::size_t Navigator::lookaheadPathIndex(const VehicleState& state,
                                          const std::vector<std::pair<double, double>>& path,
                                          std::size_t closest_index) const {
    for (std::size_t i = closest_index; i < path.size(); ++i) {
        if (distance2D(state.x, state.y, path[i].first, path[i].second) >= config_.lookahead_distance) {
            return i;
        }
    }
    return path.size() - 1;
}

}  // namespace rtk_nav
