#include "visualization/Viewer.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace rtk_nav {
namespace {

cv::Scalar cellColor(CellState state) {
    switch (state) {
        case CellState::Free:
            return cv::Scalar(242, 242, 242);
        case CellState::Occupied:
            return cv::Scalar(55, 55, 55);
        case CellState::Unknown:
            return cv::Scalar(145, 145, 145);
    }
    return cv::Scalar(80, 80, 80);
}

cv::Scalar pointColor(PointType type) {
    switch (type) {
        case PointType::Road:
            return cv::Scalar(220, 130, 40);
        case PointType::Obstacle:
            return cv::Scalar(40, 40, 230);
        case PointType::Boundary:
            return cv::Scalar(20, 20, 20);
        case PointType::Start:
            return cv::Scalar(70, 190, 70);
        case PointType::Goal:
            return cv::Scalar(210, 60, 210);
    }
    return cv::Scalar(255, 255, 255);
}

int positiveMin(int a, int b) {
    if (a <= 0) {
        return b;
    }
    if (b <= 0) {
        return a;
    }
    return std::min(a, b);
}

}  // namespace

Viewer::Viewer(ViewerConfig config) : config_(std::move(config)) {
    if (config_.cell_pixels <= 0) {
        throw std::runtime_error("Viewer cell_pixels must be positive");
    }
}

cv::Mat Viewer::renderFrame(const GridMap& map,
                            const std::vector<LocalPoint>& points,
                            const std::vector<GridCell>& path,
                            const std::vector<VehicleState>& trajectory,
                            const GridCell& start,
                            const GridCell& goal,
                            std::optional<VehicleState> vehicle) const {
    const int pixels = cellPixels(map);
    cv::Mat image(map.height() * pixels, map.width() * pixels, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int row = 0; row < map.height(); ++row) {
        for (int col = 0; col < map.width(); ++col) {
            const int image_y = (map.height() - 1 - row) * pixels;
            const cv::Rect rect(col * pixels, image_y, pixels, pixels);
            cv::rectangle(image, rect, cellColor(map.cell(row, col)), cv::FILLED);
            if (pixels >= 8) {
                cv::rectangle(image, rect, cv::Scalar(210, 210, 210), 1);
            }
        }
    }

    if (path.size() > 1) {
        std::vector<cv::Point> path_pixels;
        path_pixels.reserve(path.size());
        for (const GridCell& cell : path) {
            path_pixels.push_back(cellToPixel(map, cell, pixels));
        }
        cv::polylines(image, path_pixels, false, cv::Scalar(0, 190, 255), std::max(2, pixels / 3), cv::LINE_AA);
    }

    if (trajectory.size() > 1) {
        std::vector<cv::Point> trajectory_pixels;
        trajectory_pixels.reserve(trajectory.size());
        for (const VehicleState& state : trajectory) {
            trajectory_pixels.push_back(worldToPixel(map, state.x, state.y, pixels));
        }
        cv::polylines(image, trajectory_pixels, false, cv::Scalar(255, 210, 40), std::max(1, pixels / 4), cv::LINE_AA);
    }

    for (const LocalPoint& point : points) {
        const cv::Point pixel = worldToPixel(map, point.x, point.y, pixels);
        const int radius = std::max(3, pixels / 2);
        cv::circle(image, pixel, radius, pointColor(point.type), cv::FILLED, cv::LINE_AA);
        cv::circle(image, pixel, radius, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    cv::circle(image, cellToPixel(map, start, pixels), std::max(5, pixels), cv::Scalar(40, 210, 40), 2, cv::LINE_AA);
    cv::circle(image, cellToPixel(map, goal, pixels), std::max(5, pixels), cv::Scalar(230, 80, 230), 2, cv::LINE_AA);

    if (vehicle.has_value()) {
        drawVehicle(image, map, *vehicle, pixels);
    }

    return image;
}

void Viewer::show(const GridMap& map,
                  const std::vector<LocalPoint>& points,
                  const std::vector<GridCell>& path,
                  const std::vector<VehicleState>& trajectory,
                  const GridCell& start,
                  const GridCell& goal) const {
    if (trajectory.empty()) {
        const cv::Mat frame = renderFrame(map, points, path, trajectory, start, goal);
        cv::imshow(config_.window_name, frame);
        cv::waitKey(0);
        return;
    }

    for (const VehicleState& state : trajectory) {
        const cv::Mat frame = renderFrame(map, points, path, trajectory, start, goal, state);
        cv::imshow(config_.window_name, frame);
        const int key = cv::waitKey(config_.animation_delay_ms);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }
    }

    cv::waitKey(0);
}

void Viewer::saveSnapshot(const std::string& output_path,
                          const GridMap& map,
                          const std::vector<LocalPoint>& points,
                          const std::vector<GridCell>& path,
                          const std::vector<VehicleState>& trajectory,
                          const GridCell& start,
                          const GridCell& goal) const {
    const std::optional<VehicleState> vehicle =
        trajectory.empty() ? std::nullopt : std::optional<VehicleState>(trajectory.back());
    const cv::Mat frame = renderFrame(map, points, path, trajectory, start, goal, vehicle);
    if (!cv::imwrite(output_path, frame)) {
        throw std::runtime_error("Failed to write visualization image: " + output_path);
    }
}

int Viewer::cellPixels(const GridMap& map) const {
    const int by_width = config_.max_window_width / std::max(1, map.width());
    const int by_height = config_.max_window_height / std::max(1, map.height());
    return std::max(1, positiveMin(config_.cell_pixels, positiveMin(by_width, by_height)));
}

cv::Point Viewer::cellToPixel(const GridMap& map, const GridCell& cell, int cell_pixels) const {
    const int x = cell.col * cell_pixels + cell_pixels / 2;
    const int y = (map.height() - 1 - cell.row) * cell_pixels + cell_pixels / 2;
    return cv::Point{x, y};
}

cv::Point Viewer::worldToPixel(const GridMap& map, double x, double y, int cell_pixels) const {
    const double col = (x - map.originX()) / map.resolution();
    const double row = (y - map.originY()) / map.resolution();
    const int pixel_x = static_cast<int>(std::round(col * cell_pixels));
    const int pixel_y = static_cast<int>(std::round((static_cast<double>(map.height()) - row) * cell_pixels));
    return cv::Point{pixel_x, pixel_y};
}

void Viewer::drawVehicle(cv::Mat& image, const GridMap& map, const VehicleState& vehicle, int cell_pixels) const {
    const cv::Point center = worldToPixel(map, vehicle.x, vehicle.y, cell_pixels);
    const double size = static_cast<double>(std::max(8, cell_pixels * 2));

    auto pointAt = [&](double angle, double distance) {
        return cv::Point{
            static_cast<int>(std::round(center.x + std::cos(angle) * distance)),
            static_cast<int>(std::round(center.y - std::sin(angle) * distance))
        };
    };

    std::vector<cv::Point> body = {
        pointAt(vehicle.yaw, size),
        pointAt(vehicle.yaw + 2.45, size * 0.65),
        pointAt(vehicle.yaw - 2.45, size * 0.65)
    };

    cv::fillConvexPoly(image, body, cv::Scalar(255, 255, 40), cv::LINE_AA);
    cv::polylines(image, body, true, cv::Scalar(30, 30, 30), 2, cv::LINE_AA);
}

}  // namespace rtk_nav
