#pragma once

#include <optional>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "common/Types.h"
#include "map/GridMap.h"

namespace rtk_nav {

struct ViewerConfig {
    int cell_pixels = 8;
    int max_window_width = 1200;
    int max_window_height = 900;
    int animation_delay_ms = 25;
    std::string window_name = "RTK Navigation System";
};

class Viewer {
public:
    explicit Viewer(ViewerConfig config = {});

    cv::Mat renderFrame(const GridMap& map,
                        const std::vector<LocalPoint>& points,
                        const std::vector<GridCell>& path,
                        const std::vector<VehicleState>& trajectory,
                        const GridCell& start,
                        const GridCell& goal,
                        std::optional<VehicleState> vehicle = std::nullopt) const;

    void show(const GridMap& map,
              const std::vector<LocalPoint>& points,
              const std::vector<GridCell>& path,
              const std::vector<VehicleState>& trajectory,
              const GridCell& start,
              const GridCell& goal) const;

    void saveSnapshot(const std::string& output_path,
                      const GridMap& map,
                      const std::vector<LocalPoint>& points,
                      const std::vector<GridCell>& path,
                      const std::vector<VehicleState>& trajectory,
                      const GridCell& start,
                      const GridCell& goal) const;

private:
    int cellPixels(const GridMap& map) const;
    cv::Point cellToPixel(const GridMap& map, const GridCell& cell, int cell_pixels) const;
    cv::Point worldToPixel(const GridMap& map, double x, double y, int cell_pixels) const;
    void drawVehicle(cv::Mat& image, const GridMap& map, const VehicleState& vehicle, int cell_pixels) const;

    ViewerConfig config_;
};

}  // namespace rtk_nav
