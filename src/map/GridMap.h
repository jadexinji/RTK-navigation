#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "common/Types.h"

namespace rtk_nav {

enum class CellState : std::uint8_t {
    Unknown = 0,
    Free = 1,
    Occupied = 2
};

class GridMap {
public:
    GridMap() = default;
    GridMap(int width, int height, double resolution, double origin_x, double origin_y);

    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }
    double originX() const { return origin_x_; }
    double originY() const { return origin_y_; }

    bool inBounds(int row, int col) const;
    bool inBounds(const GridCell& cell) const;
    bool isFree(int row, int col) const;
    bool isFree(const GridCell& cell) const;

    CellState cell(int row, int col) const;
    CellState cell(const GridCell& grid_cell) const;
    void setCell(int row, int col, CellState state);
    void setCell(const GridCell& grid_cell, CellState state);

    GridCell worldToGrid(double x, double y) const;
    std::pair<double, double> gridToWorld(const GridCell& cell) const;

    const std::vector<CellState>& data() const { return data_; }
    int toIndex(const GridCell& cell) const;
    GridCell fromIndex(int index) const;

private:
    int width_ = 0;
    int height_ = 0;
    double resolution_ = 1.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    std::vector<CellState> data_;
};

}  // namespace rtk_nav
