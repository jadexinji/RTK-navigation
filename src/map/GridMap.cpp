#include "map/GridMap.h"

#include <cmath>
#include <stdexcept>

namespace rtk_nav {

GridMap::GridMap(int width, int height, double resolution, double origin_x, double origin_y)
    : width_(width),
      height_(height),
      resolution_(resolution),
      origin_x_(origin_x),
      origin_y_(origin_y),
      data_(static_cast<std::size_t>(width * height), CellState::Occupied) {
    if (width <= 0 || height <= 0) {
        throw std::runtime_error("GridMap width and height must be positive");
    }
    if (resolution <= 0.0) {
        throw std::runtime_error("GridMap resolution must be positive");
    }
}

bool GridMap::inBounds(int row, int col) const {
    return row >= 0 && row < height_ && col >= 0 && col < width_;
}

bool GridMap::inBounds(const GridCell& cell) const {
    return inBounds(cell.row, cell.col);
}

bool GridMap::isFree(int row, int col) const {
    return inBounds(row, col) && cell(row, col) == CellState::Free;
}

bool GridMap::isFree(const GridCell& cell) const {
    return isFree(cell.row, cell.col);
}

CellState GridMap::cell(int row, int col) const {
    if (!inBounds(row, col)) {
        return CellState::Occupied;
    }
    return data_[static_cast<std::size_t>(row * width_ + col)];
}

CellState GridMap::cell(const GridCell& grid_cell) const {
    return cell(grid_cell.row, grid_cell.col);
}

void GridMap::setCell(int row, int col, CellState state) {
    if (!inBounds(row, col)) {
        return;
    }
    data_[static_cast<std::size_t>(row * width_ + col)] = state;
}

void GridMap::setCell(const GridCell& grid_cell, CellState state) {
    setCell(grid_cell.row, grid_cell.col, state);
}

GridCell GridMap::worldToGrid(double x, double y) const {
    const int col = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    const int row = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    return GridCell{row, col};
}

std::pair<double, double> GridMap::gridToWorld(const GridCell& cell) const {
    const double x = origin_x_ + (static_cast<double>(cell.col) + 0.5) * resolution_;
    const double y = origin_y_ + (static_cast<double>(cell.row) + 0.5) * resolution_;
    return {x, y};
}

int GridMap::toIndex(const GridCell& cell) const {
    return cell.row * width_ + cell.col;
}

GridCell GridMap::fromIndex(int index) const {
    return GridCell{index / width_, index % width_};
}

}  // namespace rtk_nav
