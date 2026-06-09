#include "planner/AStar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace rtk_nav {
namespace {

struct QueueNode {
    GridCell cell;
    double f_score = 0.0;
    double g_score = 0.0;
};

struct QueueNodeCompare {
    bool operator()(const QueueNode& lhs, const QueueNode& rhs) const {
        if (lhs.f_score == rhs.f_score) {
            return lhs.g_score < rhs.g_score;
        }
        return lhs.f_score > rhs.f_score;
    }
};

std::vector<GridCell> reconstructPath(const GridMap& map,
                                      const std::vector<int>& parent,
                                      const GridCell& start,
                                      const GridCell& goal) {
    std::vector<GridCell> path;
    GridCell current = goal;
    path.push_back(current);

    while (current != start) {
        const int current_index = map.toIndex(current);
        const int parent_index = parent[static_cast<std::size_t>(current_index)];
        if (parent_index < 0) {
            return {};
        }
        current = map.fromIndex(parent_index);
        path.push_back(current);
    }

    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace

std::vector<GridCell> AStar::plan(const GridMap& map, const GridCell& start, const GridCell& goal) const {
    if (!map.isFree(start) || !map.isFree(goal)) {
        return {};
    }

    const int total_cells = map.width() * map.height();
    std::vector<double> g_score(static_cast<std::size_t>(total_cells),
                                std::numeric_limits<double>::infinity());
    std::vector<int> parent(static_cast<std::size_t>(total_cells), -1);
    std::vector<bool> closed(static_cast<std::size_t>(total_cells), false);

    std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;

    const int start_index = map.toIndex(start);
    g_score[static_cast<std::size_t>(start_index)] = 0.0;
    open_set.push(QueueNode{start, heuristic(start, goal), 0.0});

    const std::vector<GridCell> neighbors = {
        GridCell{-1, 0},
        GridCell{1, 0},
        GridCell{0, -1},
        GridCell{0, 1},
        GridCell{-1, -1},
        GridCell{-1, 1},
        GridCell{1, -1},
        GridCell{1, 1}
    };

    while (!open_set.empty()) {
        const QueueNode current_node = open_set.top();
        open_set.pop();

        const GridCell current = current_node.cell;
        const int current_index = map.toIndex(current);
        if (closed[static_cast<std::size_t>(current_index)]) {
            continue;
        }
        closed[static_cast<std::size_t>(current_index)] = true;

        if (current == goal) {
            return reconstructPath(map, parent, start, goal);
        }

        for (const GridCell& offset : neighbors) {
            const GridCell next{current.row + offset.row, current.col + offset.col};
            if (!map.isFree(next)) {
                continue;
            }

            if (offset.row != 0 && offset.col != 0) {
                const GridCell side_a{current.row + offset.row, current.col};
                const GridCell side_b{current.row, current.col + offset.col};
                if (!map.isFree(side_a) || !map.isFree(side_b)) {
                    continue;
                }
            }

            const int next_index = map.toIndex(next);
            if (closed[static_cast<std::size_t>(next_index)]) {
                continue;
            }

            const double step_cost = (offset.row != 0 && offset.col != 0) ? std::sqrt(2.0) : 1.0;
            const double tentative_g =
                g_score[static_cast<std::size_t>(current_index)] + step_cost;

            if (tentative_g < g_score[static_cast<std::size_t>(next_index)]) {
                parent[static_cast<std::size_t>(next_index)] = current_index;
                g_score[static_cast<std::size_t>(next_index)] = tentative_g;
                const double f_score = tentative_g + heuristic(next, goal);
                open_set.push(QueueNode{next, f_score, tentative_g});
            }
        }
    }

    return {};
}

double AStar::heuristic(const GridCell& a, const GridCell& b) const {
    const double dx = static_cast<double>(a.col - b.col);
    const double dy = static_cast<double>(a.row - b.row);
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace rtk_nav
