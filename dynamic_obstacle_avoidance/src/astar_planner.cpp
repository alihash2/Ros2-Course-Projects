#include "dynamic_obstacle_avoidance/astar_planner.hpp"
#include <cmath>
#include <queue>
#include <algorithm>
#include <limits>

namespace dynamic_obstacle_avoidance {

struct AStarNode {
    int x;
    int y;
    double g_cost;
    double h_cost;

    double f_cost() const { return g_cost + h_cost; }

    bool operator>(const AStarNode& other) const {
        return f_cost() > other.f_cost();
    }
};

static inline int to_index(int x, int y, int width) {
    return y * width + x;
}

static inline double euclidean_heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

std::vector<Node> AStarPlanner::plan(
    const std::vector<int>& occupancy_grid,
    int width,
    int height,
    Node start,
    Node goal) 
{
    std::vector<Node> path;
    if (width <= 0 || height <= 0 || occupancy_grid.size() < static_cast<size_t>(width * height)) {
        return path;
    }

    if (start.x < 0 || start.x >= width || start.y < 0 || start.y >= height ||
        goal.x < 0 || goal.x >= width || goal.y < 0 || goal.y >= height) {
        return path;
    }

    int map_size = width * height;
    std::vector<double> g_costs(map_size, std::numeric_limits<double>::infinity());
    std::vector<int> parent_ids(map_size, -1);
    std::vector<bool> closed_list(map_size, false);

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_queue;

    int start_idx = to_index(start.x, start.y, width);
    g_costs[start_idx] = 0.0;
    open_queue.push({start.x, start.y, 0.0, euclidean_heuristic(start.x, start.y, goal.x, goal.y)});

    bool found_goal = false;
    int goal_idx = to_index(goal.x, goal.y, width);

    // Nav2 costmap obstacle threshold (253 = INSCRIBED_INFLATED, 254 = LETHAL, 255 = NO_INFO)
    constexpr int OBSTACLE_THRESHOLD = 253;

    while (!open_queue.empty()) {
        AStarNode current = open_queue.top();
        open_queue.pop();

        int curr_idx = to_index(current.x, current.y, width);

        if (closed_list[curr_idx]) continue;
        closed_list[curr_idx] = true;

        if (current.x == goal.x && current.y == goal.y) {
            found_goal = true;
            break;
        }

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                int nx = current.x + dx;
                int ny = current.y + dy;

                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

                int n_idx = to_index(nx, ny, width);
                int cell_cost = occupancy_grid[n_idx];

                if (cell_cost >= OBSTACLE_THRESHOLD || closed_list[n_idx]) continue;

                // Prevent diagonal corner cutting
                if (dx != 0 && dy != 0) {
                    int card1 = to_index(current.x + dx, current.y, width);
                    int card2 = to_index(current.x, current.y + dy, width);
                    if (occupancy_grid[card1] >= OBSTACLE_THRESHOLD || occupancy_grid[card2] >= OBSTACLE_THRESHOLD) {
                        continue;
                    }
                }

                double step_dist = (dx != 0 && dy != 0) ? 1.41421356 : 1.0;
                double cost_penalty = (cell_cost > 0) ? (cell_cost / 254.0) * 2.0 : 0.0;
                double new_g = current.g_cost + step_dist + cost_penalty;

                if (new_g < g_costs[n_idx]) {
                    g_costs[n_idx] = new_g;
                    parent_ids[n_idx] = curr_idx;
                    double h = euclidean_heuristic(nx, ny, goal.x, goal.y);
                    open_queue.push({nx, ny, new_g, h});
                }
            }
        }
    }

    if (found_goal) {
        int trace_idx = goal_idx;
        while (trace_idx != -1) {
            path.push_back({trace_idx % width, trace_idx / width});
            trace_idx = parent_ids[trace_idx];
        }
        std::reverse(path.begin(), path.end());
    }

    return path;
}

} // namespace dynamic_obstacle_avoidance
