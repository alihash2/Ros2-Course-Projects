#include "dynamic_obstacle_avoidance/astar_planner.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace dynamic_obstacle_avoidance {

struct SearchNode {
    Node node;
    double g_cost = 0.0;
    double h_cost = 0.0;

    double f_cost() const { return g_cost + h_cost; }
};

int to_index(int x, int y, int width) {
    return y * width + x;
}

double heuristic(Node node, Node goal) {
    return std::sqrt(std::pow(goal.x - node.x, 2) + std::pow(goal.y - node.y, 2));
}

std::vector<Node> AStarPlanner::plan(
    const std::vector<int>& occupancy_grid,
    int width,
    int height,
    Node start,
    Node goal) 
{
    std::vector<Node> path;
    int map_size = width * height;

    // 1. Setup memory
    std::vector<bool> closed_list(map_size, false);
    std::vector<double> g_costs(map_size, std::numeric_limits<double>::infinity());
    std::vector<int> parent_ids(map_size, -1);
    std::vector<SearchNode> open_list;

    // 2. Initialize Start
    SearchNode start_node;
    start_node.node = start;
    start_node.g_cost = 0.0;
    start_node.h_cost = heuristic(start, goal);

    int start_idx = to_index(start.x, start.y, width);
    g_costs[start_idx] = 0.0;
    open_list.push_back(start_node);

    bool found_goal = false;
    int goal_idx = -1;

    // 3. Main Loop
    while (!open_list.empty()) {
        // Sort to get best node at the back
        std::sort(open_list.begin(), open_list.end(), [](const SearchNode& a, const SearchNode& b) {
            return a.f_cost() > b.f_cost();
        });

        SearchNode current = open_list.back();
        open_list.pop_back();

        int current_idx = to_index(current.node.x, current.node.y, width);

        // Check if goal reached
        if (current.node.x == goal.x && current.node.y == goal.y) {
            found_goal = true;
            goal_idx = current_idx;
            break;
        }

        if (closed_list[current_idx]) continue;
        closed_list[current_idx] = true;

        // 4. Expand 8 Neighbors
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                int nx = current.node.x + dx;
                int ny = current.node.y + dy;

                // Bounds check
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

                int n_idx = to_index(nx, ny, width);

                // Obstacle check (occupancy > 50 is obstacle)
                if (occupancy_grid[n_idx] > 50 || closed_list[n_idx]) continue;

                double move_cost = std::sqrt(dx * dx + dy * dy);
                double new_g = current.g_cost + move_cost;

                if (new_g < g_costs[n_idx]) {
                    g_costs[n_idx] = new_g;
                    parent_ids[n_idx] = current_idx;

                    SearchNode neighbor;
                    neighbor.node = {nx, ny};
                    neighbor.g_cost = new_g;
                    neighbor.h_cost = heuristic(neighbor.node, goal);
                    open_list.push_back(neighbor);
                }
            }
        }
    }

    // 5. Reconstruct Path
    if (found_goal) {
        int trace_idx = goal_idx;
        while (trace_idx != -1) {
            Node p;
            p.x = trace_idx % width;
            p.y = trace_idx / width;
            path.push_back(p);
            trace_idx = parent_ids[trace_idx];
        }
        std::reverse(path.begin(), path.end());
    }

    return path;
}

} // namespace dynamic_obstacle_avoidance
