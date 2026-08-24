#include "dynamic_obstacle_avoidance/rrt_planner.hpp"
#include <cmath>
#include <algorithm>
#include <random>

namespace dynamic_obstacle_avoidance {

double RRTPlanner::distance(Node a, Node b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

Node RRTPlanner::get_random_node(int width, int height) {
    static std::mt19937 gen(1337);
    std::uniform_int_distribution<> dis_x(0, width - 1);
    std::uniform_int_distribution<> dis_y(0, height - 1);
    return {dis_x(gen), dis_y(gen)};
}

int RRTPlanner::get_nearest_node_index(const std::vector<RRTNode>& tree, Node target) {
    int min_idx = 0;
    double min_dist = distance(tree[0].node, target);

    for (size_t i = 1; i < tree.size(); ++i) {
        double d = distance(tree[i].node, target);
        if (d < min_dist) {
            min_dist = d;
            min_idx = i;
        }
    }
    return min_idx;
}

Node RRTPlanner::step(Node from, Node to, double step_size) {
    double d = distance(from, to);
    if (d <= step_size || d < 1e-6) return to;

    double theta = std::atan2(to.y - from.y, to.x - from.x);
    return {
        static_cast<int>(std::round(from.x + step_size * std::cos(theta))),
        static_cast<int>(std::round(from.y + step_size * std::sin(theta)))
    };
}

bool RRTPlanner::is_path_clear(const std::vector<int>& grid, int width, Node from, Node to) {
    constexpr int OBSTACLE_THRESHOLD = 253;
    int height = static_cast<int>(grid.size() / width);

    double dist = distance(from, to);
    int steps = std::max(1, static_cast<int>(std::ceil(dist * 2.0)));

    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        int x = static_cast<int>(std::round(from.x + t * (to.x - from.x)));
        int y = static_cast<int>(std::round(from.y + t * (to.y - from.y)));

        if (x < 0 || x >= width || y < 0 || y >= height) return false;
        int idx = y * width + x;
        if (grid[idx] >= OBSTACLE_THRESHOLD) return false;
    }
    return true;
}

std::vector<Node> RRTPlanner::plan(
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

    std::vector<RRTNode> tree;
    tree.push_back({start, -1});

    constexpr int max_iterations = 3000;
    constexpr double step_size = 4.0;
    constexpr double goal_threshold = 4.0;
    constexpr int goal_bias_percent = 20; // 20% goal bias for faster convergence

    static std::mt19937 gen(42);
    std::uniform_int_distribution<> dis(0, 99);

    bool found_goal = false;
    int goal_node_idx = -1;

    for (int i = 0; i < max_iterations; ++i) {
        Node target = (dis(gen) < goal_bias_percent) ? goal : get_random_node(width, height);

        int nearest_idx = get_nearest_node_index(tree, target);
        Node new_node_coords = step(tree[nearest_idx].node, target, step_size);

        if (is_path_clear(occupancy_grid, width, tree[nearest_idx].node, new_node_coords)) {
            tree.push_back({new_node_coords, nearest_idx});

            if (distance(new_node_coords, goal) <= goal_threshold) {
                if (is_path_clear(occupancy_grid, width, new_node_coords, goal)) {
                    tree.push_back({goal, static_cast<int>(tree.size() - 1)});
                    goal_node_idx = static_cast<int>(tree.size() - 1);
                    found_goal = true;
                    break;
                }
            }
        }
    }

    if (found_goal) {
        int curr = goal_node_idx;
        while (curr != -1) {
            path.push_back(tree[curr].node);
            curr = tree[curr].parent_index;
        }
        std::reverse(path.begin(), path.end());
    }

    return path;
}

} // namespace dynamic_obstacle_avoidance
