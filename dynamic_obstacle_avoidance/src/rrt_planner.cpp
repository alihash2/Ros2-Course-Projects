#include "dynamic_obstacle_avoidance/rrt_planner.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

namespace dynamic_obstacle_avoidance {

double RRTPlanner::distance(Node a, Node b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

Node RRTPlanner::get_random_node(int width, int height) {
    static std::mt19937 gen(std::chrono::system_clock::now().time_since_epoch().count());
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
    if (d < step_size) return to;

    double theta = std::atan2(to.y - from.y, to.x - from.x);
    return {
        static_cast<int>(from.x + step_size * std::cos(theta)),
        static_cast<int>(from.y + step_size * std::sin(theta))
    };
}

bool RRTPlanner::is_path_clear(const std::vector<int>& grid, int width, Node from, Node to) {
    // Simple Bresenham-like collision check
    double d = distance(from, to);
    int steps = static_cast<int>(d);
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        int x = static_cast<int>(from.x + t * (to.x - from.x));
        int y = static_cast<int>(from.y + t * (to.y - from.y));
        if (x < 0 || x >= width || y < 0 || y >= static_cast<int>(grid.size() / width)) return false;
        if (grid[y * width + x] > 50) return false;
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
    std::vector<RRTNode> tree;
    tree.push_back({start, -1});

    const int max_iterations = 2000;
    const double step_size = 2.0;
    const double goal_threshold = 2.0;
    const int goal_bias_percent = 5;

    bool found_goal = false;
    int goal_node_idx = -1;

    for (int i = 0; i < max_iterations; ++i) {
        Node target;
        // Goal Bias: Occasionally try to grow directly toward the goal
        static std::mt19937 gen(std::chrono::system_clock::now().time_since_epoch().count());
        std::uniform_int_distribution<> dis(0, 100);
        if (dis(gen) < goal_bias_percent) {
            target = goal;
        } else {
            target = get_random_node(width, height);
        }

        int nearest_idx = get_nearest_node_index(tree, target);
        Node new_node_coords = step(tree[nearest_idx].node, target, step_size);

        if (is_path_clear(occupancy_grid, width, tree[nearest_idx].node, new_node_coords)) {
            tree.push_back({new_node_coords, nearest_idx});
            
            if (distance(new_node_coords, goal) < goal_threshold) {
                // Final check to goal
                if (is_path_clear(occupancy_grid, width, new_node_coords, goal)) {
                    tree.push_back({goal, static_cast<int>(tree.size() - 1)});
                    goal_node_idx = tree.size() - 1;
                    found_goal = true;
                    break;
                }
            }
        }
    }

    std::vector<Node> path;
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
