#ifndef DYNAMIC_OBSTACLE_AVOIDANCE__RRT_PLANNER_HPP_
#define DYNAMIC_OBSTACLE_AVOIDANCE__RRT_PLANNER_HPP_

#include "dynamic_obstacle_avoidance/planner_base.hpp"
#include <random>

namespace dynamic_obstacle_avoidance {

struct RRTNode {
    Node node;
    int parent_index = -1;
};

class RRTPlanner : public PlannerBase {
public:
    std::vector<Node> plan(
        const std::vector<int>& occupancy_grid,
        int width,
        int height,
        Node start,
        Node goal) override;

private:
    double distance(Node a, Node b);
    Node get_random_node(int width, int height);
    int get_nearest_node_index(const std::vector<RRTNode>& tree, Node target);
    Node step(Node from, Node to, double step_size);
    bool is_path_clear(const std::vector<int>& grid, int width, Node from, Node to);
};

} // namespace dynamic_obstacle_avoidance

#endif // DYNAMIC_OBSTACLE_AVOIDANCE__RRT_PLANNER_HPP_
