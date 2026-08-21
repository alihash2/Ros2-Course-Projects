#ifndef DYNAMIC_OBSTACLE_AVOIDANCE__ASTAR_PLANNER_HPP_
#define DYNAMIC_OBSTACLE_AVOIDANCE__ASTAR_PLANNER_HPP_

#include "dynamic_obstacle_avoidance/planner_base.hpp"

namespace dynamic_obstacle_avoidance{

    class AStarPlanner : public PlannerBase{
        public:
            std::vector<Node> plan(
                const std::vector<int>& occupancy_grid,
                int width,
                int height,
                Node start,
                Node goal
            ) override;
    };

}

#endif
