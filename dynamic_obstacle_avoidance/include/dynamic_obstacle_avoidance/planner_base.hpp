#ifndef DYNAMIC_OBSTACLE_AVOIDANCE__PLANNER_BASE_HPP_
#define DYNAMIC_OBSTACLE_AVOIDANCE__PLANNER_BASE_HPP_

#include <vector>

namespace dynamic_obstacle_avoidance{

    struct Node {
        int x;
        int y;
    };

    class PlannerBase{
        public:
            virtual ~PlannerBase() = default;

            virtual std::vector<Node>plan(
                const std::vector<int>& occupancy_grid,
                int width,
                int height,
                Node start,
                Node goal
            ) = 0;
    };

}

#endif
