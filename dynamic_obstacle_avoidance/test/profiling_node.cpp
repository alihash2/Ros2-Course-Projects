#include <iostream>
#include <vector>
#include <chrono>
#include <numeric>
#include <iomanip>
#include <memory>

#include "dynamic_obstacle_avoidance/astar_planner.hpp"
#include "dynamic_obstacle_avoidance/rrt_planner.hpp"

using namespace dynamic_obstacle_avoidance;

/**
 * Expert-Level Profiling Utility for Nav2 Planners
 * This tool measures execution time, path optimality, and success rates.
 */

struct BenchmarkResult {
    std::string name;
    double avg_time_ms;
    double path_length;
    bool success;
};

// Helper to calculate total path length
double calculate_path_length(const std::vector<Node>& path) {
    if (path.empty()) return 0.0;
    double length = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double dx = path[i+1].x - path[i].x;
        double dy = path[i+1].y - path[i].y;
        length += std::sqrt(dx*dx + dy*dy);
    }
    return length;
}

// Modular Benchmarking Function
BenchmarkResult run_benchmark(const std::string& name, 
                             PlannerBase& planner, 
                             const std::vector<int>& map, 
                             int w, int h, 
                             Node start, Node goal,
                             int iterations = 100) 
{
    std::vector<double> timings;
    std::vector<Node> last_path;
    bool success = false;

    // Warm-up run
    last_path = planner.plan(map, w, h, start, goal);
    if (!last_path.empty()) success = true;

    // Measured runs
    for (int i = 0; i < iterations; ++i) {
        auto start_time = std::chrono::high_resolution_clock::now();
        planner.plan(map, w, h, start, goal);
        auto end_time = std::chrono::high_resolution_clock::now();
        
        std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
        timings.push_back(elapsed.count());
    }

    double avg_time = std::accumulate(timings.begin(), timings.end(), 0.0) / iterations;
    double length = calculate_path_length(last_path);

    return {name, avg_time, length, success};
}

int main() {
    // 1. Create a Mock Map (100x100)
    int width = 100;
    int height = 100;
    std::vector<int> map(width * height, 0); // 0 = Free space

    // 2. Add a Wall (Obstacle) in the middle
    // Creates a vertical wall from y=20 to y=80 at x=50
    for (int y = 20; y < 80; ++y) {
        map[y * width + 50] = 100; 
    }

    Node start = {10, 50};
    Node goal = {90, 50};

    std::cout << "=====================================================" << std::endl;
    std::cout << "   Nav2 Custom Planner Profiling (100 Iterations)    " << std::endl;
    std::cout << "=====================================================" << std::endl;
    std::cout << std::left << std::setw(15) << "Planner" 
              << std::setw(15) << "Time (ms)" 
              << std::setw(15) << "Length (px)" 
              << "Status" << std::endl;
    std::cout << "----------------------------------------------------" << std::endl;

    // 3. Test A*
    AStarPlanner astar;
    auto res_astar = run_benchmark("A*", astar, map, width, height, start, goal);
    std::cout << std::left << std::setw(15) << res_astar.name 
              << std::setw(15) << std::fixed << std::setprecision(4) << res_astar.avg_time_ms 
              << std::setw(15) << res_astar.path_length 
              << (res_astar.success ? "SUCCESS" : "FAILED") << std::endl;

    // 4. Test RRT
    RRTPlanner rrt;
    auto res_rrt = run_benchmark("RRT", rrt, map, width, height, start, goal);
    std::cout << std::left << std::setw(15) << res_rrt.name 
              << std::setw(15) << std::fixed << std::setprecision(4) << res_rrt.avg_time_ms 
              << std::setw(15) << res_rrt.path_length 
              << (res_rrt.success ? "SUCCESS" : "FAILED") << std::endl;

    std::cout << "----------------------------------------------------" << std::endl;
    
    // Expert Analysis Output
    std::cout << "\n[EXPERT ANALYSIS]" << std::endl;
    if (res_astar.avg_time_ms < res_rrt.avg_time_ms) {
        std::cout << "- A* was faster on this grid resolution." << std::endl;
    } else {
        std::cout << "- RRT was faster (typical for sparse or large maps)." << std::endl;
    }

    if (res_astar.path_length < res_rrt.path_length) {
        std::cout << "- A* found a shorter path (Optimality Check: PASSED)." << std::endl;
    }
    
    std::cout << "=====================================================" << std::endl;

    return 0;
}
