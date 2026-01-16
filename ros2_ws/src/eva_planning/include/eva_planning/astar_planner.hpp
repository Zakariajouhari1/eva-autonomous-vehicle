// include/eva_planning/astar_planner.hpp
#ifndef EVA_PLANNING_ASTAR_PLANNER_HPP_
#define EVA_PLANNING_ASTAR_PLANNER_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <memory>
#include <limits>

namespace eva_planning {

struct Node {
    int x, y;
    double g_cost, h_cost, f_cost;
    std::shared_ptr<Node> parent;
    
    Node(int x_, int y_) : x(x_), y(y_), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}
    
    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

struct NodeCompare {
    bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
        return a->f_cost > b->f_cost;
    }
};

class AStarPlanner {
public:
    AStarPlanner(double resolution = 0.5, double robot_radius = 1.0);
    ~AStarPlanner() = default;
    
    void setMap(const std::vector<std::vector<int>>& grid);
    std::vector<std::pair<double, double>> plan(double start_x, double start_y, double goal_x, double goal_y);
    void setHeuristic(const std::string& type);
    
private:
    double resolution_;
    double robot_radius_;
    std::vector<std::vector<int>> grid_;
    std::string heuristic_type_;
    
    std::pair<int, int> worldToGrid(double x, double y) const;
    std::pair<double, double> gridToWorld(int gx, int gy) const;
    double calculateHeuristic(int x1, int y1, int x2, int y2) const;
    bool isValid(int x, int y) const;
    std::vector<std::shared_ptr<Node>> getNeighbors(const std::shared_ptr<Node>& node) const;
    std::vector<std::pair<double, double>> reconstructPath(std::shared_ptr<Node> goal) const;
    std::vector<std::pair<double, double>> smoothPath(const std::vector<std::pair<double, double>>& path) const;
};

} // namespace eva_planning

#endif
