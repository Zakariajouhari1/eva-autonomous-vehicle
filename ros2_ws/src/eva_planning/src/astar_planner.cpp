#include "eva_planning/astar_planner.hpp"
#include <algorithm>
#include <iostream>

namespace eva_planning {

AStarPlanner::AStarPlanner(double resolution, double robot_radius)
    : resolution_(resolution), robot_radius_(robot_radius), heuristic_type_("euclidean") {}

void AStarPlanner::setMap(const std::vector<std::vector<int>>& grid) {
    grid_ = grid;
}

void AStarPlanner::setHeuristic(const std::string& type) {
    heuristic_type_ = type;
}

std::pair<int, int> AStarPlanner::worldToGrid(double x, double y) const {
    int gx = static_cast<int>(std::round(x / resolution_));
    int gy = static_cast<int>(std::round(y / resolution_));
    
    // Safety clamping to prevent out-of-bounds
    if (!grid_.empty()) {
        int width = grid_[0].size();
        int height = grid_.size();
        gx = std::max(0, std::min(gx, width - 1));
        gy = std::max(0, std::min(gy, height - 1));
    }
    
    return {gx, gy};
}

std::pair<double, double> AStarPlanner::gridToWorld(int gx, int gy) const {
    return {gx * resolution_, gy * resolution_};
}

double AStarPlanner::calculateHeuristic(int x1, int y1, int x2, int y2) const {
    double dx = std::abs(x1 - x2);
    double dy = std::abs(y1 - y2);
    
    if (heuristic_type_ == "manhattan") {
        return (dx + dy) * resolution_;
    } else if (heuristic_type_ == "octile") {
        return std::max(dx, dy) * resolution_;
    }
    return std::sqrt(dx * dx + dy * dy) * resolution_;
}

bool AStarPlanner::isValid(int x, int y) const {
    if (grid_.empty()) return false;
    int height = grid_.size();
    int width = grid_[0].size();
    if (x < 0 || x >= width || y < 0 || y >= height) return false;
    return grid_[y][x] == 0;
}

std::vector<std::shared_ptr<Node>> AStarPlanner::getNeighbors(const std::shared_ptr<Node>& node) const {
    std::vector<std::shared_ptr<Node>> neighbors;
    std::vector<std::pair<int, int>> directions = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1},
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    };
    
    for (const auto& dir : directions) {
        int nx = node->x + dir.first;
        int ny = node->y + dir.second;
        if (isValid(nx, ny)) {
            neighbors.push_back(std::make_shared<Node>(nx, ny));
        }
    }
    return neighbors;
}

std::vector<std::pair<double, double>> AStarPlanner::reconstructPath(std::shared_ptr<Node> goal) const {
    std::vector<std::pair<double, double>> path;
    auto current = goal;
    while (current != nullptr) {
        path.push_back(gridToWorld(current->x, current->y));
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::pair<double, double>> AStarPlanner::smoothPath(
    const std::vector<std::pair<double, double>>& path) const {
    if (path.size() < 3) return path;
    
    std::vector<std::pair<double, double>> smoothed;
    smoothed.push_back(path[0]);
    
    size_t i = 0;
    while (i < path.size() - 1) {
        size_t j = path.size() - 1;
        while (j > i + 1) {
            auto [x1, y1] = worldToGrid(path[i].first, path[i].second);
            auto [x2, y2] = worldToGrid(path[j].first, path[j].second);
            
            bool collision = false;
            int dx = std::abs(x2 - x1), dy = std::abs(y2 - y1);
            int sx = (x1 < x2) ? 1 : -1, sy = (y1 < y2) ? 1 : -1;
            int err = dx - dy, x = x1, y = y1;
            
            while (true) {
                if (!isValid(x, y)) { collision = true; break; }
                if (x == x2 && y == y2) break;
                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; x += sx; }
                if (e2 < dx) { err += dx; y += sy; }
            }
            
            if (!collision) {
                smoothed.push_back(path[j]);
                i = j;
                break;
            }
            j--;
        }
        if (j == i + 1) {
            smoothed.push_back(path[i + 1]);
            i++;
        }
    }
    return smoothed;
}

std::vector<std::pair<double, double>> AStarPlanner::plan(
    double start_x, double start_y, double goal_x, double goal_y) {
    
    if (grid_.empty()) {
        std::cerr << "Error: Grid map not set!" << std::endl;
        return {};
    }
    
    auto [start_gx, start_gy] = worldToGrid(start_x, start_y);
    auto [goal_gx, goal_gy] = worldToGrid(goal_x, goal_y);
    
    if (!isValid(start_gx, start_gy) || !isValid(goal_gx, goal_gy)) {
        std::cerr << "Error: Start or goal invalid!" << std::endl;
        return {};
    }
    
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NodeCompare> open_set;
    std::unordered_map<int, std::unordered_map<int, bool>> closed_set;
    std::unordered_map<int, std::unordered_map<int, std::shared_ptr<Node>>> all_nodes;
    
    auto start_node = std::make_shared<Node>(start_gx, start_gy);
    start_node->g_cost = 0;
    start_node->h_cost = calculateHeuristic(start_gx, start_gy, goal_gx, goal_gy);
    start_node->f_cost = start_node->h_cost;
    
    open_set.push(start_node);
    all_nodes[start_gx][start_gy] = start_node;
    
    while (!open_set.empty()) {
        auto current = open_set.top();
        open_set.pop();
        
        if (current->x == goal_gx && current->y == goal_gy) {
            return smoothPath(reconstructPath(current));
        }
        
        closed_set[current->x][current->y] = true;
        
        for (auto& neighbor : getNeighbors(current)) {
            if (closed_set[neighbor->x][neighbor->y]) continue;
            
            double dx = std::abs(neighbor->x - current->x);
            double dy = std::abs(neighbor->y - current->y);
            double move_cost = ((dx + dy > 1) ? std::sqrt(2.0) : 1.0) * resolution_;
            double tentative_g = current->g_cost + move_cost;
            
            bool is_better = false;
            if (all_nodes[neighbor->x].find(neighbor->y) == all_nodes[neighbor->x].end()) {
                is_better = true;
            } else if (tentative_g < all_nodes[neighbor->x][neighbor->y]->g_cost) {
                is_better = true;
            }
            
            if (is_better) {
                neighbor->parent = current;
                neighbor->g_cost = tentative_g;
                neighbor->h_cost = calculateHeuristic(neighbor->x, neighbor->y, goal_gx, goal_gy);
                neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
                all_nodes[neighbor->x][neighbor->y] = neighbor;
                open_set.push(neighbor);
            }
        }
    }
    
    std::cerr << "Error: No path found!" << std::endl;
    return {};
}

} // namespace eva_planning
