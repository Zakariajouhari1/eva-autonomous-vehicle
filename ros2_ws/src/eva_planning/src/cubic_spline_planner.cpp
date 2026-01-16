#include "eva_planning/cubic_spline_planner.hpp"
#include <algorithm>

namespace eva_planning {

CubicSplinePlanner::CubicSplinePlanner(double max_speed, double max_accel,
                                       double max_curvature, double lookahead_dist)
    : max_speed_(max_speed), max_accel_(max_accel), max_curvature_(max_curvature),
      lookahead_dist_(lookahead_dist), obstacle_avoidance_radius_(2.0), path_resolution_(0.5) {}

double CubicSplinePlanner::distance(const Point2D& p1, const Point2D& p2) const {
    double dx = p2.x - p1.x, dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

size_t CubicSplinePlanner::findClosestPointIndex(const Point2D& pos, 
                                                   const std::vector<Point2D>& path) const {
    if (path.empty()) return 0;
    size_t closest = 0;
    double min_dist = distance(pos, path[0]);
    for (size_t i = 1; i < path.size(); ++i) {
        double dist = distance(pos, path[i]);
        if (dist < min_dist) {
            min_dist = dist;
            closest = i;
        }
    }
    return closest;
}

std::vector<Point2D> CubicSplinePlanner::getLookaheadWaypoints(
    const Point2D& current_pos, const std::vector<Point2D>& global_path) const {
    
    if (global_path.empty()) return {};
    
    size_t start_idx = findClosestPointIndex(current_pos, global_path);
    std::vector<Point2D> waypoints;
    waypoints.push_back(current_pos);
    
    double accumulated_dist = 0.0;
    for (size_t i = start_idx; i < global_path.size() - 1; ++i) {
        accumulated_dist += distance(global_path[i], global_path[i + 1]);
        waypoints.push_back(global_path[i + 1]);
        if (accumulated_dist >= lookahead_dist_) break;
    }
    
    if (waypoints.size() < 2 && !global_path.empty()) {
        waypoints.push_back(global_path.back());
    }
    
    return waypoints;
}

std::vector<Point2D> CubicSplinePlanner::generateSpline(const std::vector<Point2D>& waypoints) const {
    if (waypoints.size() < 2) return waypoints;
    
    std::vector<Point2D> trajectory;
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        Point2D p1 = waypoints[i], p2 = waypoints[i + 1];
        double segment_length = distance(p1, p2);
        int num_points = std::max(2, static_cast<int>(segment_length / path_resolution_));
        
        for (int j = 0; j < num_points; ++j) {
            double t = static_cast<double>(j) / num_points;
            trajectory.emplace_back(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y));
        }
    }
    
    if (!waypoints.empty()) trajectory.push_back(waypoints.back());
    return trajectory;
}

std::vector<Point2D> CubicSplinePlanner::avoidObstacles(
    const std::vector<Point2D>& trajectory, const std::vector<Obstacle>& obstacles) const {
    
    if (obstacles.empty()) return trajectory;
    
    std::vector<Point2D> adjusted;
    for (const auto& pt : trajectory) {
        Point2D current = pt;
        Point2D avoidance(0, 0);
        
        for (const auto& obs : obstacles) {
            double dist = distance(current, Point2D(obs.x, obs.y));
            double influence_radius = obs.radius + obstacle_avoidance_radius_ * 2.0;
            
            if (dist < influence_radius) {
                double force = (influence_radius - dist) / influence_radius;
                double dx = current.x - obs.x, dy = current.y - obs.y;
                double norm = std::sqrt(dx * dx + dy * dy);
                if (norm > 0.01) {
                    avoidance.x += (dx / norm) * force * 1.0;
                    avoidance.y += (dy / norm) * force * 1.0;
                }
            }
        }
        
        adjusted.emplace_back(current.x + avoidance.x, current.y + avoidance.y);
    }
    
    return adjusted;
}

std::vector<Point2D> CubicSplinePlanner::generateLocalTrajectory(
    const Point2D& current_pos, double current_heading,
    const std::vector<Point2D>& global_path, const std::vector<Obstacle>& obstacles) {
    
    auto waypoints = getLookaheadWaypoints(current_pos, global_path);
    if (waypoints.size() < 2) return {current_pos};
    
    auto trajectory = generateSpline(waypoints);
    trajectory = avoidObstacles(trajectory, obstacles);
    
    return trajectory;
}

} // namespace eva_planning
