#ifndef EVA_PLANNING_CUBIC_SPLINE_PLANNER_HPP_
#define EVA_PLANNING_CUBIC_SPLINE_PLANNER_HPP_

#include <vector>
#include <cmath>

namespace eva_planning {

struct Point2D {
    double x, y;
    Point2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
};

struct Obstacle {
    double x, y, radius;
    Obstacle(double x_, double y_, double r_) : x(x_), y(y_), radius(r_) {}
};

class CubicSplinePlanner {
public:
    CubicSplinePlanner(double max_speed = 5.0, double max_accel = 2.0,
                       double max_curvature = 1.0, double lookahead_dist = 10.0);
    
    std::vector<Point2D> generateLocalTrajectory(
        const Point2D& current_pos, double current_heading,
        const std::vector<Point2D>& global_path,
        const std::vector<Obstacle>& obstacles);
    
    void setMaxSpeed(double speed) { max_speed_ = speed; }
    void setLookaheadDistance(double dist) { lookahead_dist_ = dist; }
    void setObstacleAvoidanceRadius(double radius) { obstacle_avoidance_radius_ = radius; }
    
private:
    double max_speed_, max_accel_, max_curvature_, lookahead_dist_;
    double obstacle_avoidance_radius_, path_resolution_;
    
    size_t findClosestPointIndex(const Point2D& pos, const std::vector<Point2D>& path) const;
    std::vector<Point2D> getLookaheadWaypoints(const Point2D& current_pos,
                                                 const std::vector<Point2D>& global_path) const;
    std::vector<Point2D> generateSpline(const std::vector<Point2D>& waypoints) const;
    std::vector<Point2D> avoidObstacles(const std::vector<Point2D>& trajectory,
                                         const std::vector<Obstacle>& obstacles) const;
    double distance(const Point2D& p1, const Point2D& p2) const;
};

} // namespace eva_planning

#endif
