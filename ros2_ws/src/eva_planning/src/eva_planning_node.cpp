#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include "eva_planning/astar_planner.hpp"
#include "eva_planning/cubic_spline_planner.hpp"

#include <memory>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class EVAPlanningNode : public rclcpp::Node {
public:
    EVAPlanningNode() : Node("eva_planning_node") {
        // Parameters
        this->declare_parameter("global_planner_rate", 1.0);
        this->declare_parameter("local_planner_rate", 10.0);
        this->declare_parameter("map_resolution", 0.5);
        this->declare_parameter("robot_radius", 1.0);
        this->declare_parameter("max_speed", 5.0);
        this->declare_parameter("lookahead_distance", 10.0);
        this->declare_parameter("obstacle_avoidance_radius", 2.0);
        
        double global_rate = this->get_parameter("global_planner_rate").as_double();
        double local_rate = this->get_parameter("local_planner_rate").as_double();
        double map_res = this->get_parameter("map_resolution").as_double();
        double robot_radius = this->get_parameter("robot_radius").as_double();
        double max_speed = this->get_parameter("max_speed").as_double();
        double lookahead = this->get_parameter("lookahead_distance").as_double();
        double obs_radius = this->get_parameter("obstacle_avoidance_radius").as_double();
        
        // Initialize planners
        astar_planner_ = std::make_unique<eva_planning::AStarPlanner>(map_res, robot_radius);
        astar_planner_->setHeuristic("euclidean");
        
        spline_planner_ = std::make_unique<eva_planning::CubicSplinePlanner>(
            max_speed, 2.0, 1.0, lookahead);
        spline_planner_->setObstacleAvoidanceRadius(obs_radius);
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/vehicle/odom", 10,
            std::bind(&EVAPlanningNode::odomCallback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&EVAPlanningNode::goalCallback, this, std::placeholders::_1));
        
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&EVAPlanningNode::mapCallback, this, std::placeholders::_1));
        
        // Publishers
        global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/global_path", 10);
        local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/local_path", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/planning/status", 10);
        
        // Timers
        global_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / global_rate)),
            std::bind(&EVAPlanningNode::globalPlanningCallback, this));
        
        local_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / local_rate)),
            std::bind(&EVAPlanningNode::localPlanningCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "EVA Planning Node initialized");
    }

private:
    std::unique_ptr<eva_planning::AStarPlanner> astar_planner_;
    std::unique_ptr<eva_planning::CubicSplinePlanner> spline_planner_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    rclcpp::TimerBase::SharedPtr global_timer_;
    rclcpp::TimerBase::SharedPtr local_timer_;
    
    nav_msgs::msg::Odometry current_odom_;
    geometry_msgs::msg::PoseStamped current_goal_;
    std::vector<eva_planning::Point2D> global_path_;
    std::vector<eva_planning::Obstacle> detected_obstacles_;
    bool has_odom_ = false;
    bool has_goal_ = false;
    bool has_map_ = false;
    std::vector<std::vector<int>> occupancy_grid_;
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = *msg;
        has_odom_ = true;
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_goal_ = *msg;
        has_goal_ = true;
        RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", 
                    msg->pose.position.x, msg->pose.position.y);
        globalPlanningCallback();
    }
    
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        int width = msg->info.width;
        int height = msg->info.height;
        
        occupancy_grid_.resize(height);
        for (int y = 0; y < height; ++y) {
            occupancy_grid_[y].resize(width);
            for (int x = 0; x < width; ++x) {
                occupancy_grid_[y][x] = (msg->data[y * width + x] > 50) ? 1 : 0;
            }
        }
        
        astar_planner_->setMap(occupancy_grid_);
        has_map_ = true;
        RCLCPP_INFO(this->get_logger(), "Map received: %dx%d", width, height);
    }
    
    void globalPlanningCallback() {
        if (!has_odom_ || !has_goal_ || !has_map_) return;
        
        auto start = std::chrono::high_resolution_clock::now();
        
        double start_x = current_odom_.pose.pose.position.x;
        double start_y = current_odom_.pose.pose.position.y;
        double goal_x = current_goal_.pose.position.x;
        double goal_y = current_goal_.pose.position.y;
        
        auto path = astar_planner_->plan(start_x, start_y, goal_x, goal_y);
        
        if (path.empty()) {
            RCLCPP_WARN(this->get_logger(), "Global planning failed!");
            publishStatus("FAILED: No path found");
            return;
        }
        
        global_path_.clear();
        for (const auto& point : path) {
            global_path_.emplace_back(point.first, point.second);
        }
        
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "odom";
        
        for (const auto& point : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.first;
            pose.pose.position.y = point.second;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        
        global_path_pub_->publish(path_msg);
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        RCLCPP_INFO(this->get_logger(), "Global path: %zu points in %ld ms", 
                    path.size(), duration.count());
        publishStatus("SUCCESS: Global path computed");
    }
    
    void localPlanningCallback() {
        if (!has_odom_ || global_path_.empty()) return;
        
        double current_x = current_odom_.pose.pose.position.x;
        double current_y = current_odom_.pose.pose.position.y;
        
        double qx = current_odom_.pose.pose.orientation.x;
        double qy = current_odom_.pose.pose.orientation.y;
        double qz = current_odom_.pose.pose.orientation.z;
        double qw = current_odom_.pose.pose.orientation.w;
        double heading = std::atan2(2.0 * (qw * qz + qx * qy),
                                    1.0 - 2.0 * (qy * qy + qz * qz));
        
        eva_planning::Point2D current_pos(current_x, current_y);
        
        auto local_traj = spline_planner_->generateLocalTrajectory(
            current_pos, heading, global_path_, detected_obstacles_);
        
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "odom";
        
        for (const auto& point : local_traj) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        
        local_path_pub_->publish(path_msg);
    }
    
    void publishStatus(const std::string& status) {
        std_msgs::msg::String msg;
        msg.data = status;
        status_pub_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EVAPlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
