#ifndef ROBOT_SLAM__SLAM_NODE_HPP_
#define ROBOT_SLAM__SLAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include <memory>
#include <vector>
#include <cmath>
#include <string>
#include <utility>

#include "robot_slam/occupancy_grid_map.hpp"
#include "robot_slam/scan_matcher.hpp"
#include "robot_slam/motion_model.hpp"

namespace robot_slam
{

struct Pose2D
{
    double x{0.0};
    double y{0.0};
    double theta{0.0};

    Pose2D() = default;
    Pose2D(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}

    void normalize()
    {
        while (theta > M_PI) theta -= 2.0 * M_PI;
        while (theta < -M_PI) theta += 2.0 * M_PI;
    }
};

class SlamNode : public rclcpp::Node
{
public:
    explicit SlamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~SlamNode() override = default;

private:
    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timerCallback();

    // SLAM steps
    void processMotionUpdate(const nav_msgs::msg::Odometry::SharedPtr odom);
    void processMeasurementUpdate(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void updateMap(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Publishing
    void publishMap();
    void publishPose();
    void publishTransform();

    // Helper
    std::vector<std::pair<double, double>> scanToPoints(
        const sensor_msgs::msg::LaserScan::SharedPtr scan,
        const Pose2D & pose);

    // ROS I/O
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // SLAM components
    std::unique_ptr<OccupancyGridMap> map_;
    std::unique_ptr<ScanMatcher> scan_matcher_;
    std::unique_ptr<MotionModel> motion_model_;

    // State
    Pose2D current_pose_;

    // Odom cache
    Pose2D previous_odom_pose_;
    Pose2D latest_odom_pose_;

    bool first_odom_received_{false};
    bool first_scan_received_{false};

    // Mutexes
    std::mutex pose_mutex_;
    std::mutex map_mutex_;
    std::mutex odom_mutex_;

    // Params
    std::string odom_topic_;
    std::string scan_topic_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;

    double map_resolution_{0.05};
    double map_width_{50.0};
    double map_height_{50.0};
    double map_origin_x_{-25.0};
    double map_origin_y_{-25.0};
    double map_update_interval_{1.0};

    bool use_scan_matching_{true};
    int scan_matching_iterations_{20};
};

}  // namespace robot_slam

#endif  // ROBOT_SLAM__SLAM_NODE_HPP_
