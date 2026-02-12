#include "robot_slam/slam_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <mutex>
#include <vector>

namespace robot_slam
{

static inline double normalizeAngle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

SlamNode::SlamNode(const rclcpp::NodeOptions & options)
    : Node("slam_node", options)
{
    // Parameters
    this->declare_parameter<std::string>("odom_topic", "/ekf/odom");
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<double>("map_resolution", 0.05);
    this->declare_parameter<double>("map_width", 50.0);
    this->declare_parameter<double>("map_height", 50.0);
    this->declare_parameter<double>("map_origin_x", -25.0);
    this->declare_parameter<double>("map_origin_y", -25.0);
    this->declare_parameter<double>("map_update_interval", 1.0);
    this->declare_parameter<bool>("use_scan_matching", true);
    this->declare_parameter<int>("scan_matching_iterations", 20);

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    map_resolution_ = this->get_parameter("map_resolution").as_double();
    map_width_ = this->get_parameter("map_width").as_double();
    map_height_ = this->get_parameter("map_height").as_double();
    map_origin_x_ = this->get_parameter("map_origin_x").as_double();
    map_origin_y_ = this->get_parameter("map_origin_y").as_double();
    map_update_interval_ = this->get_parameter("map_update_interval").as_double();

    use_scan_matching_ = this->get_parameter("use_scan_matching").as_bool();
    scan_matching_iterations_ = this->get_parameter("scan_matching_iterations").as_int();

    // Components
    map_ = std::make_unique<OccupancyGridMap>(
        map_resolution_, map_width_, map_height_, map_origin_x_, map_origin_y_);

    scan_matcher_ = std::make_unique<ScanMatcher>(scan_matching_iterations_);
    motion_model_ = std::make_unique<MotionModel>();

    // State init
    current_pose_ = Pose2D{};
    previous_odom_pose_ = Pose2D{};
    latest_odom_pose_ = Pose2D{};

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Publishers
    auto map_qos = rclcpp::QoS(10).reliable().durability_volatile();
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);

    // Subscribers
    auto sensor_qos = rclcpp::QoS(10).best_effort().durability_volatile();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, sensor_qos,
        std::bind(&SlamNode::odomCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, sensor_qos,
        std::bind(&SlamNode::scanCallback, this, std::placeholders::_1));

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(map_update_interval_),
        std::bind(&SlamNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "SLAM Node initialized");
    RCLCPP_INFO(this->get_logger(), "  Odom topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Scan topic: %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Base frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Map size: %.1f x %.1f m at %.3f m/cell",
                map_width_, map_height_, map_resolution_);
    RCLCPP_INFO(this->get_logger(), "  Scan matching: %s", use_scan_matching_ ? "enabled" : "disabled");
}

void SlamNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    Pose2D odom_pose;
    odom_pose.x = msg->pose.pose.position.x;
    odom_pose.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    odom_pose.theta = yaw;

    // Always keep latest odom
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_pose_ = odom_pose;
    }

    if (!first_odom_received_) {
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            previous_odom_pose_ = odom_pose;
        }
        first_odom_received_ = true;
        RCLCPP_INFO(this->get_logger(), "First odometry received at (%.2f, %.2f, %.2f)",
                    odom_pose.x, odom_pose.y, odom_pose.theta);
        return;
    }

    // Motion update uses delta(prev_odom -> curr_odom)
    processMotionUpdate(msg);

    // Update previous odom AFTER using it
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        previous_odom_pose_ = odom_pose;
    }
}

void SlamNode::processMotionUpdate(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    Pose2D odom_pose;
    odom_pose.x = odom->pose.pose.position.x;
    odom_pose.y = odom->pose.pose.position.y;

    tf2::Quaternion q(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    odom_pose.theta = yaw;

    Pose2D prev_odom_copy;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        prev_odom_copy = previous_odom_pose_;
    }

    std::lock_guard<std::mutex> lock_pose(pose_mutex_);

    Eigen::Vector3d prev_odom(prev_odom_copy.x, prev_odom_copy.y, prev_odom_copy.theta);
    Eigen::Vector3d curr_odom(odom_pose.x, odom_pose.y, odom_pose.theta);
    Eigen::Vector3d prev_pose(current_pose_.x, current_pose_.y, current_pose_.theta);

    Eigen::Vector3d new_pose = motion_model_->update(prev_odom, curr_odom, prev_pose);

    current_pose_.x = new_pose.x();
    current_pose_.y = new_pose.y();
    current_pose_.theta = new_pose.z();
    current_pose_.normalize();
}

void SlamNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!first_odom_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for first odometry message...");
        return;
    }

    processMeasurementUpdate(msg);
    updateMap(msg);

    publishPose();
    publishTransform();

    if (!first_scan_received_) {
        first_scan_received_ = true;
        RCLCPP_INFO(this->get_logger(), "First laser scan received, SLAM is running");
    }
}

void SlamNode::processMeasurementUpdate(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    if (!use_scan_matching_) return;

    static int scan_count = 0;
    scan_count++;
    if (scan_count < 10) return;

    std::lock_guard<std::mutex> lock_pose(pose_mutex_);
    std::lock_guard<std::mutex> lock_map(map_mutex_);

    Eigen::Vector3d initial_pose(current_pose_.x, current_pose_.y, current_pose_.theta);
    std::vector<float> ranges(scan->ranges.begin(), scan->ranges.end());

    Eigen::Vector3d corrected_pose;
    bool success = scan_matcher_->match(
        initial_pose, ranges,
        scan->angle_min, scan->angle_increment,
        scan->range_min, scan->range_max,
        *map_, corrected_pose);

    if (success) {
        current_pose_.x = corrected_pose.x();
        current_pose_.y = corrected_pose.y();
        current_pose_.theta = corrected_pose.z();
        current_pose_.normalize();
    }
}

void SlamNode::updateMap(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    std::lock_guard<std::mutex> lock_pose(pose_mutex_);
    std::lock_guard<std::mutex> lock_map(map_mutex_);

    std::vector<float> ranges(scan->ranges.begin(), scan->ranges.end());

    map_->updateWithScan(
        current_pose_.x, current_pose_.y, current_pose_.theta,
        ranges,
        scan->angle_min, scan->angle_max, scan->angle_increment,
        scan->range_min, scan->range_max);
}

void SlamNode::timerCallback()
{
    publishMap();
}

void SlamNode::publishMap()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    nav_msgs::msg::OccupancyGrid map_msg = map_->toOccupancyGridMsg(map_frame_, this->now());
    map_pub_->publish(map_msg);
}

void SlamNode::publishPose()
{
    std::lock_guard<std::mutex> lock(pose_mutex_);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.header.stamp = this->now();

    pose_msg.pose.position.x = current_pose_.x;
    pose_msg.pose.position.y = current_pose_.y;
    pose_msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, current_pose_.theta);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_pub_->publish(pose_msg);
}

// map->odom TF: T_map_odom = T_map_base ⊕ inverse(T_odom_base)
void SlamNode::publishTransform()
{
    Pose2D odom_pose_copy;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_pose_copy = latest_odom_pose_;
    }

    Pose2D slam_pose_copy;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        slam_pose_copy = current_pose_;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;

    const double xm  = slam_pose_copy.x;
    const double ym  = slam_pose_copy.y;
    const double thm = slam_pose_copy.theta;

    const double xo  = odom_pose_copy.x;
    const double yo  = odom_pose_copy.y;
    const double tho = odom_pose_copy.theta;

    // inverse(odom->base)
    const double c = std::cos(tho);
    const double s = std::sin(tho);
    const double inv_x  = -c * xo - s * yo;
    const double inv_y  =  s * xo - c * yo;
    const double inv_th = -tho;

    // compose: map->odom = (map->base) ⊕ inverse(odom->base)
    const double c_m = std::cos(thm);
    const double s_m = std::sin(thm);

    const double tx = xm + c_m * inv_x - s_m * inv_y;
    const double ty = ym + s_m * inv_x + c_m * inv_y;
    const double tyaw = normalizeAngle(thm + inv_th);

    transform.transform.translation.x = tx;
    transform.transform.translation.y = ty;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, tyaw);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transform);
}

std::vector<std::pair<double, double>> SlamNode::scanToPoints(
    const sensor_msgs::msg::LaserScan::SharedPtr scan,
    const Pose2D & pose)
{
    std::vector<std::pair<double, double>> points;
    points.reserve(scan->ranges.size());

    const double cos_theta = std::cos(pose.theta);
    const double sin_theta = std::sin(pose.theta);

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        const float range = scan->ranges[i];

        if (std::isnan(range) || std::isinf(range)) continue;
        if (range < scan->range_min || range > scan->range_max) continue;

        const double angle = scan->angle_min + i * scan->angle_increment;
        const double local_x = range * std::cos(angle);
        const double local_y = range * std::sin(angle);

        const double world_x = pose.x + cos_theta * local_x - sin_theta * local_y;
        const double world_y = pose.y + sin_theta * local_x + cos_theta * local_y;

        points.emplace_back(world_x, world_y);
    }

    return points;
}

}  // namespace robot_slam

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_slam::SlamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
