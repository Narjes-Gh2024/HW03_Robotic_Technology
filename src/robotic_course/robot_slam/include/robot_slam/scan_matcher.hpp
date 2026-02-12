#ifndef ROBOT_SLAM__SCAN_MATCHER_HPP_
#define ROBOT_SLAM__SCAN_MATCHER_HPP_

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "robot_slam/occupancy_grid_map.hpp"

namespace robot_slam
{

/**
 * @brief Scan-to-Map matching for pose correction
 * 
 * This class implements a correlation-based scan matching algorithm
 * using gradient descent optimization to find the best pose that
 * aligns the current scan with the existing map.
 */
class ScanMatcher
{
public:
    /**
     * @brief Constructor
     * @param max_iterations Maximum optimization iterations
     * @param convergence_threshold Convergence threshold for pose change
     * @param search_window_linear Linear search window in meters
     * @param search_window_angular Angular search window in radians
     */
    ScanMatcher(int max_iterations = 20,
                double convergence_threshold = 0.001,
                double search_window_linear = 0.5,
                double search_window_angular = 0.2);

    /**
     * @brief Match scan to map and return corrected pose
     * @param initial_pose Initial pose estimate (x, y, theta)
     * @param scan_ranges Laser scan range measurements
     * @param angle_min Minimum scan angle
     * @param angle_increment Angle increment between rays
     * @param range_min Minimum valid range
     * @param range_max Maximum valid range
     * @param map Reference to the occupancy grid map
     * @param corrected_pose Output corrected pose
     * @return True if matching succeeded
     */
    bool match(const Eigen::Vector3d & initial_pose,
               const std::vector<float> & scan_ranges,
               float angle_min, float angle_increment,
               float range_min, float range_max,
               const OccupancyGridMap & map,
               Eigen::Vector3d & corrected_pose);

    /**
     * @brief Set parameters
     */
    void setMaxIterations(int iterations) { max_iterations_ = iterations; }
    void setConvergenceThreshold(double threshold) { convergence_threshold_ = threshold; }

private:
    /**
     * @brief Compute score for a given pose (sum of occupancy values at scan points)
     * @param pose Pose to evaluate
     * @param scan_points Scan points in robot frame
     * @param map Reference to occupancy map
     * @return Score (higher is better match)
     */
    double computeScore(const Eigen::Vector3d & pose,
                        const std::vector<Eigen::Vector2d> & scan_points,
                        const OccupancyGridMap & map) const;

    /**
     * @brief Compute score gradient using finite differences
     * @param pose Current pose
     * @param scan_points Scan points in robot frame
     * @param map Reference to occupancy map
     * @return Gradient vector (dx, dy, dtheta)
     */
    Eigen::Vector3d computeGradient(const Eigen::Vector3d & pose,
                                    const std::vector<Eigen::Vector2d> & scan_points,
                                    const OccupancyGridMap & map) const;

    /**
     * @brief Transform scan points from robot frame to world frame
     * @param scan_points_robot Scan points in robot frame
     * @param pose Robot pose (x, y, theta)
     * @return Scan points in world frame
     */
    std::vector<Eigen::Vector2d> transformScanPoints(
        const std::vector<Eigen::Vector2d> & scan_points_robot,
        const Eigen::Vector3d & pose) const;

    /**
     * @brief Convert laser scan to points in robot frame
     * @param scan_ranges Range measurements
     * @param angle_min Minimum angle
     * @param angle_increment Angle increment
     * @param range_min Minimum range
     * @param range_max Maximum range
     * @return Vector of 2D points in robot frame
     */
    std::vector<Eigen::Vector2d> scanToPoints(
        const std::vector<float> & scan_ranges,
        float angle_min, float angle_increment,
        float range_min, float range_max) const;

    /**
     * @brief Get interpolated occupancy value (bilinear interpolation)
     */
    double getInterpolatedOccupancy(double x, double y,
                                    const OccupancyGridMap & map) const;

    // Parameters
    int max_iterations_;
    double convergence_threshold_;
    double search_window_linear_;
    double search_window_angular_;

    // Optimization step sizes
    double step_size_linear_ = 0.05;
    double step_size_angular_ = 0.02;

    // Finite difference delta for gradient computation
    const double DELTA_LINEAR_ = 0.01;
    const double DELTA_ANGULAR_ = 0.005;
};

}  // namespace robot_slam

#endif  // ROBOT_SLAM__SCAN_MATCHER_HPP_
