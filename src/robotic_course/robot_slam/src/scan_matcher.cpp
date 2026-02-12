#include "robot_slam/scan_matcher.hpp"
#include <iostream>

namespace robot_slam
{

ScanMatcher::ScanMatcher(int max_iterations, double convergence_threshold,
                         double search_window_linear, double search_window_angular)
    : max_iterations_(max_iterations),
      convergence_threshold_(convergence_threshold),
      search_window_linear_(search_window_linear),
      search_window_angular_(search_window_angular)
{
}

bool ScanMatcher::match(const Eigen::Vector3d & initial_pose,
                        const std::vector<float> & scan_ranges,
                        float angle_min, float angle_increment,
                        float range_min, float range_max,
                        const OccupancyGridMap & map,
                        Eigen::Vector3d & corrected_pose)
{
    // Convert scan to points in robot frame
    std::vector<Eigen::Vector2d> scan_points_robot = scanToPoints(
        scan_ranges, angle_min, angle_increment, range_min, range_max);

    if (scan_points_robot.empty()) {
        corrected_pose = initial_pose;
        return false;
    }

    // Initialize search pose
    Eigen::Vector3d current_pose = initial_pose;
    double best_score = computeScore(current_pose, scan_points_robot, map);

    // Gradient descent optimization
    for (int iter = 0; iter < max_iterations_; ++iter) {
        // Compute gradient
        Eigen::Vector3d gradient = computeGradient(current_pose, scan_points_robot, map);

        // Adaptive step size based on gradient magnitude
        double grad_norm = gradient.head<2>().norm();
        double linear_step = step_size_linear_;
        double angular_step = step_size_angular_;

        if (grad_norm > 1.0) {
            linear_step /= grad_norm;
        }

        // Update pose in gradient direction
        Eigen::Vector3d new_pose = current_pose;
        new_pose.x() += linear_step * gradient.x();
        new_pose.y() += linear_step * gradient.y();
        new_pose.z() += angular_step * gradient.z();

        // Normalize angle
        while (new_pose.z() > M_PI) new_pose.z() -= 2.0 * M_PI;
        while (new_pose.z() < -M_PI) new_pose.z() += 2.0 * M_PI;

        // Check search window constraints
        if (std::abs(new_pose.x() - initial_pose.x()) > search_window_linear_ ||
            std::abs(new_pose.y() - initial_pose.y()) > search_window_linear_) {
            // Clip to search window
            new_pose.x() = std::max(initial_pose.x() - search_window_linear_,
                                    std::min(initial_pose.x() + search_window_linear_, new_pose.x()));
            new_pose.y() = std::max(initial_pose.y() - search_window_linear_,
                                    std::min(initial_pose.y() + search_window_linear_, new_pose.y()));
        }

        double angle_diff = new_pose.z() - initial_pose.z();
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
        if (std::abs(angle_diff) > search_window_angular_) {
            new_pose.z() = initial_pose.z() + 
                           std::copysign(search_window_angular_, angle_diff);
        }

        // Compute new score
        double new_score = computeScore(new_pose, scan_points_robot, map);

        // Check for improvement
        if (new_score > best_score) {
            // Check convergence
            Eigen::Vector3d pose_change = new_pose - current_pose;
            double change_norm = pose_change.head<2>().norm() + std::abs(pose_change.z());

            current_pose = new_pose;
            best_score = new_score;

            if (change_norm < convergence_threshold_) {
                break;  // Converged
            }
        } else {
            // Reduce step size and try again
            step_size_linear_ *= 0.5;
            step_size_angular_ *= 0.5;

            if (step_size_linear_ < 0.001) {
                break;  // Step size too small
            }
        }
    }

    // Reset step sizes for next call
    step_size_linear_ = 0.05;
    step_size_angular_ = 0.02;

    corrected_pose = current_pose;
    return true;
}

double ScanMatcher::computeScore(const Eigen::Vector3d & pose,
                                 const std::vector<Eigen::Vector2d> & scan_points_robot,
                                 const OccupancyGridMap & map) const
{
    // Transform scan points to world frame
    std::vector<Eigen::Vector2d> scan_points_world = transformScanPoints(scan_points_robot, pose);

    double score = 0.0;
    int valid_count = 0;

    for (const auto & point : scan_points_world) {
        double occ = getInterpolatedOccupancy(point.x(), point.y(), map);
        if (occ >= 0.0) {
            // Score based on occupancy - we want scan points to fall on occupied cells
            score += occ;
            valid_count++;
        }
    }

    // Normalize by number of valid points
    if (valid_count > 0) {
        score /= valid_count;
    }

    return score;
}

Eigen::Vector3d ScanMatcher::computeGradient(const Eigen::Vector3d & pose,
                                             const std::vector<Eigen::Vector2d> & scan_points_robot,
                                             const OccupancyGridMap & map) const
{
    Eigen::Vector3d gradient;

    // Compute partial derivatives using central differences
    Eigen::Vector3d pose_dx_plus = pose;
    pose_dx_plus.x() += DELTA_LINEAR_;
    Eigen::Vector3d pose_dx_minus = pose;
    pose_dx_minus.x() -= DELTA_LINEAR_;
    gradient.x() = (computeScore(pose_dx_plus, scan_points_robot, map) -
                    computeScore(pose_dx_minus, scan_points_robot, map)) / (2.0 * DELTA_LINEAR_);

    Eigen::Vector3d pose_dy_plus = pose;
    pose_dy_plus.y() += DELTA_LINEAR_;
    Eigen::Vector3d pose_dy_minus = pose;
    pose_dy_minus.y() -= DELTA_LINEAR_;
    gradient.y() = (computeScore(pose_dy_plus, scan_points_robot, map) -
                    computeScore(pose_dy_minus, scan_points_robot, map)) / (2.0 * DELTA_LINEAR_);

    Eigen::Vector3d pose_dtheta_plus = pose;
    pose_dtheta_plus.z() += DELTA_ANGULAR_;
    Eigen::Vector3d pose_dtheta_minus = pose;
    pose_dtheta_minus.z() -= DELTA_ANGULAR_;
    gradient.z() = (computeScore(pose_dtheta_plus, scan_points_robot, map) -
                    computeScore(pose_dtheta_minus, scan_points_robot, map)) / (2.0 * DELTA_ANGULAR_);

    return gradient;
}

std::vector<Eigen::Vector2d> ScanMatcher::transformScanPoints(
    const std::vector<Eigen::Vector2d> & scan_points_robot,
    const Eigen::Vector3d & pose) const
{
    std::vector<Eigen::Vector2d> scan_points_world;
    scan_points_world.reserve(scan_points_robot.size());

    double cos_theta = std::cos(pose.z());
    double sin_theta = std::sin(pose.z());

    for (const auto & point_robot : scan_points_robot) {
        Eigen::Vector2d point_world;
        point_world.x() = pose.x() + cos_theta * point_robot.x() - sin_theta * point_robot.y();
        point_world.y() = pose.y() + sin_theta * point_robot.x() + cos_theta * point_robot.y();
        scan_points_world.push_back(point_world);
    }

    return scan_points_world;
}

std::vector<Eigen::Vector2d> ScanMatcher::scanToPoints(
    const std::vector<float> & scan_ranges,
    float angle_min, float angle_increment,
    float range_min, float range_max) const
{
    std::vector<Eigen::Vector2d> points;
    points.reserve(scan_ranges.size());

    for (size_t i = 0; i < scan_ranges.size(); ++i) {
        float range = scan_ranges[i];

        // Skip invalid measurements
        if (std::isnan(range) || std::isinf(range)) {
            continue;
        }
        if (range < range_min || range > range_max) {
            continue;
        }

        float angle = angle_min + i * angle_increment;
        Eigen::Vector2d point;
        point.x() = range * std::cos(angle);
        point.y() = range * std::sin(angle);
        points.push_back(point);
    }

    return points;
}

double ScanMatcher::getInterpolatedOccupancy(double x, double y,
                                              const OccupancyGridMap & map) const
{
    // Convert to grid coordinates (continuous)
    double gx_cont = (x - map.getOriginX()) / map.getResolution() - 0.5;
    double gy_cont = (y - map.getOriginY()) / map.getResolution() - 0.5;

    // Get integer grid coordinates
    int gx0 = static_cast<int>(std::floor(gx_cont));
    int gy0 = static_cast<int>(std::floor(gy_cont));
    int gx1 = gx0 + 1;
    int gy1 = gy0 + 1;

    // Check bounds
    if (gx0 < 0 || gx1 >= map.getWidthCells() || 
        gy0 < 0 || gy1 >= map.getHeightCells()) {
        return -1.0;
    }

    // Interpolation weights
    double wx = gx_cont - gx0;
    double wy = gy_cont - gy0;

    // Get occupancy at four corners
    double occ00 = map.getOccupancyCell(gx0, gy0);
    double occ10 = map.getOccupancyCell(gx1, gy0);
    double occ01 = map.getOccupancyCell(gx0, gy1);
    double occ11 = map.getOccupancyCell(gx1, gy1);

    // Check for unknown cells
    if (occ00 < 0 || occ10 < 0 || occ01 < 0 || occ11 < 0) {
        // Return simple occupancy if any corner is unknown
        return map.getOccupancy(x, y);
    }

    // Bilinear interpolation
    double occ = (1.0 - wx) * (1.0 - wy) * occ00 +
                 wx * (1.0 - wy) * occ10 +
                 (1.0 - wx) * wy * occ01 +
                 wx * wy * occ11;

    return occ;
}

}  // namespace robot_slam
