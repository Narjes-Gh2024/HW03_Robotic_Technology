#include "robot_slam/motion_model.hpp"

namespace robot_slam
{

MotionModel::MotionModel(double alpha1, double alpha2, double alpha3, double alpha4)
    : alpha1_(alpha1), alpha2_(alpha2), alpha3_(alpha3), alpha4_(alpha4)
{
}

Eigen::Vector3d MotionModel::update(const Eigen::Vector3d & prev_odom,
                                    const Eigen::Vector3d & curr_odom,
                                    const Eigen::Vector3d & prev_pose) const
{
    // Get relative motion from odometry
    Eigen::Vector3d relative_motion = getRelativeMotion(prev_odom, curr_odom);

    double delta_rot1 = relative_motion.x();
    double delta_trans = relative_motion.y();
    double delta_rot2 = relative_motion.z();

    // Apply motion to previous pose
    Eigen::Vector3d new_pose;
    new_pose.x() = prev_pose.x() + delta_trans * std::cos(prev_pose.z() + delta_rot1);
    new_pose.y() = prev_pose.y() + delta_trans * std::sin(prev_pose.z() + delta_rot1);
    new_pose.z() = normalizeAngle(prev_pose.z() + delta_rot1 + delta_rot2);

    return new_pose;
}

Eigen::Vector3d MotionModel::getRelativeMotion(const Eigen::Vector3d & from,
                                               const Eigen::Vector3d & to) const
{
    // Compute relative motion (odometry model)
    double dx = to.x() - from.x();
    double dy = to.y() - from.y();
    double dtheta = normalizeAngle(to.z() - from.z());

    double delta_trans = std::sqrt(dx * dx + dy * dy);
    
    double delta_rot1;
    if (delta_trans < 1e-6) {
        // Very small translation, rotation is ambiguous
        delta_rot1 = 0.0;
    } else {
        delta_rot1 = normalizeAngle(std::atan2(dy, dx) - from.z());
    }
    
    double delta_rot2 = normalizeAngle(dtheta - delta_rot1);

    return Eigen::Vector3d(delta_rot1, delta_trans, delta_rot2);
}

void MotionModel::setNoiseParams(double alpha1, double alpha2, double alpha3, double alpha4)
{
    alpha1_ = alpha1;
    alpha2_ = alpha2;
    alpha3_ = alpha3;
    alpha4_ = alpha4;
}

double MotionModel::normalizeAngle(double angle) const
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

}  // namespace robot_slam
