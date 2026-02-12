#ifndef ROBOT_SLAM__MOTION_MODEL_HPP_
#define ROBOT_SLAM__MOTION_MODEL_HPP_

#include <cmath>
#include <Eigen/Dense>

namespace robot_slam
{

/**
 * @brief Odometry-based motion model for differential drive robot
 * 
 * This class implements the odometry motion model that computes
 * the relative motion between two odometry readings and applies
 * it to the estimated robot pose.
 */
class MotionModel
{
public:
    /**
     * @brief Constructor
     * @param alpha1 Rotation noise from rotation
     * @param alpha2 Rotation noise from translation
     * @param alpha3 Translation noise from translation
     * @param alpha4 Translation noise from rotation
     */
    MotionModel(double alpha1 = 0.1, double alpha2 = 0.1,
                double alpha3 = 0.1, double alpha4 = 0.1);

    /**
     * @brief Compute pose update from odometry
     * @param prev_odom Previous odometry reading (x, y, theta)
     * @param curr_odom Current odometry reading (x, y, theta)
     * @param prev_pose Previous estimated pose (x, y, theta)
     * @return Updated pose estimate
     */
    Eigen::Vector3d update(const Eigen::Vector3d & prev_odom,
                           const Eigen::Vector3d & curr_odom,
                           const Eigen::Vector3d & prev_pose) const;

    /**
     * @brief Set noise parameters
     */
    void setNoiseParams(double alpha1, double alpha2, double alpha3, double alpha4);

    /**
     * @brief Get the relative transformation between two poses
     * @param from Source pose
     * @param to Target pose
     * @return Relative transformation (delta_rot1, delta_trans, delta_rot2)
     */
    Eigen::Vector3d getRelativeMotion(const Eigen::Vector3d & from,
                                      const Eigen::Vector3d & to) const;

private:
    /**
     * @brief Normalize angle to [-pi, pi]
     */
    double normalizeAngle(double angle) const;

    // Noise parameters (for potential probabilistic extensions)
    double alpha1_;  // Rotation noise from rotation
    double alpha2_;  // Rotation noise from translation
    double alpha3_;  // Translation noise from translation
    double alpha4_;  // Translation noise from rotation
};

}  // namespace robot_slam

#endif  // ROBOT_SLAM__MOTION_MODEL_HPP_
