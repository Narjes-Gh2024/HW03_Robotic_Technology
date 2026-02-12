#ifndef ROBOT_SLAM__OCCUPANCY_GRID_MAP_HPP_
#define ROBOT_SLAM__OCCUPANCY_GRID_MAP_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace robot_slam
{

/**
 * @brief 2D Occupancy Grid Map using log-odds representation
 * 
 * This class manages the occupancy grid map with:
 * - Log-odds probability representation for numerical stability
 * - Bresenham's line algorithm for ray tracing
 * - Proper handling of free, occupied, and unknown cells
 */
class OccupancyGridMap
{
public:
    /**
     * @brief Constructor
     * @param resolution Map resolution in meters/cell
     * @param width Map width in meters
     * @param height Map height in meters
     * @param origin_x X coordinate of map origin
     * @param origin_y Y coordinate of map origin
     */
    OccupancyGridMap(double resolution, double width, double height,
                     double origin_x, double origin_y);

    /**
     * @brief Update map with a laser scan measurement
     * @param robot_x Robot x position in world frame
     * @param robot_y Robot y position in world frame
     * @param robot_theta Robot orientation in world frame
     * @param ranges Laser range measurements
     * @param angle_min Minimum angle of laser scan
     * @param angle_max Maximum angle of laser scan
     * @param angle_increment Angle increment between rays
     * @param range_min Minimum valid range
     * @param range_max Maximum valid range
     */
    void updateWithScan(double robot_x, double robot_y, double robot_theta,
                        const std::vector<float> & ranges,
                        float angle_min, float angle_max, float angle_increment,
                        float range_min, float range_max);

    /**
     * @brief Get occupancy probability at world coordinates
     * @param x World x coordinate
     * @param y World y coordinate
     * @return Occupancy probability [0, 1] or -1 if out of bounds
     */
    double getOccupancy(double x, double y) const;

    /**
     * @brief Get occupancy probability at grid cell
     * @param grid_x Grid x index
     * @param grid_y Grid y index
     * @return Occupancy probability [0, 1] or -1 if out of bounds
     */
    double getOccupancyCell(int grid_x, int grid_y) const;

    /**
     * @brief Convert to ROS OccupancyGrid message
     * @param frame_id Frame ID for the map
     * @param stamp Timestamp for the message
     * @return OccupancyGrid message
     */
    nav_msgs::msg::OccupancyGrid toOccupancyGridMsg(
        const std::string & frame_id,
        const rclcpp::Time & stamp) const;

    /**
     * @brief World to grid coordinate conversion
     */
    bool worldToGrid(double wx, double wy, int & gx, int & gy) const;

    /**
     * @brief Grid to world coordinate conversion
     */
    void gridToWorld(int gx, int gy, double & wx, double & wy) const;

    // Getters
    double getResolution() const { return resolution_; }
    int getWidthCells() const { return width_cells_; }
    int getHeightCells() const { return height_cells_; }
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }

    // Get raw log-odds grid (for scan matching)
    const std::vector<double> & getLogOddsGrid() const { return log_odds_grid_; }

private:
    /**
     * @brief Update a single cell with log-odds
     * @param grid_x Grid x index
     * @param grid_y Grid y index
     * @param log_odds_update Log-odds value to add
     */
    void updateCell(int grid_x, int grid_y, double log_odds_update);

    /**
     * @brief Trace a ray from robot to endpoint and update cells
     * @param x0 Start x (robot position in grid)
     * @param y0 Start y (robot position in grid)
     * @param x1 End x (hit point in grid)
     * @param y1 End y (hit point in grid)
     * @param hit True if ray hit obstacle, false if max range
     */
    void traceRay(int x0, int y0, int x1, int y1, bool hit);

    /**
     * @brief Convert log-odds to probability
     */
    double logOddsToProbability(double log_odds) const;

    /**
     * @brief Convert probability to log-odds
     */
    double probabilityToLogOdds(double prob) const;

    // Map parameters
    double resolution_;
    int width_cells_;
    int height_cells_;
    double origin_x_;
    double origin_y_;

    // Log-odds grid storage
    std::vector<double> log_odds_grid_;

    // Log-odds update values
    const double LOG_ODDS_FREE_ = -0.4;      // Probability ~0.4 that cell is occupied
    const double LOG_ODDS_OCCUPIED_ = 0.85;  // Probability ~0.7 that cell is occupied
    const double LOG_ODDS_PRIOR_ = 0.0;      // Prior (unknown)
    const double LOG_ODDS_MAX_ = 5.0;        // Clamp maximum
    const double LOG_ODDS_MIN_ = -5.0;       // Clamp minimum
};

}  // namespace robot_slam

#endif  // ROBOT_SLAM__OCCUPANCY_GRID_MAP_HPP_
