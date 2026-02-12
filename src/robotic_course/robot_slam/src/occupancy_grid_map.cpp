#include "robot_slam/occupancy_grid_map.hpp"
#include <cstring>

namespace robot_slam
{

OccupancyGridMap::OccupancyGridMap(double resolution, double width, double height,
                                   double origin_x, double origin_y)
    : resolution_(resolution),
      origin_x_(origin_x),
      origin_y_(origin_y)
{
    // Calculate grid dimensions
    width_cells_ = static_cast<int>(std::ceil(width / resolution));
    height_cells_ = static_cast<int>(std::ceil(height / resolution));

    // Initialize grid with unknown (0 log-odds = 0.5 probability)
    log_odds_grid_.resize(width_cells_ * height_cells_, LOG_ODDS_PRIOR_);
}

void OccupancyGridMap::updateWithScan(double robot_x, double robot_y, double robot_theta,
                                      const std::vector<float> & ranges,
                                      float angle_min, float /* angle_max */, float angle_increment,
                                      float range_min, float range_max)
{
    // Convert robot position to grid coordinates
    int robot_gx, robot_gy;
    if (!worldToGrid(robot_x, robot_y, robot_gx, robot_gy)) {
        return;  // Robot is outside map
    }

    // Process each laser ray
    for (size_t i = 0; i < ranges.size(); ++i) {
        float range = ranges[i];
        float angle = angle_min + i * angle_increment;

        // Skip invalid measurements
        if (std::isnan(range) || std::isinf(range)) {
            continue;
        }

        // Check if measurement is within valid range
        bool hit = (range >= range_min && range < range_max);
        
        // If range is at max or beyond, we know the ray is free up to max range
        // but we don't know what's beyond
        double effective_range = hit ? range : range_max;

        // Calculate endpoint in world coordinates
        double world_angle = robot_theta + angle;
        double end_x = robot_x + effective_range * std::cos(world_angle);
        double end_y = robot_y + effective_range * std::sin(world_angle);

        // Convert endpoint to grid coordinates
        int end_gx, end_gy;
        if (!worldToGrid(end_x, end_y, end_gx, end_gy)) {
            // Endpoint is outside map, clip to map boundary
            end_gx = std::max(0, std::min(width_cells_ - 1, end_gx));
            end_gy = std::max(0, std::min(height_cells_ - 1, end_gy));
        }

        // Trace ray and update cells
        traceRay(robot_gx, robot_gy, end_gx, end_gy, hit);
    }
}

void OccupancyGridMap::traceRay(int x0, int y0, int x1, int y1, bool hit)
{
    // Bresenham's line algorithm for ray tracing
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int x = x0;
    int y = y0;

    while (true) {
        // Check if we've reached the endpoint
        bool at_endpoint = (x == x1 && y == y1);

        if (at_endpoint) {
            // At endpoint: mark as occupied if hit, otherwise leave as is
            if (hit) {
                updateCell(x, y, LOG_ODDS_OCCUPIED_);
            }
            break;
        } else {
            // Along the ray: mark as free
            updateCell(x, y, LOG_ODDS_FREE_);
        }

        // Bresenham step
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }

        // Safety check to avoid infinite loop
        if (std::abs(x - x0) > width_cells_ || std::abs(y - y0) > height_cells_) {
            break;
        }
    }
}

void OccupancyGridMap::updateCell(int grid_x, int grid_y, double log_odds_update)
{
    // Bounds check
    if (grid_x < 0 || grid_x >= width_cells_ || grid_y < 0 || grid_y >= height_cells_) {
        return;
    }

    int index = grid_y * width_cells_ + grid_x;
    
    // Update log-odds
    log_odds_grid_[index] += log_odds_update;
    
    // Clamp to prevent saturation
    log_odds_grid_[index] = std::max(LOG_ODDS_MIN_, 
                                      std::min(LOG_ODDS_MAX_, log_odds_grid_[index]));
}

double OccupancyGridMap::getOccupancy(double x, double y) const
{
    int gx, gy;
    if (!worldToGrid(x, y, gx, gy)) {
        return -1.0;  // Out of bounds
    }
    return getOccupancyCell(gx, gy);
}

double OccupancyGridMap::getOccupancyCell(int grid_x, int grid_y) const
{
    if (grid_x < 0 || grid_x >= width_cells_ || grid_y < 0 || grid_y >= height_cells_) {
        return -1.0;  // Out of bounds
    }

    int index = grid_y * width_cells_ + grid_x;
    return logOddsToProbability(log_odds_grid_[index]);
}

bool OccupancyGridMap::worldToGrid(double wx, double wy, int & gx, int & gy) const
{
    gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));

    return (gx >= 0 && gx < width_cells_ && gy >= 0 && gy < height_cells_);
}

void OccupancyGridMap::gridToWorld(int gx, int gy, double & wx, double & wy) const
{
    wx = origin_x_ + (gx + 0.5) * resolution_;
    wy = origin_y_ + (gy + 0.5) * resolution_;
}

double OccupancyGridMap::logOddsToProbability(double log_odds) const
{
    return 1.0 / (1.0 + std::exp(-log_odds));
}

double OccupancyGridMap::probabilityToLogOdds(double prob) const
{
    // Clamp probability to avoid log(0)
    prob = std::max(0.001, std::min(0.999, prob));
    return std::log(prob / (1.0 - prob));
}

nav_msgs::msg::OccupancyGrid OccupancyGridMap::toOccupancyGridMsg(
    const std::string & frame_id,
    const rclcpp::Time & stamp) const
{
    nav_msgs::msg::OccupancyGrid msg;

    // Header
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;

    // Map metadata
    msg.info.resolution = resolution_;
    msg.info.width = width_cells_;
    msg.info.height = height_cells_;
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    // Convert log-odds to occupancy values [0-100] or -1 for unknown
    msg.data.resize(width_cells_ * height_cells_);
    
    for (int i = 0; i < width_cells_ * height_cells_; ++i) {
        double log_odds = log_odds_grid_[i];
        
        // Check if cell is still near unknown
        if (std::abs(log_odds) < 0.1) {
            msg.data[i] = -1;  // Unknown
        } else {
            double prob = logOddsToProbability(log_odds);
            msg.data[i] = static_cast<int8_t>(prob * 100.0);
        }
    }

    return msg;
}

}  // namespace robot_slam
