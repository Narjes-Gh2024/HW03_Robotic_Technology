#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <mutex>

namespace robot_slam
{

class MapSaverNode : public rclcpp::Node
{
public:
    MapSaverNode()
        : Node("map_saver_node")
    {
        this->declare_parameter<std::string>("map_topic", "/map");
        this->declare_parameter<std::string>("output_dir", ".");
        this->declare_parameter<std::string>("map_name", "slam_map");

        map_topic_ = this->get_parameter("map_topic").as_string();
        output_dir_ = this->get_parameter("output_dir").as_string();
        map_name_ = this->get_parameter("map_name").as_string();

        std::filesystem::create_directories(output_dir_);

        auto map_qos = rclcpp::QoS(10).reliable().durability_volatile();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, map_qos,
            std::bind(&MapSaverNode::mapCallback, this, std::placeholders::_1));

        // Set up a timer to save the map every 5 seconds
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&MapSaverNode::saveMapAutomatically, this));

        RCLCPP_INFO(this->get_logger(), "Map Saver Node initialized");
        RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", map_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output directory: %s", output_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Map name: %s", map_name_.c_str());
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_map_ = msg;
        if (!map_received_) {
            map_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Map received (%d x %d cells at %.3f m/cell)",
                        msg->info.width, msg->info.height, msg->info.resolution);
        }
    }

    void saveMapAutomatically()
    {
        bool ok = saveMap();
        if (ok) {
            RCLCPP_INFO(this->get_logger(), "Map saved successfully");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to save map (no map received or file error)");
        }
    }

    bool saveMap()
    {
        std::lock_guard<std::mutex> lock(map_mutex_);

        if (!current_map_) {
            RCLCPP_WARN(this->get_logger(), "No map available to save");
            return false;
        }

        std::string pgm_path = output_dir_ + "/" + map_name_ + ".pgm";
        std::string yaml_path = output_dir_ + "/" + map_name_ + ".yaml";

        if (!savePGM(pgm_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save PGM file");
            return false;
        }

        if (!saveYAML(yaml_path, map_name_ + ".pgm")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save YAML file");
            return false;
        }

        save_count_++;
        RCLCPP_INFO(this->get_logger(), "Map saved successfully (save #%d)", save_count_);
        RCLCPP_INFO(this->get_logger(), "  PGM: %s", pgm_path.c_str());
        RCLCPP_INFO(this->get_logger(), "  YAML: %s", yaml_path.c_str());

        return true;
    }

    bool savePGM(const std::string & filename)
    {
        if (!current_map_) return false;

        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
            return false;
        }

        int width = static_cast<int>(current_map_->info.width);
        int height = static_cast<int>(current_map_->info.height);

        file << "P5\n";
        file << "# SLAM generated map\n";
        file << width << " " << height << "\n";
        file << "255\n";

        std::vector<unsigned char> image_data(width * height);

        for (int y = height - 1; y >= 0; --y) {
            for (int x = 0; x < width; ++x) {
                int map_idx = y * width + x;
                int img_idx = (height - 1 - y) * width + x;

                int8_t occupancy = current_map_->data[map_idx];

                unsigned char pixel;
                if (occupancy == -1) {
                    pixel = 205;
                } else if (occupancy == 0) {
                    pixel = 254;
                } else if (occupancy >= 100) {
                    pixel = 0;
                } else {
                    pixel = static_cast<unsigned char>(255 - (occupancy * 255 / 100));
                }

                image_data[img_idx] = pixel;
            }
        }

        file.write(reinterpret_cast<const char*>(image_data.data()), image_data.size());
        file.close();
        return true;
    }

    bool saveYAML(const std::string & filename, const std::string & pgm_filename)
    {
        if (!current_map_) return false;

        std::ofstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
            return false;
        }

        file << std::fixed << std::setprecision(6);
        file << "image: " << pgm_filename << "\n";
        file << "resolution: " << current_map_->info.resolution << "\n";
        file << "origin: ["
             << current_map_->info.origin.position.x << ", "
             << current_map_->info.origin.position.y << ", "
             << "0.0]\n";
        file << "negate: 0\n";
        file << "occupied_thresh: 0.65\n";
        file << "free_thresh: 0.196\n";

        file.close();
        return true;
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;  // Timer to save the map periodically

    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    std::mutex map_mutex_;
    bool map_received_ = false;

    std::string map_topic_;
    std::string output_dir_;
    std::string map_name_;

    int save_count_ = 0;
};

}  // namespace robot_slam

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_slam::MapSaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
