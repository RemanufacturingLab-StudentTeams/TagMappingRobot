/*
 * Used to filter lidar data.
 *
 * @author Wouter van Velzen
 * @author Jasper Waaijer
 */
#include <cmath>
#include <memory>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanFilter : public rclcpp::Node
{
public:
    ScanFilter() : Node("scan_filter")
    {
        // Blindspot in Radian. Change to the angle you need.
        // When changing these values you need to run "colcon build" to recompile
        blind_1_start_ = 317.0 * M_PI / 180.0;  // Start blindspot (in degrees)
        blind_1_end_ = 337.0 * M_PI / 180.0;    // End blindstpot (in degrees)

        blind_2_start_ = 23.0 * M_PI / 180.0;
        blind_2_end_ = 43.0 * M_PI / 180.0;

        std::string sub_topic = "/scan";
        std::string pub_topic = "/scanned";

        // Abonneer op de ruwe LiDAR data
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            sub_topic, 10, std::bind(&ScanFilter::callback, this, std::placeholders::_1));

        // Publiceer de gefilterde scan
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(pub_topic, 10);

        RCLCPP_INFO(this->get_logger(),
                    "LiDAR filter active — blindspot1 from %.1f° to %.1f°, blindspot 2 from %.1f° to %.1f°",
                    blind_1_start_ * 180.0 / M_PI,
                    blind_1_end_ * 180.0 / M_PI,
                    blind_2_start_ * 180.0 / M_PI,
                    blind_2_end_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(),
                    "Listening on topic: %s, broadcasting on topic: %s",
                    sub_topic.c_str(),
                    pub_topic.c_str());
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto filtered = sensor_msgs::msg::LaserScan();
        filtered.header = msg->header;
        filtered.angle_min = msg->angle_min;
        filtered.angle_max = msg->angle_max;
        filtered.angle_increment = msg->angle_increment;
        filtered.time_increment = msg->time_increment;
        filtered.scan_time = msg->scan_time;
        filtered.range_min = msg->range_min;
        filtered.range_max = msg->range_max;
        filtered.ranges = msg->ranges;

        float angle = msg->angle_min;

        // Loop om blindspots blind te maken
        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            //blindspot 1
            if (angle >= blind_1_start_ && angle <= blind_1_end_)
            {
                filtered.ranges[i] = std::numeric_limits<float>::infinity();  // Sets range data to infinite if it should not see anything
            } else if (angle >= blind_2_start_ && angle <= blind_2_end_) //blindspot 2
            {
                filtered.ranges[i] = std::numeric_limits<float>::infinity();  // Sets range data to infinite if it should not see anything
            }
            angle += msg->angle_increment;
        }

        publisher_->publish(filtered);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    double blind_1_start_;
    double blind_1_end_;
    double blind_2_start_;
    double blind_2_end_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanFilter>());
    rclcpp::shutdown();
    return 0;
}
