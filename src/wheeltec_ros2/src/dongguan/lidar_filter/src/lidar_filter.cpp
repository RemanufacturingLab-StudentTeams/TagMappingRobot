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

        // Abonneer op de ruwe LiDAR data
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanFilter::callback, this, std::placeholders::_1));

        // Publiceer de gefilterde scan
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scanned", 10);

        RCLCPP_INFO(this->get_logger(),
                    "ScanFilter actief — blindspot van %.1f° tot %.1f°",
                    blind_1_start_ * 180.0 / M_PI,
                    blind_1_end_ * 180.0 / M_PI);
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Callback function started!");

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
