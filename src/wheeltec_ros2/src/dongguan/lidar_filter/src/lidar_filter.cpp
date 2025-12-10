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
        // Blindspot (in radians) – pas aan wat je wilt
        blind_start_ = 160.0 * M_PI / 180.0;  // beginhoek blindspot
        blind_end_ = 240.0 * M_PI / 180.0;    // eindhoek blindspot

        // Abonneer op de ruwe LiDAR data
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanFilter::callback, this, std::placeholders::_1));

        // Publiceer de gefilterde scan
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scanned", 10);

        RCLCPP_INFO(this->get_logger(),
                    "ScanFilter actief — blindspot van %.1f° tot %.1f°",
                    blind_start_ * 180.0 / M_PI,
                    blind_end_ * 180.0 / M_PI);
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
            if (angle >= blind_start_ && angle <= blind_end_)
            {
                filtered.ranges[i] = std::numeric_limits<float>::infinity();  // niets gezien
            }
            angle += msg->angle_increment;
        }

        publisher_->publish(filtered);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    double blind_start_;
    double blind_end_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanFilter>());
    rclcpp::shutdown();
    return 0;
}
