#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("patrol_node")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::laser_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(100ms, std::bind(&Patrol::control_loop, this));
        RCLCPP_INFO(this->get_logger(), "Real Robot Autonomous Patrol System Initialized.");
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { latest_scan_ = msg; }

    void control_loop()
    {
        if (!latest_scan_) return;
        auto vel_msg = geometry_msgs::msg::Twist();
        int total_rays = latest_scan_->ranges.size();
        double angle_increment = latest_scan_->angle_increment;
        double current_angle = latest_scan_->angle_min;

        bool obstacle_detected = false;
        double max_valid_distance = -1.0;
        double safest_angle = 0.0;

        for (int i = 0; i < total_rays; ++i) {
            double r = latest_scan_->ranges[i];
            if (!std::isnan(r) && !std::isinf(r) && r >= latest_scan_->range_min && r <= latest_scan_->range_max) {
                if (current_angle >= -M_PI_2 && current_angle <= M_PI_2) {
                    if (r > max_valid_distance) {
                        max_valid_distance = r;
                        safest_angle = current_angle;
                    }
                }
                if (current_angle >= (-20.0 * M_PI / 180.0) && current_angle <= (20.0 * M_PI / 180.0)) {
                    if (r < 0.35) obstacle_detected = true;
                }
            }
            current_angle += angle_increment;
        }

        if (!obstacle_detected) {
            vel_msg.linear.x = 0.1;
            vel_msg.angular.z = 0.0;
        } else {
            vel_msg.linear.x = 0.05;
            vel_msg.angular.z = (safest_angle >= 0.0) ? 0.5 : -0.5;
        }
        cmd_vel_pub_->publish(vel_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
