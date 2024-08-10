#ifndef AVOIDANCENODE_HPP
#define AVOIDANCENODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <chrono>

class AvoidanceNode : public rclcpp::Node
{
public:
    AvoidanceNode();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void publish_marker(double vector[2], const std::string& ns, int id, float r, float g, float b);
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double distance_threshold;
    double k_obstacle;

};

#endif 
