#ifndef AVOIDANCENODE_HPP
#define AVOIDANCENODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <chrono>
#include <array>
#include <vector>
#include <tuple>


class AvoidanceNode : public rclcpp::Node
{
public:
    AvoidanceNode();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);    
    void timer_callback();
    void update_timer(double new_frequency);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publish_markers_array(const std::vector<visualization_msgs::msg::Marker>& markers_array);
    visualization_msgs::msg::Marker create_marker(double vector[2], const std::string& ns, int id, float r, float g, float b);

    void vff_algorithm();
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;  // Subscription for Odometry
    rclcpp::TimerBase::SharedPtr timer;
    sensor_msgs::msg::LaserScan::SharedPtr laser_scan_; // Class member variable
    rclcpp::Time last_scan_time_; // New member for tracking the last scan time
    double distance_threshold;
    double k_obstacle;
    double speed;
    bool initial_orientation_received = false;  // To check if initial orientation is received
    double initial_yaw = 0.0;  // To store the initial yaw
    double current_yaw = 0.0;
    double alignment_threshold = 0.0;
    double frequency;
    
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

};

#endif 
