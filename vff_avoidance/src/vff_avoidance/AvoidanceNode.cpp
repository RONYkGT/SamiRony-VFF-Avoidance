#include "../../include/vff_avoidance/AvoidanceNode.hpp"

using namespace std::chrono_literals;

AvoidanceNode::AvoidanceNode()
: Node("vff_avoidance")
{
    // Declare parameters with default values
    this->declare_parameter<double>("distance_threshold", 0.5);
    this->declare_parameter<double>("k_obstacle", -0.0025);
    this->declare_parameter<double>("speed", 0.5);  // Default speed value
    this->declare_parameter<double>("alignment_threshold", 1.0);  // Angular threshold for alignment
    this->declare_parameter<double>("frequency", 20.0);  


    // Retrieve the parameters (values from the YAML file)
    this->get_parameter("distance_threshold", distance_threshold);
    this->get_parameter("k_obstacle", k_obstacle);
    this->get_parameter("speed", speed);  
    this->get_parameter("alignment_threshold", alignment_threshold);  
    this->get_parameter("frequency", frequency); 

    RCLCPP_INFO(this->get_logger(), "robot initialized");
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&AvoidanceNode::scan_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&AvoidanceNode::odometry_callback, this, std::placeholders::_1));
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    // Add callback function whenever parameters get changed on rqt
    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&AvoidanceNode::parameters_callback, this, std::placeholders::_1));

    // Timer setup for 20Hz execution
    int period_in_milliseconds = 1000 / frequency;
    timer = this->create_wall_timer(
        std::chrono::milliseconds(period_in_milliseconds), std::bind(&AvoidanceNode::timer_callback, this));
}


rcl_interfaces::msg::SetParametersResult AvoidanceNode::parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    RCLCPP_INFO(this->get_logger(), "updating parameters");
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters)
    {
        if (param.get_name() == "distance_threshold")
        {
            distance_threshold = param.as_double();
            RCLCPP_INFO(this->get_logger(), "distance_threshold updated to: %f", distance_threshold);
        }
        else if (param.get_name() == "k_obstacle")
        {
            k_obstacle = param.as_double();
            RCLCPP_INFO(this->get_logger(), "k_obstacle updated to: %f", k_obstacle);
        }
        else if (param.get_name() == "speed")
        {
            speed = param.as_double();
            RCLCPP_INFO(this->get_logger(), "speed updated to: %f", speed);
        }
        else if (param.get_name() == "alignment_threshold")
        {
            alignment_threshold = param.as_double();
            RCLCPP_INFO(this->get_logger(), "alignment_threshold updated to: %f", alignment_threshold);
        }
    }
    return result;
}



void AvoidanceNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // We are getting the current orientation of the bot in "yaw"
    // Convert the quaternion to yaw (rotation around the Z axis)
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_yaw = yaw;
    if (!initial_orientation_received) {
        initial_yaw = yaw;
        initial_orientation_received = true;
        RCLCPP_INFO(this->get_logger(), "Initial yaw set to: %f", initial_yaw);
    }
}

void AvoidanceNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Store the laser scan data in the class member variable
    laser_scan_ = msg;
    last_scan_time_ = this->now(); // Update the last scan time
    // Check if the initial orientation has not been received or if the laser scan data is not yet available.
    if (!initial_orientation_received || !laser_scan_) {
        RCLCPP_WARN(this->get_logger(), "Orientation not yet received.");
        return;
    }

    // Calculate the adjusted yaw to rotate the target vector accordingly
    double adjusted_yaw = initial_yaw - current_yaw;
    //this makes sure the target vector is facing where it was facing initially when the bot rotates
    double target_vector[2] = {cos(adjusted_yaw), sin(adjusted_yaw)}; 

    // Add all the vectors below the threshold inside the obstacle vector
    double min_distance = distance_threshold;
    double obstacle_vector[2] = {0, 0};
    size_t index = 0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        if (msg->ranges[i] < distance_threshold) {
            double angle = msg->angle_min + i * msg->angle_increment;
            double x = k_obstacle * (1 / pow(msg->ranges[i],2)) * cos(angle);
            double y = k_obstacle * (1 / pow(msg->ranges[i],2)) * sin(angle);
            obstacle_vector[0] += x;
            obstacle_vector[1] += y;
            if (msg->ranges[i] < min_distance)
                min_distance = msg->ranges[i]; //keep track of closest obstacle
        }
    }


    // Combine target and obstacle vectors
    double result_vector[2];
    result_vector[0] = target_vector[0] + obstacle_vector[0];
    result_vector[1] = target_vector[1] + obstacle_vector[1];


    // Check if obstacle and target vectors are nearly opposite
    double dot_product = target_vector[0] * obstacle_vector[0] + target_vector[1] * obstacle_vector[1];
    if (dot_product < 0 && fabs(dot_product) >= 0.99) { 
        // Add a small bias to the result vector
        result_vector[0] += 0.3;
        result_vector[1] += 0.3;
    }

    // Normalise the result vector so it doesnt go higher than 1.0 in length (too fast) because of adding too many obstacle vectors
    double magnitude = sqrt(pow(result_vector[0], 2) + pow(result_vector[1], 2));
    if (magnitude > 1.0) {
        result_vector[0] /= magnitude;
        result_vector[1] /= magnitude;
    }

    

    // Calculate the linear and angular velocities
    // Get the speed based on the length of result vector (pythagoras)
    double result_magnitude = std::sqrt(result_vector[0] * result_vector[0] + result_vector[1] * result_vector[1]);
    // Get the angular speed based on the angle of result vector
    double result_angle = std::atan2(result_vector[1], result_vector[0]);

    // Initialize the twist message
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.angular.z = result_angle;

    if (min_distance < distance_threshold/2 && !(std::abs(result_angle) < alignment_threshold)) {
        // Robot is too close to the obstacle and too far from facing the right direction, stop linear motion
        twist_msg.linear.x = 0.0;
    } else {
        // Regular motion: Move forward and rotate
        twist_msg.linear.x = speed * result_magnitude;
    }
    
    publisher_->publish(twist_msg);
    RCLCPP_INFO(this->get_logger(), "linear vel: %f, ang vel: %f", twist_msg.linear.x, twist_msg.angular.z);

    // Publish markers for visualization
    publish_marker(target_vector, "target_vector", 0, 0.0, 0.0, 1.0); // Blue for target
    publish_marker(obstacle_vector, "obstacle_vector", 1, 1.0, 0.0, 0.0); // Red for obstacle
    publish_marker(result_vector, "result_vector", 2, 0.0, 1.0, 0.0); // Green for result
}
void AvoidanceNode::timer_callback()
{
    if (!laser_scan_) {
        RCLCPP_WARN(this->get_logger(), "Laser scan message not available.");
        return;
    }
    // Check if the laser scan message is older than 2 seconds
    auto now = this->now();
    auto scan_age = now - last_scan_time_;
    if (scan_age.seconds() > 2) {
        RCLCPP_WARN(this->get_logger(), "Laser scan data is older than 2 seconds.");
        return;
    }
    // Retrieve the current frequency parameter
    double current_frequency = this->get_parameter("frequency").as_double();

    // Check if the frequency parameter has changed
    if (current_frequency != frequency) {
        frequency = current_frequency;
        int period_in_milliseconds = 1000 / frequency;
            
        // Create a new timer with the updated frequency
        timer = this->create_wall_timer(
            std::chrono::milliseconds(period_in_milliseconds), 
            std::bind(&AvoidanceNode::timer_callback, this)
        );

    }
    
   
}
void AvoidanceNode::publish_marker(double vector[2], const std::string& ns, int id, float r, float g, float b) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1; // Shaft diameter
    marker.scale.y = 0.2; // Head diameter
    marker.scale.z = 0.2; // Head length

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    geometry_msgs::msg::Point start, end;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;
    end.x = vector[0];
    end.y = vector[1];
    end.z = 0.0;

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker_publisher_->publish(marker);
}

