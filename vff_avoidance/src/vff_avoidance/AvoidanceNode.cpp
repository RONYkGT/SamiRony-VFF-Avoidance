#include "../../include/vff_avoidance/AvoidanceNode.hpp"

using namespace std::chrono_literals;

AvoidanceNode::AvoidanceNode()
: Node("obstacle_avoidance")
{
    distance_threshold = 1.0;
    k_obstacle = -1.0;
    RCLCPP_INFO(this->get_logger(), "robot initialized");
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&AvoidanceNode::scan_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

}


void AvoidanceNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    double min_distance = distance_threshold;
    double target_vector[2] = {1,0};
    double obstacle_vector[2];
    size_t index;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      if(msg->ranges[i] < min_distance)
      {
        min_distance = msg->ranges[i];
        index = i;
      }
    }
    double angle = msg->angle_min + index * msg->angle_increment;
    RCLCPP_INFO(this->get_logger(), "min_dist: %f, index: %d", min_distance, index);
    double x = k_obstacle * (1/msg->ranges[index]) * cos(angle);
    double y = k_obstacle * (1/msg->ranges[index]) * sin(angle);
    obstacle_vector[0] = x;
    obstacle_vector[1] = y;

    double result_vector[2];
    result_vector[0] = target_vector[0] + obstacle_vector[0];
    result_vector[1] = target_vector[1] + obstacle_vector[1];

    publish_marker(target_vector, "target_vector", 0, 1.0, 0.0, 0.0); // Red for target
    publish_marker(obstacle_vector, "obstacle_vector", 1, 0.0, 1.0, 0.0); // Green for obstacle
    publish_marker(result_vector, "result_vector", 2, 0.0, 0.0, 1.0); // Blue for result

    
}

void AvoidanceNode::publish_marker(double vector[2], const std::string& ns, int id, float r, float g, float b) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "base_link"; // Set this to your robot's frame
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AvoidanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
