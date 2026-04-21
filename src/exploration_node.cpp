#include "phi_p3dx_mapping/exploration_node.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

ExplorationNode::ExplorationNode(const std::string &node_name, double timer_period)
  : Node(node_name),
    laser_angle_min_(0.0), laser_angle_increment_(0.0),
    map_msg_(nullptr)
{
  // Publisher: comandos de velocidade
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Subscriber: scan laser
  auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "laser_scan", sensor_qos,
    std::bind(&ExplorationNode::laser_callback, this, std::placeholders::_1));

  // Subscriber: mapa global
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local();
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", map_qos,
    std::bind(&ExplorationNode::map_callback, this, std::placeholders::_1));

  // Timer para loop de controle
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period),
    std::bind(&ExplorationNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "[%s] iniciado", node_name.c_str());
}

void ExplorationNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  laser_ranges_ = msg->ranges;
  on_laser();
}

void ExplorationNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_msg_ = msg;
  on_map();
}

double ExplorationNode::get_front_distance(double half_angle_deg) const
{
  if (laser_ranges_.empty()) return std::numeric_limits<double>::infinity();

  size_t n = laser_ranges_.size();
  size_t center = n / 2;
  size_t half = static_cast<size_t>(std::abs(half_angle_deg * M_PI / 180.0) / std::max(laser_angle_increment_, 1e-9));
  
  size_t start = (center > half) ? center - half : 0;
  size_t end = std::min(center + half, n - 1);

  double min_dist = std::numeric_limits<double>::infinity();
  for (size_t i = start; i <= end; ++i) {
    if (laser_ranges_[i] < min_dist) {
      min_dist = laser_ranges_[i];
    }
  }
  return min_dist;
}

void ExplorationNode::publish_velocity(double v, double w)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = v;
  twist.angular.z = w;
  cmd_vel_pub_->publish(twist);
}

void ExplorationNode::stop()
{
  publish_velocity(0.0, 0.0);
}
