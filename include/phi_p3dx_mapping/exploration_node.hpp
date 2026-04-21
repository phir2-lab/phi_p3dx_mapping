#ifndef PHI_P3DX_MAPPING__EXPLORATION_NODE_HPP_
#define PHI_P3DX_MAPPING__EXPLORATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <string>

/**
 * @brief Classe base para nós de exploração do ambiente.
 * 
 * Assina as mensagens de laser e do mapa, além de publicar para cmd_vel.
 */
class ExplorationNode : public rclcpp::Node
{
public:
  ExplorationNode(const std::string &node_name = "exploration_node", double timer_period = 0.1);
  virtual ~ExplorationNode() = default;

protected:
  // Publishers & Subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Dados do Laser
  std::vector<float> laser_ranges_;
  double laser_angle_min_;  
  double laser_angle_increment_; 
  
  // Mapa
  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;

  // Hooks para customização
  virtual void on_laser() {}
  virtual void on_map() {}
  virtual void control_loop() {}

  // Utilidades
  double get_front_distance(double half_angle_deg = 15.0) const;
  void publish_velocity(double v, double w);
  void stop();

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
};

#endif  // PHI_P3DX_MAPPING__EXPLORATION_NODE_HPP_
