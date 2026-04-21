#ifndef PHI_P3DX_MAPPING__MAPPING_NODE_HPP_
#define PHI_P3DX_MAPPING__MAPPING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tuple>
#include <string>

/**
 * @brief Classe base para nós de mapeamento em ROS2.
 * 
 * Gerencia a subscrição de odometria e publicação do mapa (OccupancyGrid).
 */
class MappingNode : public rclcpp::Node
{
public:
  MappingNode(const std::string &node_name = "mapping_node", double timer_period = 0.1);
  virtual ~MappingNode() = default;

protected:
  // Publishers & Subscribers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Estado do robô
  double x_, y_, theta_;

  // Mapa
  nav_msgs::msg::OccupancyGrid map_msg_;
  double resolution_;
  int origin_x_cells_;
  int origin_y_cells_;

  // Hooks para customização
  virtual void on_odom() {}

  // Métodos úteis para manipulação de mapa
  void init_map(double map_width_m, double map_height_m, double resolution);
  std::tuple<int, int> meters_to_cells(double wx, double wy) const;
  void publish_map();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif  // PHI_P3DX_MAPPING__MAPPING_NODE_HPP_
