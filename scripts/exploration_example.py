#!/usr/bin/env python3
"""
Exemplo simples de nodo de exploração.
"""
import rclpy
import random
from phi_p3dx_mapping.exploration_node import ExplorationNode

class ExplorationExample(ExplorationNode):
    def __init__(self):
        super().__init__(node_name='exploration_example_py')
        self.turning_ticks = 0
        self.turn_direction = 0.0

    def _control_loop(self) -> None:
        if not self.laser_ranges:
            return

        front_dist = self.get_front_distance(20.0)
        MIN_DIST = 0.7

        if self.turning_ticks > 0:
            self.publish_velocity(0.0, self.turn_direction)
            self.turning_ticks -= 1
        else:
            if front_dist < MIN_DIST:
                self.turning_ticks = random.randint(5, 20)
                self.turn_direction = 0.4 if random.choice([True, False]) else -0.4
                self.get_logger().info(f'Obstáculo detectado a frente ({front_dist:.2f}m). Virando...')
                self.publish_velocity(0.0, self.turn_direction)
            else:
                self.publish_velocity(0.25, 0.0)

    def on_map(self) -> None:
        if self.map_msg:
            self.get_logger().info(
                f'Mapa recebido! Dimensões: {self.map_msg.info.width}x{self.map_msg.info.height}, '
                f'Resolução: {self.map_msg.info.resolution:.2f}',
                once=True
            )

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
