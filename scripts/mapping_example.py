#!/usr/bin/env python3
"""
Exemplo simples de mapeamento iterativo.
"""
import rclpy
from phi_p3dx_mapping.mapping_node import MappingNode

class MappingExample(MappingNode):
    def __init__(self):
        super().__init__(node_name='mapping_example_py')

    def _control_loop(self) -> None:
        pass

    def on_odom(self) -> None:
        # Marca um raio de 0.5m como área livre ao redor do robô
        self.mark_free_space(self.x, self.y, 0.5)
        self.publish_map()

    def mark_free_space(self, wx: float, wy: float, radius_m: float):
        r_cells = int(radius_m / self.resolution)
        mx, my = self.world_to_map(wx, wy)

        w = self.map_msg.info.width
        h = self.map_msg.info.height

        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                nx = mx + dx
                ny = my + dy

                if 0 <= nx < w and 0 <= ny < h:
                    if dx*dx + dy*dy <= r_cells * r_cells:
                        idx = ny * w + nx
                        self.map_msg.data[idx] = 0

def main(args=None):
    rclpy.init(args=args)
    node = MappingExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
