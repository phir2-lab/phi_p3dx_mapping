import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


class ExplorationNode(Node):
    """
    Classe base para nós de exploração em ROS2.
    Assina mensagens de laser e public a /cmd_vel.
    """
    def __init__(self, node_name: str = 'exploration_node', timer_period: float = 0.1):
        super().__init__(node_name)

        self.laser_ranges = []
        self.laser_angle_min = 0.0
        self.laser_angle_increment = 0.0

        self.map_msg = None

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(
            LaserScan, '/laser_scan', self._cb_laser,
            qos_profile=qos_profile_sensor_data)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(OccupancyGrid, '/map', self._cb_map, map_qos)

        self.create_timer(timer_period, self._control_loop)
        self.get_logger().info(f'[{node_name}] iniciado.')

    def _cb_laser(self, msg: LaserScan) -> None:
        self.laser_ranges = [
            r if msg.range_min <= r <= msg.range_max else float('inf')
            for r in msg.ranges
        ]
        self.laser_angle_min = msg.angle_min
        self.laser_angle_increment = msg.angle_increment
        self.on_laser()

    def _cb_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.on_map()

    def get_front_distance(self, half_angle_deg: float = 15.0) -> float:
        if not self.laser_ranges:
            return float('inf')
        
        n = len(self.laser_ranges)
        center = n // 2
        half = int(math.radians(half_angle_deg) / max(self.laser_angle_increment, 1e-9))
        
        start = max(0, center - half)
        end = min(n - 1, center + half)
        
        region = self.laser_ranges[start : end + 1]
        return min(region) if region else float('inf')

    def publish_velocity(self, v: float, w: float) -> None:
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.pub_cmd.publish(twist)

    def stop(self) -> None:
        self.publish_velocity(0.0, 0.0)

    def on_laser(self) -> None:
        pass

    def on_map(self) -> None:
        pass

    def _control_loop(self) -> None:
        pass
