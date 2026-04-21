import math
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry, OccupancyGrid

def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

class MappingNode(Node):
    """
    Classe base para nós de mapeamento em ROS2.
    Gerencia odometria e publicação de OccupancyGrid.
    """
    def __init__(self, node_name: str = 'mapping_node', timer_period: float = 0.1):
        super().__init__(node_name)

        # Estado do robô
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0

        # Mapa
        self.map_msg = OccupancyGrid()
        self.resolution = 0.05

        # Publisher do mapa
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        # Subscriber de odometria
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)

        # Timer
        self.create_timer(timer_period, self._control_loop)

        # Inicializa o mapa com 20x20 metros
        self.init_map(20.0, 20.0, 0.05)

        self.get_logger().info(f'[{node_name}] iniciado.')

    def init_map(self, map_width_m: float, map_height_m: float, resolution: float):
        self.resolution = resolution
        width_cells = int(map_width_m / self.resolution)
        height_cells = int(map_height_m / self.resolution)

        self.map_msg.header.frame_id = 'map'
        self.map_msg.info.resolution = self.resolution
        self.map_msg.info.width = width_cells
        self.map_msg.info.height = height_cells
        
        self.map_msg.info.origin.position.x = -map_width_m / 2.0
        self.map_msg.info.origin.position.y = -map_height_m / 2.0
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0

        self.map_msg.data = [-1] * (width_cells * height_cells)

    def _cb_odom(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.on_odom()

    def world_to_map(self, wx: float, wy: float) -> tuple:
        mx = int((wx - self.map_msg.info.origin.position.x) / self.resolution)
        my = int((wy - self.map_msg.info.origin.position.y) / self.resolution)
        return (mx, my)

    def publish_map(self) -> None:
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

    def on_odom(self) -> None:
        pass

    def _control_loop(self) -> None:
        pass
