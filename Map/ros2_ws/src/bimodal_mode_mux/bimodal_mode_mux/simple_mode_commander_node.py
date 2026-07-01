import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class SimpleModeCommander(Node):
    def __init__(self):
        super().__init__('simple_mode_commander_node')
        self.declare_parameter('enable_auto_switch', True)
        self.declare_parameter('initial_mode', 'AIR')
        self.declare_parameter('switch_period_sec', 20.0)
        self.declare_parameter('publish_period_sec', 1.0)
        self.modes = ['AIR', 'GROUND', 'IDLE']
        initial_mode = str(self.get_parameter('initial_mode').value).strip().upper()
        if initial_mode not in self.modes:
            self.get_logger().warn(f'Invalid initial_mode={initial_mode}, falling back to AIR')
            initial_mode = 'AIR'
        self.index = self.modes.index(initial_mode)
        self.current_mode = initial_mode

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(String, '/bimodal/active_mode', qos)

        publish_period = float(self.get_parameter('publish_period_sec').value)
        switch_period = float(self.get_parameter('switch_period_sec').value)
        self.publish_timer = self.create_timer(max(publish_period, 0.1), self._publish_current_mode)
        self.switch_timer = None
        if bool(self.get_parameter('enable_auto_switch').value):
            self.switch_timer = self.create_timer(max(switch_period, 1.0), self._switch)

        self._publish_current_mode()

    def _publish_current_mode(self):
        msg = String()
        msg.data = self.current_mode
        self.pub.publish(msg)

    def _switch(self):
        self.index = (self.index + 1) % len(self.modes)
        self.current_mode = self.modes[self.index]
        self._publish_current_mode()
        self.get_logger().info(f'active_mode switched to {self.current_mode}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleModeCommander()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
