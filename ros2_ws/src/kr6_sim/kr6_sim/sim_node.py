"""sim_node — skeleton (GATE R0). Real logic lands at its gate per PLAN_ROS.md."""
import rclpy
from rclpy.node import Node


class Sim_node(Node):  # placeholder class name, refined per gate
    def __init__(self):
        super().__init__('sim_node')
        self.get_logger().info('sim_node up (skeleton)')


def main(args=None):
    rclpy.init(args=args)
    node = Sim_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
