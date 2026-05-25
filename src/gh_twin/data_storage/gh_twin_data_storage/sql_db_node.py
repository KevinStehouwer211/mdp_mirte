import rclpy
from rclpy.node import Node


class SqlDbNode(Node):
    def __init__(self):
        super().__init__('sql_db_node')
        self.get_logger().info('SQL DB node placeholder. This node will later handle structured relational storage.')

    def main_loop(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = SqlDbNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
